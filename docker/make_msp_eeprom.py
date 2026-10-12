#!/usr/bin/env python3
"""Derive an MSP-receiver eeprom from the SIM-receiver one the image ships.

The image seeds /data/eeprom.bin from a template baked in at build time. That
template has receiver_type = SIM, which is what makes the SITL build take its
RC from the UE datarefs. An algorithm sending sticks over MSP needs the other
receiver type, and there are only two ways to get there: run the CLI, or ship a
second template. This produces the second template.

Why not just poke the byte: every record is covered by a CRC16 that the
firmware verifies before it will load anything (isEEPROMContentValid). A file
with a stale checksum is not "mostly right" - it fails the check, the firmware
falls back to resetEEPROM(), and you get factory defaults, which is precisely
the silent failure this whole directory exists to avoid. So the checksum is
recomputed here, and --verify exists to prove the understanding of the layout
is right before anything is written.

Layout, from config/config_eeprom.c:

    [1B format][record...][2B terminator=0][2B crc16]

and each record:

    [2B size][2B pgn][1B version][1B flags][payload]

size counts the 6-byte record header plus the payload, so it is also the stride
to the next record. flags' low two bits pick the profile; 0 means system, i.e.
a single instance rather than one per profile - rxConfig is PG_DECLARE (not
_ARRAY), so it is stored once with flags = 0.

receiver_type is the first member of rxConfig_t, so it is the first payload
byte of the record whose pgn is PG_RX_CONFIG.

--set-gyro-lpf exists for a SITL-only problem that has to be worked around in
configuration: gyroFilter() has exactly one call site, fc_core.c:955, inside
the SITL_BUILD lockMainPID() gate -- so it runs once per simulator frame, while
the PT1 coefficient it applies was computed for getLooptime() = 500 us
(gyro.c:285). pt1FilterUpdateCutoff never corrects the stored dT (filter.c:74),
so a nominal 60 Hz low pass becomes ~1.65 Hz at 60 fps and ~0.69 Hz at 25 fps.
On an airframe with no rotational damping that is 57-75 degrees of phase lag
added right where the rate loop needs margin. Setting gyro_main_lpf_hz to 0
makes initGyroFilter take the nullFilterApply branch (gyro.c:257). There is no
MSP command for gyro config -- the CLI is the only other way in, and the CLI
cannot be reached during a session (entering it latches ARMING_DISABLED_CLI).

--set-motor-direction-inverted exists for the opposite reason: it is not a
workaround but an alignment. The UE plugin models each rotor's yaw reaction and
has to match what the firmware's mixer actually does, and INAV folds
motor_direction_inverted into that (mixer.c:312):

    signYaw = (rcCommand[YAW] < 0 ? 1 : -1) * (motorDirectionInverted ? 1 : -1)

The real GEPRC F722 has it ON; a fresh SITL container has the factory OFF, so
the plugin cannot agree with both. Mismatched, INAV's yaw loop drives the
airframe the wrong way and the only symptom is that yaw runs away - no partial
failure. Aligning SITL to ON lets one plugin setting serve both.

--set-gps-provider exists because FEATURE_GPS cannot be turned on by itself.
gpsInit() (io/gps.c:461) clears the bit outright when the selected provider is
serial-based and no port carries FUNCTION_GPS, and that is every SITL container:
the simulator calls gpsFakeSet() directly instead of feeding a receiver. UBLOX
is serial-based, so the bit is cleared a few lines after the eeprom is loaded
and no value the eeprom can hold survives to runtime - which is why the shipped
templates all read GPS off and why turning the bit on changed nothing. FAKE is
driver-based, so gpsInit() returns before the clear. Setting the bit without
this is a no-op, and so is this without the bit.

The failure mode is silent in both directions: the sim link reports healthy, the
aircraft flies, and posEstimator.gps.* is simply never refreshed, because
gpsUpdate() returns early on the missing feature.

Usage:
    make_msp_eeprom.py --verify eeprom.default.bin
    make_msp_eeprom.py --set-receiver MSP -o eeprom.msp.bin eeprom.default.bin
    make_msp_eeprom.py --set-gyro-lpf 0 -o out.bin eeprom.msp.bin
    make_msp_eeprom.py --set-motor-direction-inverted ON -o out.bin in.bin
    make_msp_eeprom.py --set-gps-provider FAKE --set-feature GPS=ON -o out.bin in.bin
"""

import argparse
import struct
import sys

EEPROM_CONF_VERSION = 126
# From src/main/config/parameter_group_ids.h, and the versions from their
# PG_REGISTER_WITH_RESET_TEMPLATE calls.
PG_RX_CONFIG = 24
PG_RX_CONFIG_VERSION = 13
PG_GYRO_CONFIG = 10
PG_GYRO_CONFIG_VERSION = 12
PG_MIXER_PROFILE = 20
# From PG_REGISTER_ARRAY_WITH_RESET_FN(mixerProfile_t, ...), mixer_profile.c:40.
PG_MIXER_PROFILE_VERSION = 1
# PG_FEATURE_CONFIG is 19 (parameter_group_ids.h:38); the version is the last
# argument of PG_REGISTER_WITH_RESET_TEMPLATE(featureConfig_t, featureConfig, ...).
PG_FEATURE_CONFIG = 19
PG_FEATURE_CONFIG_VERSION = 0
# featureConfig_t is a single uint32_t (fc/config.h). That makes this the one
# record with nothing inside its payload to check against a factory value, so
# the size does the work instead: 6 bytes of header plus exactly 4.
FEATURE_CONFIG_PAYLOAD_SIZE = 4

CR_CLASSIFICATION_MASK = 0x3
CR_CLASSIFICATION_SYSTEM = 0

RX_TYPE = {"NONE": 0, "SERIAL": 1, "MSP": 2, "SIM": 3}
RX_TYPE_NAME = {v: k for k, v in RX_TYPE.items()}

# rxConfig_t.rcmap's factory value, used below to confirm the record we found
# really is rxConfig and not a pgn collision.
EXPECTED_RCMAP = bytes([0, 1, 3, 2])

# gyroConfig_t starts looptime, gyro_anti_aliasing_lpf_hz, then gyro_main_lpf_hz
# -- with no gyro_to_use in between, because SITL leaves USE_DUAL_GYRO undefined
# (it is the one field that #ifdef can move). That absence is worth checking
# rather than assuming: if a future SITL target did enable dual gyro, offset 4
# would become the uint8 gyro_to_use and this offset would silently point one
# byte into the wrong field. The proof is that the two fields before it read
# their factory values -- gyro_to_use could only ever be 0 or 1, so a 500 there
# means the offset is where this comment says it is.
EXPECTED_LOOPTIME = 500
EXPECTED_ANTI_ALIASING_LPF_HZ = 250
GYRO_MAIN_LPF_OFFSET = 4

# mixerConfig_t.motorDirectionInverted is int8_t and leads the struct
# (mixer_profile.h:13), and mixerConfig_t leads mixerProfile_t, so it is the
# first payload byte of the PG_MIXER_PROFILE record. motorMixer_t is four
# floats, and MotorMixers[] follows the 20-byte mixerConfig_t -- so the mmix
# values sit at payload+20 and are the strongest proof of where the config
# ends: a struct that grew a member moves them and the unpack stops reading 1.0
# and the +/-0.707 quad-X column values.
MOTOR_DIRECTION_INVERTED_OFFSET = 0
MIXER_CONFIG_SIZE = 20
PLATFORM_MULTIROTOR = 0
# SETTING_MODEL_PREVIEW_TYPE_DEFAULT. Set from the reset fn, not reachable from
# the CLI (mixer_profile.c:51), so it is a stable factory value to check.
EXPECTED_MODEL_PREVIEW_TYPE = -1
# MAX_SUPPORTED_MOTORS is 8 on this target; only the four a quad-X uses are
# checked, since a motor the mixer leaves at 0.0 fails the throttle test anyway.
QUAD_MOTORS = 4
MOTOR_DIRECTION_INVERTED = {"OFF": 0, "ON": 1}
MOTOR_DIRECTION_INVERTED_NAME = {v: k for k, v in MOTOR_DIRECTION_INVERTED.items()}

# fc/cli.c:161 featureNames[] -- the table is indexed by bit number, so an entry's
# position *is* its shift, which is why this maps names to bits rather than to
# masks. The unused slots ("") are left out rather than kept as placeholders so a
# typo fails as an unknown name instead of clearing some other feature's bit.
FEATURE_BIT = {
    "THR_VBAT_COMP": 0, "VBAT": 1, "TX_PROF_SEL": 2, "BAT_PROF_AUTOSWITCH": 3,
    "GEOZONE": 4, "SOFTSERIAL": 6, "GPS": 7, "RPM_FILTERS": 8,
    "TELEMETRY": 10, "CURRENT_METER": 11, "REVERSIBLE_MOTORS": 12, "RSSI_ADC": 15,
    "LED_STRIP": 16, "DASHBOARD": 17, "BLACKBOX": 19, "TRANSPONDER": 21,
    "AIRMODE": 22, "SUPEREXPO": 23, "VTX": 24, "PWM_OUTPUT_ENABLE": 28,
    "OSD": 29, "FW_LAUNCH": 30, "FW_AUTOTRIM": 31,
}
FEATURE_ON_OFF = {"OFF": 0, "ON": 1}

# PG_GPS_CONFIG is 30, from PG_REGISTER_WITH_RESET_TEMPLATE(gpsConfig_t,
# gpsConfig, PG_GPS_CONFIG, 5) at io/gps.c:120.
PG_GPS_CONFIG = 30
PG_GPS_CONFIG_VERSION = 5
# gpsConfig_t's five leading members are all enums (io/gps.h:94), and every one
# of them is an int in this ABI, so provider sits in the first four payload
# bytes with nothing packing ahead of it.
GPS_PROVIDER_OFFSET = 0
GPS_PROVIDER = {"UBLOX": 0, "MSP": 1, "FAKE": 2}
GPS_PROVIDER_NAME = {v: k for k, v in GPS_PROVIDER.items()}

# The four enum members behind provider, with the ranges their definitions allow
# (io/gps.h:42-90). Reading all four of them in range is what makes a 4-byte
# enum stride believable; if the ABI ever packed them narrower, provider would
# still read 0 and every check here would fail, which is the failure worth
# having.
SBAS_MODE_MAX = 6
DYN_MODEL_MAX = 6
GPS_PROVIDER_MAX = len(GPS_PROVIDER) - 1


def crc16_ccitt(crc, byte):
    """One byte of CRC16-CCITT, matching common/crc.c:crc16_ccitt.

    The C version shifts a uint16_t, so every step truncates to 16 bits; the
    masks here are that truncation, not decoration.
    """
    crc ^= byte << 8
    for _ in range(8):
        if crc & 0x8000:
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF
        else:
            crc = (crc << 1) & 0xFFFF
    return crc


def crc16(data, crc=0):
    for b in data:
        crc = crc16_ccitt(crc, b)
    return crc


def u16(data, offset):
    return data[offset] | (data[offset + 1] << 8)


def u32(data, offset):
    return struct.unpack_from("<I", data, offset)[0]


def i16(data, offset):
    value = u16(data, offset)
    return value - 0x10000 if value & 0x8000 else value


def parse(data):
    """Return (records, footer_offset, stored_crc, computed_crc).

    records is a list of (offset, size, pgn, version, flags). Raises ValueError
    if the stream does not parse - a malformed stream and a bad checksum are
    different failures and are reported as such.
    """
    if len(data) < 3:
        raise ValueError("file is too short to hold a config header")

    if data[0] != EEPROM_CONF_VERSION:
        raise ValueError(
            "format byte is %d, expected EEPROM_CONF_VERSION %d - this file is "
            "not an eeprom for this firmware version" % (data[0], EEPROM_CONF_VERSION)
        )

    records = []
    crc = crc16(data[0:1])
    p = 1

    while True:
        if p + 6 > len(data):
            raise ValueError("ran off the end of the file looking for a record")

        size = u16(data, p)
        if size == 0:
            break  # terminator reached

        if size < 6:
            raise ValueError("record at %d has size %d, smaller than its own header" % (p, size))
        if p + size > len(data):
            raise ValueError("record at %d claims %d bytes, past end of file" % (p, size))

        records.append((p, size, u16(data, p + 2), data[p + 4], data[p + 5]))
        crc = crc16(data[p:p + size], crc)
        p += size

    footer_offset = p
    if p + 4 > len(data):
        raise ValueError("ran off the end of the file looking for the footer")

    crc = crc16(data[p:p + 2], crc)  # terminator is covered by the CRC
    p += 2
    stored = u16(data, p)

    return records, footer_offset, stored, crc


def require_valid(data):
    """Refuse to build on a source whose checksum does not check out.

    The setters recompute the checksum, so running one over a corrupt file would
    launder bad bytes into a file the firmware happily accepts. A bad source has
    to be a hard stop, not something the output quietly repairs.
    """
    _, _, stored, computed = parse(data)
    if stored != computed:
        raise ValueError(
            "source checksum is stored 0x%04X but computes to 0x%04X - the source "
            "is corrupt, and recomputing over it would produce a valid-looking "
            "file built on bad bytes" % (stored, computed))


def find_system(data, pgn, version):
    """The single system record with this pgn, as (offset, size, payload).

    Raises unless there is exactly one and its version matches. Both checks earn
    their keep. "System" is the low two bits of flags being 0: a profile-scoped
    record would appear once per profile, and which one is live depends on
    runtime state, so its bytes are not a fixed answer. The version is the one
    in PG_REGISTER_WITH_RESET_TEMPLATE, and it is the check that catches the
    real hazard -- a firmware change that reshapes the struct bumps it, which is
    exactly when a hardcoded member offset goes silently wrong.
    """
    records, _, _, _ = parse(data)
    hits = [(o, s, v, data[o + 6:o + s])
            for o, s, p, v, f in records
            if p == pgn and (f & CR_CLASSIFICATION_MASK) == CR_CLASSIFICATION_SYSTEM]
    if len(hits) != 1:
        raise ValueError("expected exactly one system pgn %d record, found %d"
                         % (pgn, len(hits)))

    offset, size, found, payload = hits[0]
    if found != version:
        raise ValueError(
            "pgn %d at 0x%04X has version %d, expected %d - the struct changed "
            "shape, so the member offsets hardcoded in this script are stale"
            % (pgn, offset, found, version))
    return offset, size, payload


def feature_payload(data):
    """The validated featureConfig_t payload and its file offset."""
    offset, size, payload = find_system(data, PG_FEATURE_CONFIG, PG_FEATURE_CONFIG_VERSION)

    if size != 6 + FEATURE_CONFIG_PAYLOAD_SIZE:
        raise ValueError(
            "PG_FEATURE_CONFIG at 0x%04X is %d bytes, expected %d - featureConfig_t "
            "is not a single uint32_t in this build, so its bits cannot be trusted"
            % (offset, size, 6 + FEATURE_CONFIG_PAYLOAD_SIZE))
    return offset, payload


def rx_payload(data):
    """The validated rxConfig_t payload and its file offset."""
    offset, _, payload = find_system(data, PG_RX_CONFIG, PG_RX_CONFIG_VERSION)

    rcmap = payload[1:5]
    if rcmap != EXPECTED_RCMAP:
        raise ValueError(
            "record at 0x%04X has rcmap %s, expected %s - the payload does not "
            "look like rxConfig_t, so the byte offset of receiver_type cannot be "
            "trusted" % (offset, list(rcmap), list(EXPECTED_RCMAP))
        )
    return offset, payload


def gyro_payload(data):
    """The validated gyroConfig_t payload and its file offset."""
    offset, _, payload = find_system(data, PG_GYRO_CONFIG, PG_GYRO_CONFIG_VERSION)

    looptime = u16(payload, 0)
    anti_aliasing = u16(payload, 2)
    if (looptime, anti_aliasing) != (EXPECTED_LOOPTIME, EXPECTED_ANTI_ALIASING_LPF_HZ):
        raise ValueError(
            "PG_GYRO_CONFIG at 0x%04X reads looptime=%d anti_aliasing=%d, expected "
            "%d/%d - gyroConfig_t's leading fields are not where this script "
            "thinks they are, so the offset of gyro_main_lpf_hz cannot be trusted"
            % (offset, looptime, anti_aliasing, EXPECTED_LOOPTIME,
               EXPECTED_ANTI_ALIASING_LPF_HZ))
    return offset, payload


def mixer_payload(data):
    """The validated mixerProfile_t payload and its file offset.

    This is a PG_DECLARE_ARRAY record, and the instance stored is profile 0 --
    whose flags are 0, which also reads as CR_CLASSIFICATION_SYSTEM under the
    low-two-bits mask. That coincidence is why find_system accepts it, and why
    there is exactly one: a differently configured profile 1 would show up as a
    second record with flags 1, and profile 0 is still the live one.
    """
    offset, _, payload = find_system(data, PG_MIXER_PROFILE, PG_MIXER_PROFILE_VERSION)

    platform = payload[1]
    preset = i16(payload, 4)
    if platform != PLATFORM_MULTIROTOR or preset != EXPECTED_MODEL_PREVIEW_TYPE:
        raise ValueError(
            "PG_MIXER_PROFILE at 0x%04X reads platformType=%d appliedMixerPreset=%d, "
            "expected %d/%d - mixerConfig_t's leading fields are not where this "
            "script thinks they are, so the offset of motorDirectionInverted cannot "
            "be trusted"
            % (offset, platform, preset, PLATFORM_MULTIROTOR,
               EXPECTED_MODEL_PREVIEW_TYPE))

    for i in range(QUAD_MOTORS):
        thr, roll, pitch, yaw = struct.unpack_from(
            "<4f", payload, MIXER_CONFIG_SIZE + 16 * i)
        if abs(thr - 1.0) > 1e-3 or max(abs(roll), abs(pitch), abs(yaw)) > 1.001:
            raise ValueError(
                "PG_MIXER_PROFILE at 0x%04X has mmix %d = (%.4f, %.4f, %.4f, %.4f) at "
                "payload+%d, which is not a mixer row (throttle 1.0, axes within "
                "+/-1) - mixerConfig_t is not %d bytes here, so the mmix check that "
                "pins motorDirectionInverted's offset has failed"
                % (offset, i, thr, roll, pitch, yaw, MIXER_CONFIG_SIZE + 16 * i,
                   MIXER_CONFIG_SIZE))
    return offset, payload


def gps_payload(data):
    """The validated gpsConfig_t payload and its file offset.

    provider is the field that matters, and the four enums behind it are what
    make it believable: sbasMode, autoConfig, autoBaud and dynModel are each
    constrained by their own definitions, so a stride that was not four bytes
    would push at least one of them out of range. Unlike the structs checked
    above there is no factory value to compare against, because autoConfig and
    autoBaud are legitimately 0 or 1 and both ends are in range.
    """
    offset, _, payload = find_system(data, PG_GPS_CONFIG, PG_GPS_CONFIG_VERSION)

    provider = u32(payload, GPS_PROVIDER_OFFSET)
    sbas = u32(payload, 4)
    auto_config = u32(payload, 8)
    auto_baud = u32(payload, 12)
    dyn_model = u32(payload, 16)
    if (provider > GPS_PROVIDER_MAX or sbas > SBAS_MODE_MAX
            or auto_config > 1 or auto_baud > 1 or dyn_model > DYN_MODEL_MAX):
        raise ValueError(
            "PG_GPS_CONFIG at 0x%04X reads provider=%d sbasMode=%d autoConfig=%d "
            "autoBaud=%d dynModel=%d - at least one is outside its own enum, so "
            "gpsProvider_e is not a four-byte member here and the provider offset "
            "cannot be trusted"
            % (offset, provider, sbas, auto_config, auto_baud, dyn_model))
    return offset, payload


def describe(data):
    """Print the layout and the two validated fields, or raise explaining why not.

    Raw bytes are printed before anything is interpreted: on a file that turns
    out to be corrupt the table is the useful output, and interpreting a
    checksum-failed file would be the exact silent-wrongness this tool exists to
    prevent.
    """
    records, footer_offset, stored, computed = parse(data)

    print("format        : %d" % data[0])
    print("records       : %d" % len(records))
    print("footer at     : 0x%04X" % footer_offset)
    print("config size   : %d bytes (file is %d)" % (footer_offset + 4, len(data)))
    print("checksum      : stored 0x%04X, computed 0x%04X  %s"
          % (stored, computed, "OK" if stored == computed else "MISMATCH"))
    print()

    for offset, size, pgn, version, flags in records:
        payload = data[offset + 6:offset + size]
        note = ""
        if pgn == PG_RX_CONFIG and (flags & CR_CLASSIFICATION_MASK) == CR_CLASSIFICATION_SYSTEM:
            note = "  <- PG_RX_CONFIG, receiver_type = %s (%d)" % (
                RX_TYPE_NAME.get(payload[0], "?"), payload[0])
        elif pgn == PG_GYRO_CONFIG and (flags & CR_CLASSIFICATION_MASK) == CR_CLASSIFICATION_SYSTEM:
            note = "  <- PG_GYRO_CONFIG, gyro_main_lpf_hz = %d" % u16(payload, GYRO_MAIN_LPF_OFFSET)
        elif pgn == PG_MIXER_PROFILE and (flags & CR_CLASSIFICATION_MASK) == CR_CLASSIFICATION_SYSTEM:
            note = "  <- PG_MIXER_PROFILE (profile 0), motorDirectionInverted = %s (%d)" % (
                MOTOR_DIRECTION_INVERTED_NAME.get(payload[0], "?"), payload[0])
        print("  0x%04X  size=%-5d pgn=%-4d ver=%-3d flags=%d%s"
              % (offset, size, pgn, version, flags, note))

    print()
    require_valid(data)

    offset, payload = rx_payload(data)
    print("receiver_type : %s (%d) at file offset 0x%04X"
          % (RX_TYPE_NAME.get(payload[0], "?"), payload[0], offset + 6))
    print("rcmap         : %s (AETR - confirms this is rxConfig_t)"
          % list(payload[1:5]))

    offset, payload = gyro_payload(data)
    print("gyro          : looptime=%d anti_aliasing=%d (both factory values, so "
          "the layout is confirmed)" % (u16(payload, 0), u16(payload, 2)))
    print("gyro_main_lpf : %d Hz at file offset 0x%04X"
          % (u16(payload, GYRO_MAIN_LPF_OFFSET), offset + 6 + GYRO_MAIN_LPF_OFFSET))

    offset, payload = mixer_payload(data)
    print("motor_dir_inv : %s (%d) at file offset 0x%04X"
          % (MOTOR_DIRECTION_INVERTED_NAME.get(payload[0], "?"), payload[0],
             offset + 6 + MOTOR_DIRECTION_INVERTED_OFFSET))
    print("mixer         : platformType=%d appliedMixerPreset=%d, mmix0=(%.3f, %.3f, "
          "%.3f, %.3f) at payload+%d (a quad-X row, so mixerConfig_t's size is "
          "confirmed)"
          % tuple([payload[1], i16(payload, 4)]
                  + list(struct.unpack_from("<4f", payload, MIXER_CONFIG_SIZE))
                  + [MIXER_CONFIG_SIZE]))

    offset, payload = gps_payload(data)
    provider = u32(payload, GPS_PROVIDER_OFFSET)
    print("gps_provider  : %s (%d) at file offset 0x%04X"
          % (GPS_PROVIDER_NAME.get(provider, "?"), provider,
             offset + 6 + GPS_PROVIDER_OFFSET))

    offset, payload = feature_payload(data)
    mask = u32(payload, 0)
    enabled = ", ".join(name for name, bit
                        in sorted(FEATURE_BIT.items(), key=lambda kv: kv[1])
                        if mask & (1 << bit))
    print("features      : %s (mask 0x%08X at file offset 0x%04X)"
          % (enabled or "(none)", mask, offset + 6))
    print("note          : this is the eeprom's own answer and it overrides "
          "DEFAULT_FEATURES,\n"
          "                so a feature target.h asks for can still be off here")


def refoot(out):
    """Recompute the checksum over header..footer and return immutable bytes.

    Everything from the header up to and including the footer is covered; the
    checksum itself is not, which is why the footer offset is recovered from a
    fresh parse rather than assumed.
    """
    _, footer_offset, _, _ = parse(bytes(out))
    crc = crc16(out[0:footer_offset + 2])
    out[footer_offset + 2] = crc & 0xFF
    out[footer_offset + 3] = (crc >> 8) & 0xFF
    print("checksum      : -> 0x%04X" % crc)
    return bytes(out)


def set_receiver(data, value):
    """Return data with receiver_type set and the checksum recomputed.

    Returns the input unchanged, having said so, when it already holds the
    value. The setters chain, so neither has to know whether the other ran.
    """
    require_valid(data)
    offset, payload = rx_payload(data)
    current = payload[0]

    if current == value:
        print("\nreceiver_type is already %s; nothing to change."
              % RX_TYPE_NAME.get(value, value))
        return data

    out = bytearray(data)
    out[offset + 6] = value

    print("\nreceiver_type : %s (%d) -> %s (%d)"
          % (RX_TYPE_NAME.get(current, "?"), current,
             RX_TYPE_NAME.get(value, "?"), value))
    return refoot(out)


def set_gyro_lpf(data, hz):
    """Return data with gyro_main_lpf_hz set and the checksum recomputed.

    0 is the interesting value. It makes initGyroFilter take the nullFilterApply
    branch (gyro.c:257), which is the only way out of a filter whose coefficient
    was computed for the 500 us looptime but whose one call site runs per
    simulator frame. See the module docstring for the whole chain.
    """
    require_valid(data)
    offset, payload = gyro_payload(data)
    current = u16(payload, GYRO_MAIN_LPF_OFFSET)

    if current == hz:
        print("\ngyro_main_lpf_hz is already %d; nothing to change." % hz)
        return data

    at = offset + 6 + GYRO_MAIN_LPF_OFFSET
    out = bytearray(data)
    out[at] = hz & 0xFF
    out[at + 1] = (hz >> 8) & 0xFF

    print("\ngyro_main_lpf_hz : %d -> %d" % (current, hz))
    return refoot(out)


def set_motor_direction_inverted(data, value):
    """Return data with motorDirectionInverted set and the checksum recomputed.

    This one is an alignment, not a workaround: see the module docstring. It
    changes only how the firmware mixes yaw against the motor laydown, so it is
    the setting the UE plugin's MotorYawSign has to agree with.
    """
    require_valid(data)
    offset, payload = mixer_payload(data)
    current = payload[0]

    if current == value:
        print("\nmotor_direction_inverted is already %s; nothing to change."
              % MOTOR_DIRECTION_INVERTED_NAME.get(value, value))
        return data

    out = bytearray(data)
    out[offset + 6 + MOTOR_DIRECTION_INVERTED_OFFSET] = value

    print("\nmotor_direction_inverted : %s (%d) -> %s (%d)"
          % (MOTOR_DIRECTION_INVERTED_NAME.get(current, "?"), current,
             MOTOR_DIRECTION_INVERTED_NAME.get(value, "?"), value))
    return refoot(out)


def set_gps_provider(data, value):
    """Return data with gps_provider set and the checksum recomputed.

    This is the half of the GPS fix that has to live here rather than in the
    feature mask; the module docstring has the whole chain. Setting one without
    the other is a no-op at runtime either way.
    """
    require_valid(data)
    offset, payload = gps_payload(data)
    current = u32(payload, GPS_PROVIDER_OFFSET)

    if current == value:
        print("\ngps_provider is already %s; nothing to change."
              % GPS_PROVIDER_NAME.get(value, value))
        return data

    out = bytearray(data)
    struct.pack_into("<I", out, offset + 6 + GPS_PROVIDER_OFFSET, value)

    print("\ngps_provider : %s (%d) -> %s (%d)"
          % (GPS_PROVIDER_NAME.get(current, "?"), current,
             GPS_PROVIDER_NAME.get(value, "?"), value))
    return refoot(out)


def set_features(data, wanted):
    """Return data with the named features on or off, and the checksum recomputed.

    Only the named bits move. The rest of the mask is left byte-for-byte as it
    was found, because this template also carries settings the image depends on
    -- turning GPS on the way the CLI would, by starting from factory defaults
    and re-enabling the interesting features, is how a SITL image ends up with
    receiver_type reset and no way to arm.

    A feature whose bit is already in the asked-for state is reported as such and
    left alone, which makes the setter safe to re-run.
    """
    require_valid(data)
    offset, payload = feature_payload(data)
    before = u32(payload, 0)
    mask = before

    for name in wanted:
        if name not in FEATURE_BIT:
            raise ValueError("unknown feature %r (known: %s)"
                             % (name, ", ".join(sorted(FEATURE_BIT))))

    changes = []
    for name, value in wanted.items():
        bit = 1 << FEATURE_BIT[name]
        updated = (mask | bit) if value else (mask & ~bit)
        if updated != mask:
            changes.append("%s %s" % (name, "ON" if value else "OFF"))
        mask = updated

    if not changes:
        print("\nfeatures are already as asked; nothing to change.")
        return data

    out = bytearray(data)
    struct.pack_into("<I", out, offset + 6, mask)

    print("\nfeatures      : %s" % ", ".join(changes))
    print("feature mask  : 0x%08X -> 0x%08X at file offset 0x%04X"
          % (before, mask, offset + 6))
    return refoot(out)


def main():
    ap = argparse.ArgumentParser(
        description="Inspect an INAV SITL eeprom, and optionally change its "
                    "receiver type, its GPS provider, its gyro main LPF cutoff, "
                    "its motor direction inversion, or its feature bits.")
    ap.add_argument("source", help="eeprom to read")
    ap.add_argument("--verify", action="store_true",
                    help="parse and check the checksum, change nothing")
    ap.add_argument("--set-receiver", choices=sorted(RX_TYPE),
                    help="receiver type to write")
    ap.add_argument("--set-gyro-lpf", type=int, metavar="HZ",
                    help="gyro_main_lpf_hz to write; 0 disables the filter "
                         "(see the module docstring for why SITL wants that)")
    ap.add_argument("--set-motor-direction-inverted",
                    choices=sorted(MOTOR_DIRECTION_INVERTED),
                    help="motor_direction_inverted to write; ON matches the real "
                         "GEPRC F722, which the UE plugin is set up for")
    ap.add_argument("--set-gps-provider", choices=sorted(GPS_PROVIDER),
                    help="gps_provider to write; FAKE is the only one that lets "
                         "FEATURE_GPS survive gpsInit() in SITL, so it is the "
                         "only one that turns the velocity reference on")
    ap.add_argument("--set-feature", action="append", metavar="NAME=ON|OFF",
                    help="feature bit to write, e.g. GPS=ON; repeatable, and "
                         "only the named bits move (known: %s)"
                         % ", ".join(sorted(FEATURE_BIT)))
    ap.add_argument("-o", "--output",
                    help="where to write the result (required with a setter)")
    args = ap.parse_args()

    setters = (args.set_receiver is not None or args.set_gps_provider is not None
               or args.set_gyro_lpf is not None
               or args.set_motor_direction_inverted is not None
               or args.set_feature)
    if not args.verify and not setters:
        ap.error("give --verify, --set-receiver, --set-gps-provider, "
                 "--set-gyro-lpf, --set-motor-direction-inverted or --set-feature")
    if setters and not args.output:
        ap.error("a setter needs -o")
    if args.set_gyro_lpf is not None and not 0 <= args.set_gyro_lpf <= 0xFFFF:
        ap.error("--set-gyro-lpf must be 0..65535")

    features = {}
    for spec in args.set_feature or ():
        name, sep, value = spec.partition("=")
        name, value = name.strip().upper(), value.strip().upper()
        if not sep or name not in FEATURE_BIT:
            ap.error("--set-feature takes a known NAME=ON|OFF, got %r" % spec)
        if value not in FEATURE_ON_OFF:
            ap.error("--set-feature value must be ON or OFF, got %r" % value)
        features[name] = FEATURE_ON_OFF[value]

    try:
        with open(args.source, "rb") as f:
            data = f.read()
    except OSError as e:
        print("ERROR: %s" % e, file=sys.stderr)
        return 1

    print("%s (%d bytes)\n" % (args.source, len(data)))

    try:
        if not setters:
            describe(data)
            return 0

        # Chained, so one invocation can derive the MSP template from the SIM
        # one and apply the gyro workaround in the same pass.
        out = data
        if args.set_receiver is not None:
            out = set_receiver(out, RX_TYPE[args.set_receiver])
        if args.set_gps_provider is not None:
            out = set_gps_provider(out, GPS_PROVIDER[args.set_gps_provider])
        if args.set_gyro_lpf is not None:
            out = set_gyro_lpf(out, args.set_gyro_lpf)
        if args.set_motor_direction_inverted is not None:
            out = set_motor_direction_inverted(
                out, MOTOR_DIRECTION_INVERTED[args.set_motor_direction_inverted])
        if features:
            out = set_features(out, features)

        with open(args.output, "wb") as f:
            f.write(out)
        print("wrote %s (%d bytes)" % (args.output, len(out)))
        print("\nverifying the result:")
        describe(out)
    except (ValueError, OSError) as e:
        # ValueError is a layout problem this script refuses to guess at;
        # OSError is a file that will not open or write -- an output copied out
        # of the image keeps the template's 0444, so writing over it in place is
        # a normal thing to run into and deserves the same one-line failure.
        print("ERROR: %s" % e, file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
