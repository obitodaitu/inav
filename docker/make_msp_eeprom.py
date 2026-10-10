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

Usage:
    make_msp_eeprom.py --verify eeprom.default.bin
    make_msp_eeprom.py --set-receiver MSP -o eeprom.msp.bin eeprom.default.bin
    make_msp_eeprom.py --set-gyro-lpf 0 -o out.bin eeprom.msp.bin
    make_msp_eeprom.py --set-motor-direction-inverted ON -o out.bin in.bin
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


def main():
    ap = argparse.ArgumentParser(
        description="Inspect an INAV SITL eeprom, and optionally change its "
                    "receiver type, its gyro main LPF cutoff, or its motor "
                    "direction inversion.")
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
    ap.add_argument("-o", "--output",
                    help="where to write the result (required with a setter)")
    args = ap.parse_args()

    setters = (args.set_receiver is not None or args.set_gyro_lpf is not None
               or args.set_motor_direction_inverted is not None)
    if not args.verify and not setters:
        ap.error("give --verify, --set-receiver, --set-gyro-lpf or "
                 "--set-motor-direction-inverted")
    if setters and not args.output:
        ap.error("a setter needs -o")
    if args.set_gyro_lpf is not None and not 0 <= args.set_gyro_lpf <= 0xFFFF:
        ap.error("--set-gyro-lpf must be 0..65535")

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
        if args.set_gyro_lpf is not None:
            out = set_gyro_lpf(out, args.set_gyro_lpf)
        if args.set_motor_direction_inverted is not None:
            out = set_motor_direction_inverted(
                out, MOTOR_DIRECTION_INVERTED[args.set_motor_direction_inverted])

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
