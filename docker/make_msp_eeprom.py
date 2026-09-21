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

Usage:
    make_msp_eeprom.py --verify eeprom.default.bin
    make_msp_eeprom.py --set-receiver MSP -o eeprom.msp.bin eeprom.default.bin
"""

import argparse
import sys

EEPROM_CONF_VERSION = 126
PG_RX_CONFIG = 24
CR_CLASSIFICATION_MASK = 0x3
CR_CLASSIFICATION_SYSTEM = 0

RX_TYPE = {"NONE": 0, "SERIAL": 1, "MSP": 2, "SIM": 3}
RX_TYPE_NAME = {v: k for k, v in RX_TYPE.items()}

# rxConfig_t.rcmap's factory value, used below to confirm the record we found
# really is rxConfig and not a pgn collision.
EXPECTED_RCMAP = bytes([0, 1, 3, 2])


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


def describe(data):
    records, footer_offset, stored, computed = parse(data)
    ok = stored == computed

    print("format        : %d" % data[0])
    print("records       : %d" % len(records))
    print("footer at     : 0x%04X" % footer_offset)
    print("config size   : %d bytes (file is %d)" % (footer_offset + 4, len(data)))
    print("checksum      : stored 0x%04X, computed 0x%04X  %s"
          % (stored, computed, "OK" if ok else "MISMATCH"))
    print()

    rx_records = []
    for offset, size, pgn, version, flags in records:
        payload = data[offset + 6:offset + size]
        note = ""
        if pgn == PG_RX_CONFIG and (flags & CR_CLASSIFICATION_MASK) == CR_CLASSIFICATION_SYSTEM:
            rx_records.append((offset, size, payload))
            note = "  <- PG_RX_CONFIG, receiver_type = %s (%d)" % (
                RX_TYPE_NAME.get(payload[0], "?"), payload[0])
        print("  0x%04X  size=%-5d pgn=%-4d ver=%-3d flags=%d%s"
              % (offset, size, pgn, version, flags, note))

    print()
    if len(rx_records) != 1:
        raise ValueError("expected exactly one system PG_RX_CONFIG record, found %d"
                         % len(rx_records))

    offset, size, payload = rx_records[0]
    rcmap = payload[1:5]
    if rcmap != EXPECTED_RCMAP:
        raise ValueError(
            "record at 0x%04X has rcmap %s, expected %s - the payload does not "
            "look like rxConfig_t, so the byte offset of receiver_type cannot be "
            "trusted" % (offset, list(rcmap), list(EXPECTED_RCMAP))
        )
    print("receiver_type : %s (%d) at file offset 0x%04X"
          % (RX_TYPE_NAME.get(payload[0], "?"), payload[0], offset + 6))
    print("rcmap         : %s (AETR - confirms this is rxConfig_t)" % list(rcmap))

    if not ok:
        raise ValueError("checksum mismatch - refusing to treat this file as valid")

    return offset + 6, payload[0]


def set_receiver(data, value):
    """Return a copy of data with receiver_type set, checksum recomputed."""
    byte_offset, current = describe(data)

    if current == value:
        print("\nreceiver_type is already %s; nothing to change."
              % RX_TYPE_NAME.get(value, value))
        return None

    out = bytearray(data)
    out[byte_offset] = value

    # Everything from the header up to and including the footer is covered.
    # The checksum itself is not, which is why the footer offset has to be
    # recovered from a fresh parse rather than assumed.
    records, footer_offset, _, _ = parse(bytes(out))
    crc = crc16(out[0:footer_offset + 2])
    out[footer_offset + 2] = crc & 0xFF
    out[footer_offset + 3] = (crc >> 8) & 0xFF

    print("\nreceiver_type : %s (%d) -> %s (%d)"
          % (RX_TYPE_NAME.get(current, "?"), current,
             RX_TYPE_NAME.get(value, "?"), value))
    print("checksum      : -> 0x%04X" % crc)
    return bytes(out)


def main():
    ap = argparse.ArgumentParser(
        description="Inspect an INAV SITL eeprom, and optionally flip its receiver type.")
    ap.add_argument("source", help="eeprom to read")
    ap.add_argument("--verify", action="store_true",
                    help="parse and check the checksum, change nothing")
    ap.add_argument("--set-receiver", choices=sorted(RX_TYPE),
                    help="receiver type to write")
    ap.add_argument("-o", "--output",
                    help="where to write the result (required with --set-receiver)")
    args = ap.parse_args()

    if not args.verify and not args.set_receiver:
        ap.error("give --verify or --set-receiver")

    with open(args.source, "rb") as f:
        data = f.read()

    print("%s (%d bytes)\n" % (args.source, len(data)))

    try:
        if args.set_receiver:
            if not args.output:
                ap.error("--set-receiver needs -o")
            out = set_receiver(data, RX_TYPE[args.set_receiver])
            if out is not None:
                with open(args.output, "wb") as f:
                    f.write(out)
                print("wrote %s (%d bytes)" % (args.output, len(out)))
                print("\nverifying the result:")
                describe(out)
        else:
            describe(data)
    except ValueError as e:
        print("ERROR: %s" % e, file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
