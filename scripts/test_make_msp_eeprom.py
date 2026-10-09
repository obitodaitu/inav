"""Unit tests for docker/make_msp_eeprom.py.

Runs under both the stdlib runner and pytest:

    scripts/run_unit_tests.sh
    python3 -m unittest scripts.test_make_msp_eeprom -v
    pytest scripts/test_make_msp_eeprom.py

The module is import-safe (its __main__ guard is the only side effect), so it
is imported straight from docker/ via a sys.path insert rather than copied.
"""

import contextlib
import io
import os
import sys
import unittest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "docker"))

import make_msp_eeprom as m  # noqa: E402


def quiet(fn, *args, **kwargs):
    """Call fn while swallowing its progress prints."""
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*args, **kwargs)


def synth_eeprom(records):
    """Build a valid eeprom stream from (pgn, payload) pairs."""
    out = bytearray([m.EEPROM_CONF_VERSION])
    for pgn, payload in records:
        size = 6 + len(payload)
        out += size.to_bytes(2, "little")
        out += pgn.to_bytes(2, "little")
        out += bytes([0, 0])          # version, flags = system
        out += bytes(payload)
    out += b"\x00\x00"                # terminator, covered by the CRC
    out += m.crc16(bytes(out)).to_bytes(2, "little")
    # parse() reads records in chunks of 6 and only then notices the zero-size
    # terminator, so it needs a few bytes of slack past the footer. Real eeproms
    # are a padded 32 KiB, which is why the parser gets away with it.
    out += b"\xff" * 8
    return bytes(out)


class Crc16Tests(unittest.TestCase):
    def test_empty_is_zero(self):
        self.assertEqual(m.crc16(b""), 0)

    def test_known_check_value(self):
        # CRC-16/XMODEM (poly 0x1021, init 0, no reflection, no xorout).
        self.assertEqual(m.crc16(b"123456789"), 0x31C3)

    def test_incremental_matches_oneshot(self):
        data = b"the quick brown fox"
        crc = 0
        for b in data:
            crc = m.crc16_ccitt(crc, b)
        self.assertEqual(crc, m.crc16(data))

    def test_truncates_to_16_bits(self):
        # If the mask were dropped, the C uint16_t truncation would not match.
        for byte in (0xFF, 0x00, 0x80, 0x7F):
            self.assertLessEqual(m.crc16_ccitt(0xFFFF, byte), 0xFFFF)


class U16Tests(unittest.TestCase):
    def test_little_endian(self):
        self.assertEqual(m.u16(bytes([0x34, 0x12]), 0), 0x1234)
        self.assertEqual(m.u16(bytes([0x00, 0x80]), 0), 0x8000)

    def test_respects_offset(self):
        self.assertEqual(m.u16(b"\xff\x34\x12", 1), 0x1234)


class ParseTests(unittest.TestCase):
    def test_valid_stream_roundtrip(self):
        data = synth_eeprom([(1011, b"\xaa\xbb\xcc")])
        records, footer_offset, stored, computed = m.parse(data)
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0][2], 1011)          # pgn
        self.assertEqual(stored, computed)
        self.assertEqual(footer_offset, 1 + 6 + 3)     # header + payload

    def test_empty_config_is_valid(self):
        data = synth_eeprom([])
        records, footer_offset, stored, computed = m.parse(data)
        self.assertEqual(records, [])
        self.assertEqual(footer_offset, 1)
        self.assertEqual(stored, computed)

    def test_too_short(self):
        for data in (b"", b"\x7e", b"\x7e\x00"):
            with self.assertRaises(ValueError):
                m.parse(data)

    def test_wrong_format_byte(self):
        with self.assertRaises(ValueError):
            m.parse(bytes([m.EEPROM_CONF_VERSION + 1, 0, 0]))

    def test_record_smaller_than_header(self):
        data = bytes([m.EEPROM_CONF_VERSION, 5, 0, 0, 0, 0, 0, 0])
        with self.assertRaises(ValueError):
            m.parse(data)

    def test_record_past_end_of_file(self):
        data = bytes([m.EEPROM_CONF_VERSION, 100, 0, 0, 0, 0, 0, 0, 0, 0])
        with self.assertRaises(ValueError):
            m.parse(data)

    def test_bad_stored_checksum_is_reported(self):
        data = bytearray(synth_eeprom([(1011, b"\x01")]))
        data[-9] ^= 0xFF                  # high byte of the stored CRC16
        records, _, stored, computed = m.parse(bytes(data))
        self.assertNotEqual(stored, computed)


class DescribeSyntheticTests(unittest.TestCase):
    def test_finds_rx_config_and_returns_receiver_type(self):
        payload = bytes([m.RX_TYPE["MSP"]]) + m.EXPECTED_RCMAP + b"\x00"
        data = synth_eeprom([(m.PG_RX_CONFIG, payload)])
        byte_offset, receiver_type = quiet(m.describe, data)
        self.assertEqual(receiver_type, m.RX_TYPE["MSP"])
        self.assertEqual(byte_offset, 1 + 6)  # first payload byte

    def test_rejects_unexpected_rcmap(self):
        payload = bytes([m.RX_TYPE["MSP"]]) + b"\x09\x09\x09\x09" + b"\x00"
        data = synth_eeprom([(m.PG_RX_CONFIG, payload)])
        with self.assertRaises(ValueError):
            quiet(m.describe, data)

    def test_rejects_bad_checksum(self):
        payload = bytes([m.RX_TYPE["MSP"]]) + m.EXPECTED_RCMAP + b"\x00"
        data = bytearray(synth_eeprom([(m.PG_RX_CONFIG, payload)]))
        data[-9] ^= 0xFF
        with self.assertRaises(ValueError):
            quiet(m.describe, bytes(data))


class ShippedEepromTests(unittest.TestCase):
    """The two blobs the image actually ships must agree with the tool."""

    @classmethod
    def setUpClass(cls):
        with open(os.path.join(REPO_ROOT, "docker", "eeprom.default.bin"), "rb") as f:
            cls.default = f.read()
        with open(os.path.join(REPO_ROOT, "docker", "eeprom.msp.bin"), "rb") as f:
            cls.msp = f.read()

    def test_default_reports_sim(self):
        _, receiver_type = quiet(m.describe, self.default)
        self.assertEqual(receiver_type, m.RX_TYPE["SIM"])

    def test_msp_reports_msp(self):
        _, receiver_type = quiet(m.describe, self.msp)
        self.assertEqual(receiver_type, m.RX_TYPE["MSP"])

    def test_set_receiver_reproduces_shipped_msp(self):
        # The whole point of the tool: byte-for-byte the file in the image.
        out = quiet(m.set_receiver, self.default, m.RX_TYPE["MSP"])
        self.assertEqual(out, self.msp)

    def test_set_receiver_to_same_value_is_a_noop(self):
        self.assertIsNone(quiet(m.set_receiver, self.msp, m.RX_TYPE["MSP"]))

    def test_set_receiver_is_reversible(self):
        out = quiet(m.set_receiver, self.msp, m.RX_TYPE["SIM"])
        self.assertEqual(out, self.default)


if __name__ == "__main__":
    unittest.main()
