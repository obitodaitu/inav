"""Unit tests for docker/msp_rc_client.py.

Runs under both the stdlib runner and pytest:

    scripts/run_unit_tests.sh
    python3 -m unittest scripts.test_msp_rc_client -v
    pytest scripts/test_msp_rc_client.py

Only the module's pure wire-level helpers are exercised here; MspLink and the
arming state machine need a live socket and belong in an integration test.
"""

import os
import struct
import sys
import unittest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "docker"))

import msp_rc_client as c  # noqa: E402


def v1_response(cmd, payload=b"", err=False):
    body = bytes([len(payload), cmd]) + payload
    checksum = 0
    for b in body:
        checksum ^= b
    return b"$M" + (b"!" if err else b">") + body + bytes([checksum])


def v2_response(cmd, payload=b"", err=False):
    body = struct.pack("<BHH", 0, cmd, len(payload)) + payload
    return b"$X" + (b"!" if err else b">") + body + bytes([c.crc8_dvb_s2(body)])


class DescribeFlagsTests(unittest.TestCase):
    def test_nothing_set(self):
        self.assertEqual(c.describe_flags(0), "none set")

    def test_armed(self):
        self.assertEqual(c.describe_flags(c.ARMED), "ARMED")

    def test_state_bits_then_blockers(self):
        self.assertEqual(
            c.describe_flags(c.ARMED | c.ARMING_DISABLED_RC_LINK),
            "ARMED|RC_LINK",
        )

    def test_simulator_mode(self):
        self.assertEqual(
            c.describe_flags(c.SIMULATOR_MODE_SITL), "SIMULATOR_MODE_SITL"
        )

    def test_was_ever_armed(self):
        self.assertEqual(c.describe_flags(c.WAS_EVER_ARMED), "WAS_EVER_ARMED")

    def test_rc_link_bit_number(self):
        # The flag is named because bit 18 is what --reboot polls for.
        self.assertEqual(c.ARMING_DISABLED_RC_LINK, 1 << 18)


class VehicleIndexTests(unittest.TestCase):
    def test_base_port(self):
        self.assertEqual(c.vehicle_index(5760), (0, True))

    def test_stride_is_ten(self):
        self.assertEqual(c.vehicle_index(5770), (1, True))
        self.assertEqual(c.vehicle_index(5790), (3, True))

    def test_misaligned_port(self):
        self.assertEqual(c.vehicle_index(5765), (0, False))

    def test_below_base_is_not_valid(self):
        index, looks_valid = c.vehicle_index(5000)
        self.assertFalse(looks_valid)
        self.assertIsInstance(index, int)


class StickTests(unittest.TestCase):
    def test_center_is_1500(self):
        self.assertEqual(c.stick(0.0), 1500)

    def test_extremes(self):
        self.assertEqual(c.stick(1.0), c.RC_MAX)
        self.assertEqual(c.stick(-1.0), c.RC_MIN)

    def test_quarter_travel(self):
        self.assertEqual(c.stick(0.5), 1750)
        self.assertEqual(c.stick(-0.5), 1250)

    def test_result_is_int(self):
        self.assertIsInstance(c.stick(0.333), int)


class Crc8DvbS2Tests(unittest.TestCase):
    def test_empty_is_zero(self):
        self.assertEqual(c.crc8_dvb_s2(b""), 0)

    def test_known_check_value(self):
        # CRC-8/DVB-S2 (poly 0xD5, init 0, no reflection, no xorout).
        self.assertEqual(c.crc8_dvb_s2(b"123456789"), 0xBC)

    def test_stays_in_one_byte(self):
        self.assertLessEqual(c.crc8_dvb_s2(bytes(range(256))), 0xFF)


class BuildRequestTests(unittest.TestCase):
    def test_v1_empty(self):
        self.assertEqual(c.build_request(1, b""), b"$M<\x00\x01\x01")

    def test_v1_checksum_is_xor_of_body(self):
        payload = bytes([1, 2, 3])
        frame = c.build_request(200, payload)
        self.assertTrue(frame.startswith(b"$M<"))
        body = frame[3:-1]
        expected = 0
        for b in body:
            expected ^= b
        self.assertEqual(frame[-1], expected)
        self.assertEqual(body, bytes([len(payload), 200]) + payload)

    def test_v2_uses_x_magic(self):
        frame = c.build_request(0x2000, b"")
        self.assertTrue(frame.startswith(b"$X<"))
        body = frame[3:-1]
        self.assertEqual(body, struct.pack("<BHH", 0, 0x2000, 0))
        self.assertEqual(frame[-1], c.crc8_dvb_s2(body))

    def test_v1_for_byte_commands(self):
        self.assertTrue(c.build_request(255).startswith(b"$M<"))

    def test_v2_at_the_0x1000_threshold(self):
        self.assertTrue(c.build_request(0x1000).startswith(b"$X<"))

    def test_v2_payload_is_carried_and_crc_covers_it(self):
        payload = b"\xde\xad\xbe\xef"
        frame = c.build_request(0x2001, payload)
        body = frame[3:-1]
        self.assertTrue(body.endswith(payload))
        self.assertEqual(frame[-1], c.crc8_dvb_s2(body))


class FindFrameTests(unittest.TestCase):
    def test_v1_ok_frame(self):
        self.assertEqual(c._find_frame(v1_response(105)), (0, False, False))

    def test_v1_error_frame(self):
        self.assertEqual(c._find_frame(v1_response(68, err=True)), (0, False, True))

    def test_v2_ok_frame(self):
        self.assertEqual(c._find_frame(v2_response(1)), (0, True, False))

    def test_leading_garbage_is_skipped(self):
        self.assertEqual(c._find_frame(b"junk" + v1_response(1)), (4, False, False))

    def test_partial_marker_is_not_a_frame(self):
        self.assertEqual(c._find_frame(b"$M"), (None, False, False))

    def test_earliest_frame_wins(self):
        buf = v1_response(1) + v2_response(2)
        self.assertEqual(c._find_frame(buf), (0, False, False))

    def test_request_marker_is_not_a_response(self):
        # Requests use '<'; only '>' / '!' end a response.
        self.assertEqual(c._find_frame(c.build_request(1)), (None, False, False))


class DecodeTests(unittest.TestCase):
    def test_v1_roundtrip(self):
        frame = v1_response(105, b"\x01\x02")
        self.assertEqual(c._decode(frame, 0, False), (105, b"\x01\x02", len(frame)))

    def test_v1_short(self):
        self.assertIs(c._decode(b"$M>", 0, False), c.SHORT)

    def test_v1_bad_checksum(self):
        frame = bytearray(v1_response(105, b"\x01\x02"))
        frame[-1] ^= 0xFF
        self.assertIs(c._decode(bytes(frame), 0, False), c.BAD)

    def test_v2_roundtrip(self):
        frame = v2_response(0x2000, b"\xaa\xbb")
        self.assertEqual(
            c._decode(frame, 0, True), (0x2000, b"\xaa\xbb", len(frame))
        )

    def test_v2_bad_crc(self):
        frame = bytearray(v2_response(0x2000, b"\xaa\xbb"))
        frame[-1] ^= 0xFF
        self.assertIs(c._decode(bytes(frame), 0, True), c.BAD)

    def test_decode_from_offset(self):
        frame = b"\x00\x00" + v1_response(7, b"x")
        cmd, payload, end = c._decode(frame, 2, False)
        self.assertEqual((cmd, payload), (7, b"x"))
        self.assertEqual(end, len(frame))


if __name__ == "__main__":
    unittest.main()
