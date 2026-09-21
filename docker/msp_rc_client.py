#!/usr/bin/env python3
"""Fly a SITL container over MSP.

The counterpart to `launch_sitl.sh --receiver msp`. That flag boots the
container on an eeprom whose receiver_type is MSP, which makes the firmware
take its RC channels from MSP_SET_RAW_RC on the container's TCP port instead of
from the simulator's datarefs. The simulator link is untouched by this: Unreal
still runs the physics and still supplies every sensor. Only the sticks move.

This is a reference, not a library. It is deliberately small and dependency-free
so it can be read in one sitting and reimplemented in whatever the algorithm
actually lives in -- the protocol is about forty lines, and everything else here
is a startup sequence.

Usage:
    msp_rc_client.py --port 5760 --arm
    msp_rc_client.py --port 5760 --seconds 10 --throttle 0.3

What it does, and why in this order:

  1. Waits for the flight controller to answer MSP. The container binds its
     MSP port during startup, and the port is up well before the first
     MSP_SET_RAW_RC would be useful.
  2. Sends sticks at 50 Hz and nothing else. This is the whole job.
  3. Arming is an AUX channel, not a command -- there is no "arm" message in
     MSP. The client raises AUX1 and holds it. Whatever range the ARM box was
     configured with decides the threshold, so this does not guess at it.
  4. Arming runs in three stages -- AUX1 low while the link comes up, then
     AUX1 high with the throttle STILL down, then the sticks -- because the
     firmware latches the arm switch (fc_core.c, "CHECK: Arming switch"): if
     anything else blocks arming at the moment the switch goes high,
     ARM_SWITCH is raised and stays raised until the switch goes low again.
     Two things trip that, and both are easy to do by accident:

       - sending AUX1 high in the first packet, before
         ARMING_DISABLED_RC_LINK has cleared, and
       - arming with the throttle already raised. Anything at or above
         rxConfig()->min_check (1100 by default) is not "low"
         (rc_controls.c: calculateThrottleStatus), so ARMING_DISABLED_THROTTLE
         blocks the arm and ARM_SWITCH latches in the same pass. ARM_SWITCH
         is earlier in armDisableReasonsChecklist, so it is the name the
         firmware reports -- the log blames the switch and the real cause is
         never shown.

     Either one costs the whole flight: the aircraft stays disarmed for as
     long as the client keeps AUX1 high, however long that is. The stage-1
     window also clears a latch left over from a previous run, which is how
     the client recovers without a container restart.

The one thing that will bite you: the MSP receiver times out after 200 ms of
silence (rx/msp.c sets rxSignalTimeout = DELAY_5_HZ). Stop sending and the
aircraft failsafes, whether or not you meant it. Keep the loop running for as
long as the aircraft should be in the air.
"""

import argparse
import socket
import struct
import sys
import time

MSP_SET_RAW_RC = 200
MSP_RC = 105
MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP2_INAV_STATUS = 0x2000

# AETR, before rcmap is applied. rx.c's default rcmap is {0,1,3,2}, which means
# the wire order is Roll, Pitch, THROTTLE, Yaw -- the throttle is at index 2
# here, not 3. Reading MSP_RC back gives a *different* order (it reports the
# post-remap internal sequence, where throttle is index 3), so a loop that
# echoes what it reads will swap throttle and yaw. This asymmetry is the single
# most common way to get this wrong.
ROLL, PITCH, THROTTLE, YAW = 0, 1, 2, 3

RC_MIN, RC_MID, RC_MAX = 1000, 1500, 2000

# rx/msp.c: DELAY_5_HZ. Silence longer than this and the receiver reports no
# signal, which is a failsafe.
RX_TIMEOUT_MS = 200

DEFAULT_RATE_HZ = 50

# How long AUX1 is held low after connecting, and how long it is then held
# high with the throttle still down. The first is long enough for the firmware
# to notice the link and clear ARMING_DISABLED_RC_LINK; the second is where it
# actually arms. Both exist because the arm switch latches -- see note 4 in
# the module docstring. The RX task runs at 10 Hz, so the first is a wide
# margin; arming itself is immediate once nothing blocks it.
ARM_SETTLE_S = 1.5
ARM_HOLD_S = 0.5


def crc8_dvb_s2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_request(cmd, payload=b""):
    """Frame an MSP request. v2 for commands >= 0x1000, v1 otherwise."""
    if cmd >= 0x1000:
        body = struct.pack("<BHH", 0, cmd, len(payload)) + payload
        return b"$X<" + body + bytes([crc8_dvb_s2(body)])

    body = bytes([len(payload), cmd]) + payload
    checksum = 0
    for b in body:
        checksum ^= b
    return b"$M<" + body + bytes([checksum])


_V1_HEADER = b"$M>"
_V2_HEADER = b"$X>"
SHORT = object()
BAD = object()


def _decode(buf, start, v2):
    if v2:
        if len(buf) < start + 8:
            return SHORT
        length = struct.unpack_from("<H", buf, start + 6)[0]
        end = start + 9 + length
        if len(buf) < end:
            return SHORT
        if crc8_dvb_s2(buf[start + 3:start + 8 + length]) != buf[start + 8 + length]:
            return BAD
        return (struct.unpack_from("<H", buf, start + 4)[0],
                buf[start + 8:start + 8 + length], end)

    if len(buf) < start + 5:
        return SHORT
    length = buf[start + 3]
    end = start + 6 + length
    if len(buf) < end:
        return SHORT
    checksum = 0
    for b in buf[start + 3:start + 5 + length]:
        checksum ^= b
    if checksum != buf[start + 5 + length]:
        return BAD
    return buf[start + 4], buf[start + 5:start + 5 + length], end


def read_frame(sock, buf, timeout):
    """(cmd, payload, buf) for the next well-formed response, or (None, None, buf)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        i1 = buf.find(_V1_HEADER)
        i2 = buf.find(_V2_HEADER)
        start, v2 = (None, False)
        if i2 < 0 or (0 <= i1 < i2):
            start, v2 = (i1, False) if i1 >= 0 else (None, False)
        else:
            start, v2 = i2, True

        if start is not None:
            got = _decode(buf, start, v2)
            if got is BAD:
                buf = buf[:start] + buf[start + 1:]
                continue
            if got is not SHORT:
                cmd, payload, end = got
                # Drop the consumed frame, keep whatever followed it: the
                # remainder may already hold the start of the next response.
                return cmd, payload, buf[:start] + buf[end:]
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            continue
        if not chunk:
            break
        buf += chunk
    return None, None, buf


class MspLink:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=2.0)
        self.sock.settimeout(0.2)
        self.buf = b""

    def request(self, cmd, payload=b"", timeout=1.0):
        """Send a command and return the response *to that command*.

        Skipping ahead is not optional. Every MSP_SET_RAW_RC is acknowledged,
        so while the stick loop runs there is always a backlog of ACKs sitting
        in the buffer -- one per packet sent, fifty a second. Reading a single
        frame returns the oldest ACK, never the reply that was asked for. This
        walks past them (and anything else) until the command number matches.
        """
        self.sock.sendall(build_request(cmd, payload))
        deadline = time.time() + timeout
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                return None, None, self.buf
            got_cmd, got_payload, self.buf = read_frame(self.sock, self.buf, remaining)
            if got_cmd is None:
                return None, None, self.buf
            if got_cmd == cmd:
                return got_cmd, got_payload, self.buf

    def send_raw_rc(self, channels):
        """channels: 8+ values in AETR wire order."""
        payload = b"".join(struct.pack("<H", c) for c in channels)
        self.sock.sendall(build_request(MSP_SET_RAW_RC, payload))

    def close(self):
        self.sock.close()


def wait_for_fc(link, timeout=15.0):
    """Poll until the firmware answers. Returns the variant string, or None."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        cmd, payload, link.buf = link.request(MSP_API_VERSION, timeout=0.5)
        if cmd == MSP_API_VERSION:
            cmd, payload, link.buf = link.request(MSP_FC_VARIANT, timeout=0.5)
            variant = payload.decode("ascii", "replace") if cmd == MSP_FC_VARIANT else "?"
            return variant
        time.sleep(0.25)
    return None


def read_channels(link):
    """The channels as the firmware sees them, post-remap internal order."""
    cmd, payload, link.buf = link.request(MSP_RC, timeout=0.5)
    if cmd != MSP_RC or len(payload) < 8:
        return None
    return list(struct.unpack_from("<%dH" % (len(payload) // 2), payload, 0))


def main():
    ap = argparse.ArgumentParser(
        description="Send RC sticks to a SITL container over MSP.",
        epilog="The container must have been started with --receiver msp, or its "
               "receiver_type is SIM and these packets go nowhere.")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5760,
                    help="the container's MSP port: 5760 + 10*VehicleIndex (default 5760)")
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ, help="Hz (default 50)")
    ap.add_argument("--seconds", type=float, default=0,
                    help="how long to fly; 0 means until interrupted (default 0)")
    ap.add_argument("--arm", action="store_true",
                    help="raise AUX1 and hold it, to arm")
    ap.add_argument("--throttle", type=float, default=0.0,
                    help="0..1, scaled from the bottom of the stick range (default 0)")
    ap.add_argument("--roll", type=float, default=0.0, help="-1..1 (default 0)")
    ap.add_argument("--pitch", type=float, default=0.0, help="-1..1 (default 0)")
    ap.add_argument("--yaw", type=float, default=0.0, help="-1..1 (default 0)")
    args = ap.parse_args()

    period = 1.0 / args.rate
    if period * 1000.0 > RX_TIMEOUT_MS:
        print("error: %.1f Hz is slower than the %d ms receiver timeout; the "
              "aircraft would failsafe between packets." % (args.rate, RX_TIMEOUT_MS),
              file=sys.stderr)
        return 1

    link = MspLink(args.host, args.port)

    variant = wait_for_fc(link)
    if variant is None:
        print("error: no MSP response on %s:%d." % (args.host, args.port), file=sys.stderr)
        print("       Is the container up, and did it bind this port? "
              "`docker logs inav-sitl-N` should show 'Bind TCP [::]:%d to UART1'."
              % args.port, file=sys.stderr)
        return 1
    print("connected to %s:%d (%s)" % (args.host, args.port, variant))

    def stick(v):
        return int(round(RC_MID + v * (RC_MAX - RC_MID)))

    # Throttle is one-sided: 0 is the bottom of the range, not the centre.
    throttle = int(round(RC_MIN + args.throttle * (RC_MAX - RC_MIN)))

    channels = [RC_MID] * 8
    channels[ROLL] = stick(args.roll)
    channels[PITCH] = stick(args.pitch)
    channels[THROTTLE] = throttle
    channels[YAW] = stick(args.yaw)
    channels[4] = RC_MAX if args.arm else RC_MIN

    # The two frames on the way to the one above. Both keep the throttle down,
    # and the first keeps AUX1 down as well -- see note 4 in the module
    # docstring for what each of them is for.
    settle_channels = list(channels)
    settle_channels[THROTTLE] = RC_MIN
    settle_channels[4] = RC_MIN

    arm_channels = list(channels)
    arm_channels[THROTTLE] = RC_MIN

    # Without --arm there is nothing to sequence: AUX1 is low, so the throttle
    # does nothing and can go straight to where it was asked to be.
    settle_packets = int(round(args.rate * ARM_SETTLE_S)) if args.arm else 0
    arm_packets = settle_packets + (int(round(args.rate * ARM_HOLD_S)) if args.arm else 0)

    print("sticks: roll=%d pitch=%d throttle=%d yaw=%d aux1=%d"
          % (channels[0], channels[1], channels[2], channels[3], channels[4]))
    if not args.arm:
        print("note: --arm not given, so AUX1 is low and the aircraft will not arm.")
    else:
        print("arming: AUX1 low for %.1fs while the link comes up, then raised "
              "with the throttle still down for %.1fs, then the sticks."
              % (ARM_SETTLE_S, ARM_HOLD_S))
        if throttle == RC_MIN:
            print("note: --throttle was not given, so the throttle channel stays "
                  "at the bottom of its range.")
    print("sending at %.0f Hz. Ctrl-C to stop." % args.rate)

    started = time.time()
    sent = 0
    try:
        while True:
            if args.seconds and (time.time() - started) >= args.seconds:
                break

            # Re-sent every iteration on purpose. rxMspFrameReceive zeroes every
            # channel the packet does not carry, so a short frame silently
            # clears the AUX channels -- including the one holding ARM.
            #
            # AUX1 goes high only after the settle window, and the throttle
            # only after the arm window. Doing either too early latches
            # ARM_SWITCH and the aircraft then never arms, however long the
            # loop runs -- see note 4 in the module docstring.
            if sent < settle_packets:
                frame = settle_channels
            elif sent < arm_packets:
                frame = arm_channels
            else:
                frame = channels
            link.send_raw_rc(frame)
            sent += 1

            # Pace against an absolute schedule, not sleep(period): the send
            # itself takes time, and drift would show up as a slow link.
            target = started + sent * period
            slack = target - time.time()
            if slack > 0:
                time.sleep(slack)

            if sent % int(args.rate) == 0:
                seen = read_channels(link)
                if seen is not None:
                    # Deliberately shown in the firmware's own order. It will
                    # not match what was sent above -- see the note on AETR.
                    print("  %5.1fs  fc sees: %s" % (time.time() - started, seen[:6]))
    except KeyboardInterrupt:
        print()
    finally:
        # Drop the sticks rather than just closing: closing the socket is
        # indistinguishable from a network problem, and leaving the aircraft
        # holding the last stick position is how a demo ends in a wall.
        stop = [RC_MID] * 8
        stop[THROTTLE] = RC_MIN
        stop[4] = RC_MIN
        for _ in range(20):
            try:
                link.send_raw_rc(stop)
            except OSError:
                break
            time.sleep(0.02)
        link.close()
        print("sent %d packets, sticks released." % sent)

    return 0


if __name__ == "__main__":
    sys.exit(main())
