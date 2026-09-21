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
    msp_rc_client.py --port 5760 --reboot --arm --throttle 0.5 --seconds 10

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

  5. --reboot is a prologue, not a mode. It reboots the flight controller
     first, then falls through into everything above, so
     `--reboot --arm --throttle 0.5 --seconds 10` is one command: revive,
     arm, fly.

Why --reboot exists. The firmware subscribes to the Unreal plugin exactly
once, on the transition into UE_CONNECTED (ue.c: registerAllDrefs is called
from that case), and that state has no path back. Restart Play-In-Editor and
the firmware notices the stale link, engages failsafe, and then sits there
forever: it still writes actuator values, still binds MSP, still answers every
poll it is sent, and never asks for sensor data again. The container is deaf
and `docker ps` calls it healthy. Rebooting re-execs the firmware's own image
(target.c: systemReset -> execvp), which re-runs the simulator handshake and
re-issues the subscriptions. Same container, same PID 1, same ports, no
`docker restart`, no resubmitted eeprom.

The prologue is not a bare "send MSP_REBOOT and wait":

  - MSP_REBOOT is refused while armed, so it disarms first. And because
    disarming with the ARM switch is gated on !failsafeIsActive(), an
    aircraft whose RC link died has to be woken out of the failsafe with
    stick motion before the switch will do anything. Holding AUX1 low with
    the sticks centred never disarms it.
  - It keeps streaming sticks throughout, because the 200 ms receiver
    timeout below does not stop applying just because the client is busy.
  - It refuses to believe a reboot happened until the old TCP connection
    actually drops. The ACK arrives about a second before the image is
    replaced, and the old process answers MSP the whole time, so "it
    answered again" is satisfied by a flight controller that never rebooted.
  - It then requires the NEW process to report SIMULATOR_MODE_SITL, which is
    set only by the code that does the subscribing. MSP_API_VERSION is not
    good enough for this: a flight controller that missed the ten-second
    handshake window boots with no simulator link at all and answers
    MSP_API_VERSION cheerfully while being unable to fly.

--reboot is strictly weaker than `docker restart inav-sitl-N`, and this
script says so when it gives up. It cannot reach a container whose MSP stack
is already dead -- notably one where the re-exec itself failed, which leaves
PID 1 spinning with every file descriptor closed and no log line. Use
`docker restart` for that.

The one thing that will bite you: the MSP receiver times out after 200 ms of
silence (rx/msp.c sets rxSignalTimeout = DELAY_5_HZ). Stop sending and the
aircraft failsafes, whether or not you meant it. Keep the loop running for as
long as the aircraft should be in the air. The single exception is the reboot
window inside --reboot: there the firmware is going down anyway and rebuilds
its receiver state from scratch when it comes back. Everywhere else -- and in
any reimplementation of this file -- the rule holds.
"""

import argparse
import socket
import struct
import sys
import time

MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_REBOOT = 68
MSP_RAW_IMU = 102
MSP_RC = 105
MSP_SET_RAW_RC = 200
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

# How far --reboot swings the sticks while it waits for a disarm, as a
# fraction of half travel. Two axes at once, in anti-phase, so the summed
# |channel - 1500| is 400 us where failsafe stick motion only asks for 50.
WIGGLE = 0.4

# runtime_config.h: armingFlag_e. Two state bits and the blocker mask all live
# in the one uint32_t that MSP2_INAV_STATUS reports verbatim, which is what
# makes a single poll answer both "is it armed?" and "can it arm?".
ARMED = 1 << 2
WAS_EVER_ARMED = 1 << 3
SIMULATOR_MODE_HITL = 1 << 4
SIMULATOR_MODE_SITL = 1 << 5

# The blocker bits by number, with the firmware's own names. Bit 17 is not in
# the enum, which is why ARMING_DISABLED_ALL_FLAGS is not a contiguous mask --
# do not "simplify" it into a range.
ARMING_DISABLE_BITS = {
    6:  "GEOZONE",
    7:  "FAILSAFE_SYSTEM",
    8:  "NOT_LEVEL",
    9:  "SENSORS_CALIBRATING",
    10: "SYSTEM_OVERLOADED",
    11: "NAVIGATION_UNSAFE",
    12: "COMPASS_NOT_CALIBRATED",
    13: "ACCELEROMETER_NOT_CALIBRATED",
    14: "ARM_SWITCH",
    15: "HARDWARE_FAILURE",
    16: "BOXFAILSAFE",
    18: "RC_LINK",
    19: "THROTTLE",
    20: "CLI",
    21: "CMS_MENU",
    22: "OSD_MENU",
    23: "ROLLPITCH_NOT_CENTERED",
    24: "SERVO_AUTOTRIM",
    25: "OOM",
    26: "INVALID_SETTING",
    27: "PWM_OUTPUT_ERROR",
    28: "NO_PREARM",
    29: "DSHOT_BEEPER",
    30: "LANDING_DETECTED",
}

ARMING_DISABLED_ALL_FLAGS = sum(1 << bit for bit in ARMING_DISABLE_BITS)

# Named individually because --reboot asks about these two specifically.
# ARM_SWITCH only clears when AUX1 goes low, so it is never something to wait
# on; RC_LINK is the flag that is set when the RC frames are going nowhere.
ARMING_DISABLED_ARM_SWITCH = 1 << 14
ARMING_DISABLED_RC_LINK = 1 << 18


def describe_flags(flags):
    """The set bits of armingFlags, by name, for a diagnostic line."""
    names = []
    if flags & ARMED:
        names.append("ARMED")
    if flags & WAS_EVER_ARMED:
        names.append("WAS_EVER_ARMED")
    if flags & SIMULATOR_MODE_SITL:
        names.append("SIMULATOR_MODE_SITL")
    for bit, name in sorted(ARMING_DISABLE_BITS.items()):
        if flags & (1 << bit):
            names.append(name)
    return "|".join(names) if names else "none set"


def vehicle_index(port):
    """The VehicleIndex a port implies, and whether the port looks like one."""
    offset = port - 5760
    return offset // 10, offset % 10 == 0 and offset >= 0


def stick(v):
    """A stick deflection in -1..1 to a channel value."""
    return int(round(RC_MID + v * (RC_MAX - RC_MID)))


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


_V1_MAGIC = b"$M"
_V2_MAGIC = b"$X"
_MARK_OK = b">"
_MARK_ERR = b"!"
SHORT = object()
BAD = object()


def _find_frame(buf):
    """(start, v2, is_error) for the earliest framed response, else (None, ..).

    Both markers are searched for, not just '>'. mspSerialEncode frames a
    refusal -- MSP_REBOOT aimed at an armed aircraft, for one -- as '$M!' or
    '$X!', and a client that only knows '>' cannot tell a refusal from a
    flight controller that has stopped answering. Those two call for opposite
    responses, so they must not be the same observable.
    """
    found = (None, False, False)
    for magic, v2 in ((_V1_MAGIC, False), (_V2_MAGIC, True)):
        i = buf.find(magic)
        while i >= 0:
            if i + 2 >= len(buf):
                # The marker byte has not arrived yet. Stop scanning this
                # magic rather than skip past a frame that is merely partial
                # -- `find` returned the first occurrence, so there is no
                # later one to find.
                break
            if buf[i + 2:i + 3] in (_MARK_OK, _MARK_ERR):
                if found[0] is None or i < found[0]:
                    found = (i, v2, buf[i + 2:i + 3] == _MARK_ERR)
                break
            i = buf.find(magic, i + 1)
    return found


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
    """(cmd, payload, buf, err) for the next well-formed response.

    (None, None, buf, False) on timeout. `err` is True when the reply came
    back with the error marker instead of a payload.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        start, v2, err = _find_frame(buf)
        if start is not None:
            got = _decode(buf, start, v2)
            if got is BAD:
                # Not really a frame -- the magic turned up inside a payload.
                # Drop one byte and resync on whatever follows.
                buf = buf[:start] + buf[start + 1:]
                continue
            if got is not SHORT:
                cmd, payload, end = got
                # Drop the consumed frame, keep whatever followed it: the
                # remainder may already hold the start of the next response.
                return cmd, payload, buf[:start] + buf[end:], err
        # Clamp the socket timeout to the time actually left. The socket's own
        # 200 ms timeout is the firmware's RC timeout as well, so a status
        # poll that overran it would put the aircraft it was checking on into
        # the failsafe it was checking for.
        try:
            sock.settimeout(max(0.005, min(0.2, deadline - time.time())))
            chunk = sock.recv(4096)
        except socket.timeout:
            continue
        if not chunk:
            break
        buf += chunk
    return None, None, buf, False


class MspLink:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = socket.create_connection((host, port), timeout=2.0)
        self.sock.settimeout(0.2)
        self.buf = b""

    def request(self, cmd, payload=b"", timeout=1.0):
        """Send a command and return (cmd, payload, buf, err) for *that* command.

        Skipping ahead is not optional. Every MSP_SET_RAW_RC is acknowledged,
        so while the stick loop runs there is always a backlog of ACKs sitting
        in the buffer -- one per packet sent, fifty a second. Reading a single
        frame returns the oldest ACK, never the reply that was asked for. This
        walks past them (and anything else) until the command number matches.

        An error reply carries the command number it is refusing, so it comes
        back from here too -- with err set. Callers that care about refusal
        must check it; a caller that ignores it sees exactly what a timeout
        looks like.
        """
        self.sock.sendall(build_request(cmd, payload))
        deadline = time.time() + timeout
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                return None, None, self.buf, False
            got_cmd, got_payload, self.buf, err = read_frame(self.sock, self.buf, remaining)
            if got_cmd is None:
                return None, None, self.buf, False
            if got_cmd == cmd:
                return got_cmd, got_payload, self.buf, err

    def send_raw_rc(self, channels):
        """channels: 8+ values in AETR wire order."""
        payload = b"".join(struct.pack("<H", c) for c in channels)
        self.sock.sendall(build_request(MSP_SET_RAW_RC, payload))

    def rebind(self, sock):
        """Replace the socket without changing this object's identity.

        In place on purpose. Callers hold a reference to *this* link --
        including the finally block that releases the sticks -- so opening a
        second socket would leave that release aimed at the dead connection
        while the live flight controller held on to the last AUX1-high frame.
        """
        try:
            self.sock.close()
        except OSError:
            pass
        self.sock = sock
        self.sock.settimeout(0.2)
        self.buf = b""

    def close(self):
        self.sock.close()


class Repeater:
    """Streams MSP_SET_RAW_RC on an absolute schedule, counting packets.

    Every wait in this file happens *between* sends, never instead of one. The
    firmware's MSP receiver gives up after RX_TIMEOUT_MS of silence, so a
    helper that went quiet in order to read status would put the aircraft into
    the failsafe the read was checking for. Polls therefore borrow a slice of
    a frame interval, and this keeps the schedule honest across them.
    """

    def __init__(self, link, rate):
        self.link = link
        self.period = 1.0 / rate
        self.epoch = time.time()
        self.sent = 0

    def send(self, channels):
        self.link.send_raw_rc(channels)
        self.sent += 1
        # An absolute schedule, not sleep(period): the send itself takes time,
        # and drift would show up as a slow link. A caller that spent time on
        # something else is caught up here rather than left behind.
        slack = (self.epoch + self.sent * self.period) - time.time()
        if slack > 0:
            time.sleep(slack)
        return self.sent

    def restart(self):
        """Drop to a clean schedule and return the new epoch.

        --seconds is documented as stick time, and the --reboot prologue can
        run for a minute on a schedule of its own. The main loop asks for a
        fresh one rather than inheriting a spent one, so a long revive does
        not silently eat the flight window.
        """
        self.epoch = time.time()
        self.sent = 0
        return self.epoch


def wait_for_fc(link, timeout=15.0):
    """Poll until the firmware answers. Returns the variant string, or None."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        cmd, payload, link.buf, _ = link.request(MSP_API_VERSION, timeout=0.5)
        if cmd == MSP_API_VERSION:
            cmd, payload, link.buf, _ = link.request(MSP_FC_VARIANT, timeout=0.5)
            variant = payload.decode("ascii", "replace") if cmd == MSP_FC_VARIANT else "?"
            return variant
        time.sleep(0.25)
    return None


def read_channels(link):
    """The channels as the firmware sees them, post-remap internal order."""
    cmd, payload, link.buf, _ = link.request(MSP_RC, timeout=0.5)
    if cmd != MSP_RC or len(payload) < 8:
        return None
    return list(struct.unpack_from("<%dH" % (len(payload) // 2), payload, 0))


def read_arming_flags(link, timeout=0.5):
    """armingFlags from MSP2_INAV_STATUS, or None if there was no reply.

    None means "not yet" and must never be read as "clear". The MSP listener
    is bound during init() but only serviced once the scheduler starts, so a
    socket that connects and then answers nothing is the ordinary state of a
    flight controller that is still coming up.
    """
    cmd, payload, link.buf, err = link.request(MSP2_INAV_STATUS, timeout=timeout)
    if cmd != MSP2_INAV_STATUS or err or len(payload) < 13:
        return None
    return struct.unpack_from("<I", payload, 9)[0]


def read_raw_imu(link):
    """The nine raw sensor words, or None. Used only as a liveness probe."""
    cmd, payload, link.buf, err = link.request(MSP_RAW_IMU, timeout=0.05)
    if cmd != MSP_RAW_IMU or err or len(payload) < 18:
        return None
    return payload[:18]


def wait_disarmed(link, args, repeater):
    """Stream AUX1 low with the sticks swinging until ARMED clears.

    Returns (ok, elapsed, flags). The swing is not decoration. Disarming with
    the ARM switch is gated on !failsafeIsActive() (rc_controls.c), and the
    only way out of a failsafe once the link is back is stick motion
    (failsafe.c: failsafeCheckStickMotion sums |ch - 1500| over roll, pitch
    and yaw against failsafe_stick_motion_threshold, 50 by default). Hold the
    sticks centred and an aircraft in failsafe never disarms on the switch --
    it waits out failsafe_off_delay and drops instead, and with that setting
    at 0 it never drops at all.
    """
    started = time.time()
    deadline = started + args.reboot_disarm_timeout
    next_poll = last_report = 0.0
    flags = None

    while time.time() < deadline:
        # Half a second each way, on two axes in anti-phase.
        swing = WIGGLE if int((time.time() - started) * 2) % 2 == 0 else -WIGGLE
        frame = [RC_MID] * 8
        frame[THROTTLE] = RC_MIN
        frame[4] = RC_MIN
        frame[ROLL] = stick(swing)
        frame[PITCH] = stick(-swing)
        repeater.send(frame)

        now = time.time()
        if now < next_poll:
            continue
        next_poll = now + 0.25

        # 150 ms, not the usual 200: the poll borrows time from the frame
        # interval, and the sum has to stay under RX_TIMEOUT_MS.
        flags = read_arming_flags(link, timeout=0.15)
        if flags is not None and not (flags & ARMED):
            return True, now - started, flags

        if now - last_report >= 1.0:
            last_report = now
            if flags is None:
                print("  %4.1fs  no status reply" % (now - started))
            else:
                print("  %4.1fs  still armed: flags=0x%08x (%s)"
                      % (now - started, flags, describe_flags(flags)))

    return False, time.time() - started, flags


def wait_dropped(link, timeout):
    """Wait for the old TCP connection to die. Returns (ok, elapsed).

    This is the reboot boundary, and the only observation in the whole
    prologue that a live process cannot fake. The ACK proves nothing: fcReboot
    spends a second in delay() before it re-execs, and the old image answers
    MSP for that whole second. "It answered again" is satisfied by a flight
    controller that never rebooted.
    """
    started = time.time()
    deadline = started + timeout
    while time.time() < deadline:
        try:
            link.sock.settimeout(max(0.005, deadline - time.time()))
            if not link.sock.recv(4096):
                return True, time.time() - started
        except socket.timeout:
            # Still holding the connection open. socket.timeout is a subclass
            # of OSError, so it has to be caught before the branch below.
            break
        except OSError:
            return True, time.time() - started
    return False, time.time() - started


def reconnect(link, timeout):
    """Rebind to the MSP port, retrying until it is really back.

    Returns (ok, ever_connected, elapsed). Retrying is not optional: the
    listener is not bound until init() returns, and init() blocks in
    simUeInit for up to the ten-second handshake window, so the port is
    refused for a while after the re-exec.

    A successful connect is not enough on its own either. tcpReceive only
    calls accept() while no client is connected, so a connect issued during
    the old image's last second lands in its listen backlog, succeeds from
    here, and is reset when it exits. That is why every attempt has to be
    followed by an actual reply before it counts.
    """
    started = time.time()
    deadline = started + timeout
    ever_connected = False
    while time.time() < deadline:
        try:
            sock = socket.create_connection((link.host, link.port), timeout=1.0)
        except OSError:
            time.sleep(0.5)
            continue
        ever_connected = True
        link.rebind(sock)
        try:
            alive = read_arming_flags(link, timeout=1.0) is not None
        except OSError:
            # The connection died inside the check. That is the backlog case
            # above, and it is expected, so it must not escape as a traceback.
            alive = False
        if alive:
            return True, ever_connected, time.time() - started
        time.sleep(0.5)
    return False, ever_connected, time.time() - started


def wait_revived(link, args, repeater):
    """Stream neutral sticks until the new process reports a live sim link.

    Two gates, both read from the same armingFlags word, both required:

      - SIMULATOR_MODE_SITL set. ue.c raises it in its UE_INIT case, which is
        reachable only once the handshake and the dataref re-subscription have
        succeeded, and simUeClose clears it. A process that came up deaf --
        the exact case this mode exists for -- starts at zero and stays there.
        This is the direct in-band probe of the thing being repaired.
      - No arming blockers set. Waiting for CALIBRATING alone is not enough:
        ARM_SWITCH latches on *any* remaining blocker at the moment AUX1 goes
        high (fc_core.c, "CHECK: Arming switch"), and because ARM_SWITCH is
        itself a blocker the latch then holds itself. The arm sequence has to
        start from a word that has been read clean, not assumed clean.

    MSP_API_VERSION is deliberately not the gate. A flight controller that
    missed the handshake window boots with no simulator link at all, logs
    "[SIM] Connection with UE simulator NOT established", and answers
    MSP_API_VERSION normally while being unable to arm or fly. Waiting on it
    would report success against a machine that can never take off.
    """
    started = time.time()
    deadline = started + args.reboot_revive_timeout
    next_poll = next_imu = last_report = 0.0
    sim_seen = False
    flags = None
    imu_prev = imu_now = None

    hold = [RC_MID] * 8
    hold[THROTTLE] = RC_MIN
    hold[4] = RC_MIN

    while time.time() < deadline:
        repeater.send(hold)

        now = time.time()
        if now < next_poll:
            continue
        next_poll = now + 0.25

        flags = read_arming_flags(link, timeout=0.15)
        if flags is None:
            continue

        if flags & ARMING_DISABLED_ARM_SWITCH:
            # AUX1 has been held low for this entire wait, and the switch flag
            # clears only when AUX1 goes low. Waiting cannot fix this, so do
            # not pretend to.
            print("error: ARMING_DISABLED_ARM_SWITCH is set while AUX1 is held "
                  "low, which is not a state that clears by waiting. "
                  "flags=0x%08x (%s)" % (flags, describe_flags(flags)),
                  file=sys.stderr)
            return False, now - started, flags

        sim_seen = sim_seen or bool(flags & SIMULATOR_MODE_SITL)

        if now >= next_imu:
            next_imu = now + 2.0
            sample = read_raw_imu(link)
            if sample is not None:
                imu_prev, imu_now = imu_now, sample

        if sim_seen and not (flags & ARMING_DISABLED_ALL_FLAGS):
            print("sim link back after %.1fs: flags=0x%08x (%s)"
                  % (now - started, flags, describe_flags(flags)))
            print("  armable: no arming blockers set")
            if imu_prev is None or imu_now is None:
                print("  sensors: not sampled (the link came back first)")
            elif imu_prev != imu_now:
                print("  fresh sensors: MSP_RAW_IMU moved between samples")
            else:
                print("  MSP_RAW_IMU identical across samples (a still "
                      "airframe can quantise the same way; not proof either way)")
            return True, now - started, flags

        if now - last_report >= 2.0:
            last_report = now
            print("  %4.1fs  waiting: flags=0x%08x (%s)"
                  % (now - started, flags, describe_flags(flags)))

    return False, time.time() - started, flags


def reboot_and_revive(link, args, repeater, variant):
    """Reboot the FC and wait for it to re-subscribe to the simulator.

    Returns None once the link is provably back, or an exit code. No step can
    be skipped: each one is the only evidence for something the next one
    assumes, and the failure mode of getting it wrong is not a hang but a
    confident report about an aircraft that never rebooted.
    """
    index, sane_port = vehicle_index(args.port)
    print("target: %s:%d is %s, VehicleIndex %d%s"
          % (link.host, link.port, variant, index,
             "" if sane_port else " (NOT a 5760+10k port -- check --port)"))

    if variant.strip() != "INAV":
        print("error: %s:%d identifies as %r, not INAV. Refusing to reboot a "
              "flight controller we cannot name." % (link.host, link.port, variant),
              file=sys.stderr)
        return 1

    flags = read_arming_flags(link, timeout=1.0)
    if flags is None:
        print("error: %s:%d answers MSP but not MSP2_INAV_STATUS."
              % (link.host, link.port), file=sys.stderr)
        return 1
    print("target: flags=0x%08x (%s)" % (flags, describe_flags(flags)))

    if flags & ARMED:
        print("disarming: MSP_REBOOT is refused while armed. AUX1 low with the "
              "sticks swinging, %.0fs budget -- the swing is what clears a "
              "failsafe." % args.reboot_disarm_timeout)
        ok, elapsed, flags = wait_disarmed(link, args, repeater)
        if not ok:
            print("error: still armed after %.0fs, so MSP_REBOOT was not sent "
                  "and nothing has changed." % elapsed, file=sys.stderr)
            if flags is not None:
                print("       flags=0x%08x (%s)" % (flags, describe_flags(flags)),
                      file=sys.stderr)
            print("       An armed aircraft in failsafe disarms itself after "
                  "failsafe_off_delay -- 20s by default, and never if that "
                  "setting is 0. `docker restart inav-sitl-%d` is the way out."
                  % index, file=sys.stderr)
            return 2
        print("disarmed after %.1fs" % elapsed)

    cmd, payload, link.buf, err = link.request(MSP_REBOOT, timeout=1.0)
    if err:
        print("error: the FC refused MSP_REBOOT. It is still running unchanged. "
              "A refusal is framed '$M!', which is why this is not being read "
              "as a timeout.", file=sys.stderr)
        return 3
    if cmd is None:
        print("note: no ACK -- the link may have gone down inside the request "
              "window. Waiting for the connection to drop either way.")
    else:
        print("rebooting: MSP_REBOOT acknowledged")

    ok, elapsed = wait_dropped(link, args.reboot_drop_timeout)
    if not ok:
        print("error: the connection survived %.0fs after the ACK, so the "
              "image was not replaced. The aircraft is still the one that was "
              "already running." % args.reboot_drop_timeout, file=sys.stderr)
        return 4
    print("reboot confirmed: the FC closed the old connection after %.1fs" % elapsed)

    ok, ever_connected, elapsed = reconnect(link, args.reboot_connect_timeout)
    if not ok:
        if not ever_connected:
            print("error: the MSP port never came back after %.0fs. A failed "
                  "exec leaves the container Up with PID 1 spinning and every "
                  "file descriptor closed, and it writes no log line. Check "
                  "`docker logs inav-sitl-%d` for the absence of a second "
                  "'UE simulator detected', then `docker restart inav-sitl-%d`."
                  % (elapsed, index, index), file=sys.stderr)
            return 5
        print("error: connected to the port but got no reply in %.0fs. Another "
              "MSP client probably owns UART1 -- stop it before rebooting."
              % elapsed, file=sys.stderr)
        return 6
    print("reconnected after %.1fs" % elapsed)

    ok, elapsed, flags = wait_revived(link, args, repeater)
    if not ok:
        if flags is not None and flags & ARMING_DISABLED_ARM_SWITCH:
            return 7      # wait_revived has already said what is wrong
        print("error: the rebooted FC never reported a live simulator link in "
              "%.0fs." % elapsed, file=sys.stderr)
        if flags is not None:
            print("       flags=0x%08x (%s)" % (flags, describe_flags(flags)),
                  file=sys.stderr)
            if flags & ARMING_DISABLED_RC_LINK:
                print("       RC_LINK is set: the eeprom's receiver_type is "
                      "probably SIM, not MSP. This client's frames are landing "
                      "in a buffer nobody reads -- check `docker logs "
                      "inav-sitl-%d` for the eeprom it loaded." % index,
                      file=sys.stderr)
        print("       The firmware spends ten seconds waiting for the simulator "
              "handshake, and only once per boot. If Play-In-Editor is not in "
              "Play right now, reboot again with it running -- running this "
              "client a second time will not help.", file=sys.stderr)
        return 7

    print("to confirm out of band (the container was NOT restarted):")
    print("  docker logs inav-sitl-%d | grep 'UE simulator detected'   "
          "# a SECOND line" % index)
    print("  docker inspect -f '{{.State.StartedAt}}' inav-sitl-%d    "
          "# must be UNCHANGED" % index)
    print("note: this is a real reboot -- any parameter changed since the last "
          "`save` is gone.")
    return None


def main():
    ap = argparse.ArgumentParser(
        description="Send RC sticks to a SITL container over MSP.",
        epilog="The container must have been started with --receiver msp, or its "
               "receiver_type is SIM and these packets go nowhere.")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=None,
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
    ap.add_argument("--reboot", action="store_true",
                    help="reboot the FC first -- a re-exec in the same container "
                         "-- so it re-subscribes to the simulator after a "
                         "Play-In-Editor restart. Runs before the stick loop, so "
                         "it composes with --arm/--throttle. Requires --port.")
    ap.add_argument("--reboot-disarm-timeout", type=float, default=35.0,
                    help="s to wait for a disarm before rebooting; MSP_REBOOT is "
                         "refused while armed (default 35)")
    ap.add_argument("--reboot-drop-timeout", type=float, default=5.0,
                    help="s to wait for the old connection to drop after the "
                         "reboot ACK. A connection that survives means no reboot "
                         "happened (default 5)")
    ap.add_argument("--reboot-connect-timeout", type=float, default=20.0,
                    help="s to keep retrying the MSP connection while the FC "
                         "re-execs; the port is unbound until the simulator "
                         "handshake finishes (default 20)")
    ap.add_argument("--reboot-revive-timeout", type=float, default=30.0,
                    help="s to wait for the simulator link to come back after "
                         "the reboot (default 30)")
    args = ap.parse_args()

    if args.port is None:
        if args.reboot:
            print("error: --reboot requires an explicit --port. Its default is "
                  "5760, which is VehicleIndex 0, and rebooting the wrong "
                  "aircraft looks exactly like rebooting the right one.",
                  file=sys.stderr)
            return 1
        args.port = 5760

    period = 1.0 / args.rate
    if period * 1000.0 > RX_TIMEOUT_MS:
        print("error: %.1f Hz is slower than the %d ms receiver timeout; the "
              "aircraft would failsafe between packets." % (args.rate, RX_TIMEOUT_MS),
              file=sys.stderr)
        return 1

    try:
        link = MspLink(args.host, args.port)
    except OSError as err:
        print("error: cannot connect to %s:%d: %s" % (args.host, args.port, err),
              file=sys.stderr)
        print("       Is the container up, and did it bind this port?",
              file=sys.stderr)
        return 1

    variant = wait_for_fc(link)
    if variant is None:
        print("error: no MSP response on %s:%d." % (args.host, args.port), file=sys.stderr)
        print("       Is the container up, and did it bind this port? "
              "`docker logs inav-sitl-N` should show 'Bind TCP [::]:%d to UART1'."
              % args.port, file=sys.stderr)
        return 1
    print("connected to %s:%d (%s)" % (args.host, args.port, variant))

    repeater = Repeater(link, args.rate)

    if args.reboot:
        try:
            rc = reboot_and_revive(link, args, repeater, variant)
        except OSError as err:
            # The prologue streams into a socket that is deliberately going
            # away, so a send or recv can raise at any point in it. Losing the
            # link here means the port came back unusable, not that anything
            # is recoverable from in here.
            print("error: the MSP link died during the reboot handshake: %s"
                  % err, file=sys.stderr)
            print("       `docker restart inav-sitl-%d` is the fallback."
                  % vehicle_index(args.port)[0], file=sys.stderr)
            link.close()
            return 5
        if rc is not None:
            link.close()
            return rc

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

    # Taken here, after the --reboot prologue, so --seconds measures stick time
    # and a long revive does not eat the flight window.
    started = repeater.restart()
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
            if repeater.sent < settle_packets:
                frame = settle_channels
            elif repeater.sent < arm_packets:
                frame = arm_channels
            else:
                frame = channels
            repeater.send(frame)

            if repeater.sent % int(args.rate) == 0:
                seen = read_channels(link)
                if seen is not None:
                    # Deliberately shown in the firmware's own order. It will
                    # not match what was sent above -- see the note on AETR.
                    print("  %5.1fs  fc sees: %s" % (time.time() - started, seen[:6]))
    except KeyboardInterrupt:
        print()
    except OSError as err:
        # The FC or the container went away mid-flight. Drop through to the
        # release below, which is already written to tolerate a dead socket.
        print("\nlink lost: %s" % err, file=sys.stderr)
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
        print("sent %d packets, sticks released." % repeater.sent)

    return 0


if __name__ == "__main__":
    sys.exit(main())
