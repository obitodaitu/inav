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
import csv
import socket
import struct
import sys
import time

MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_REBOOT = 68
MSP_RAW_IMU = 102
MSP_MOTOR = 104
MSP_RC = 105
MSP_ATTITUDE = 108
MSP_SET_RAW_RC = 200
MSP2_INAV_STATUS = 0x2000

# Payload offsets, read out of the firmware rather than guessed.
#
# MSP_RAW_IMU is three accel words, then three gyro words, then three mag words
# (fc_msp.c:519-534). The gyro words come from gyroRateDps(), an int16_t of
# deg/s with the boot zero already subtracted (gyro.c:604-611) -- so they are
# signed, and they are the rates the PID loop is actually working from rather
# than raw counts.
#
# MSP2_INAV_STATUS opens with cycleTime in microseconds (fc_msp.c:501, written at
# fc_core.c:927). It is NOT the control-iteration period, and reading it as one
# will mislead: cycleTime is getTaskDeltaTime(TASK_SELF), the interval since the
# scheduler last called taskMainPidLoop -- and most of those calls do nothing.
# The sensor half of that task sits behind the sim-frame token (fc_core.c:951-962),
# so gyroFilter() and imuUpdateAttitude() run once per received frame while
# pidController() runs on every scheduler tick in between. A flat cycleTime
# therefore says the scheduler is on time and nothing at all about the rate the
# measurement actually refreshes at. armingFlags is the u32 at offset 9.
RAW_IMU_RATES_OFFSET = 6
# ...and the three words in front of the gyro words are the accelerometer, in
# the body frame (fc_msp.c:519-521). They are written as accADCf * 512, and
# accADCf is in G, so 512 counts is one G. Free to read -- same request.
RAW_IMU_ACC_OFFSET = 0
ACC_LSB_PER_G = 512.0
STATUS_CYCLE_TIME_OFFSET = 0
STATUS_ARMING_FLAGS_OFFSET = 9

MSP_STATUS = 101
# MSP_ALTITUDE is the estimated Z position in cm, then the vertical velocity in
# cm/s, then the raw barometric altitude (fc_msp.c:685-693). read_altitude_cm
# uses only the first word -- "is it off the ground" needs no more than the sign
# of an estimated position -- but read_altitude_vel_cm keeps the second one,
# which is the estimator's own velocity and does not have to be differenced.
MSP_ALTITUDE = 109
MSP2_PID = 0x2030
MSP2_SET_PID = 0x2031

# MSP_RC_TUNING writes the live control profile's rate/TPA block as 11 bytes
# (fc_msp.c:761-772) and MSP_SET_RC_TUNING takes 10 or 11 back in the same field
# order (fc_msp.c:2087-2107). Same shape as MSP2_PID above, and for the same
# reason: a 10-byte write is accepted too, so the read side is what pins the
# layout, and reading 11 back makes the round trip lossless.
MSP_RC_TUNING = 111
MSP_SET_RC_TUNING = 204

# Offsets into that block, from the field order the firmware writes.
RC_TUNING_RATES = slice(2, 5)            # stabilized.rates[ROLL,PITCH,YAW]
RC_TUNING_TPA_RATE = 5                   # throttle.dynPID
RC_TUNING_TPA_BREAKPOINT = slice(8, 10)  # throttle.pa_breakpoint, u16 LE

# settings.yaml:1389-1394 -- tpa_rate is field throttle.dynPID, min 0, max 200.
# settings.yaml:1395-1400 -- tpa_breakpoint is field throttle.pa_breakpoint,
# PWM_RANGE_MIN..MAX, which is 1000..2000 (rx.h:30-31). Both bounds are enforced
# here because the firmware only enforces one of them: the write path clamps the
# rate with MIN(tmp_u8, TPA_RATE_MAX) but stores the breakpoint verbatim
# (fc_msp.c:2098-2101), so a bad breakpoint would be accepted and then never
# fire.
TPA_RATE_MAX = 200
TPA_BREAKPOINT_MIN = 1000
TPA_BREAKPOINT_MAX = 2000

# pid.h:59-71, in order. Only the first three are per-axis gain banks on a
# multirotor; the rest are the outer loops, which is why PID_LEVEL is a separate
# experiment from anything that touches this table.
PID_NAMES = ("ROLL", "PITCH", "YAW", "POS_Z", "POS_XY", "VEL_XY",
             "SURFACE", "LEVEL", "HEADING", "VEL_Z", "POS_HEADING")

# MSP_STATUS writes the box mode flags at a fixed offset, MSP2_INAV_STATUS writes
# a boxBitmask_t of a width that is not visible from here. Four bytes is all the
# boxes this cares about need, so the deprecated command is the right one --
# its layout is pinned by the firmware (fc_msp.c:487).
STATUS_BOX_FLAGS_OFFSET = 6

# pid.h:58-72. The level controller is not a per-axis gain bank like the others:
# it is the whole outer loop, and it is the only part of the control law that
# treats roll and pitch differently from yaw.
PID_ITEM_COUNT = 11
PID_LEVEL = 7

MSP_BOXIDS = 119

# Box identities, transcoded from the `boxes[]` table at fc_msp_box.c:49-107.
# These are the permanentIds, NOT the boxId_e enum in rc_modes.h: that enum is
# the internal id used to index activeBoxes[], and its numbering diverges from
# this one -- BOXMANUAL is 10 there and 12 here, BOXFAILSAFE 18/27,
# BOXAIRMODE 20/29, BOXANGLEHOLD 55/64. The two are easy to conflate, the enum
# is the one that turns up first when searching, and only this one is on the
# wire, so the table above is the source that has to be read.
BOX_NAMES_BY_ID = {
    0: "ARM", 1: "ANGLE", 2: "HORIZON", 3: "NAV ALTHOLD", 5: "HEADING HOLD",
    6: "HEADFREE", 7: "HEADADJ", 8: "CAMSTAB", 10: "NAV RTH",
    11: "NAV POSHOLD", 12: "MANUAL", 13: "BEEPER", 15: "LEDS OFF",
    16: "LIGHTS", 19: "OSD OFF", 20: "TELEMETRY", 21: "AUTO TUNE",
    26: "BLACKBOX", 27: "FAILSAFE", 28: "NAV WP", 29: "AIR MODE",
    30: "HOME RESET", 31: "GCS NAV", 32: "FPV ANGLE MIX", 33: "SURFACE",
    34: "FLAPERON", 35: "TURN ASSIST", 36: "NAV LAUNCH", 37: "SERVO AUTOTRIM",
    39: "CAMERA CONTROL 1", 40: "CAMERA CONTROL 2", 41: "CAMERA CONTROL 3",
    42: "OSD ALT 1", 43: "OSD ALT 2", 44: "OSD ALT 3", 45: "NAV COURSE HOLD",
    46: "MC BRAKING", 47: "USER1", 48: "USER2", 49: "LOITER CHANGE",
    50: "MSP RC OVERRIDE", 51: "PREARM", 52: "TURTLE", 53: "NAV CRUISE",
    54: "AUTO LEVEL TRIM", 55: "WP PLANNER", 56: "SOARING", 57: "USER3",
    58: "USER4", 59: "MISSION CHANGE", 60: "BEEPER MUTE",
    61: "MULTI FUNCTION", 62: "MIXER PROFILE 2", 63: "MIXER TRANSITION",
    64: "ANGLE HOLD", 65: "GIMBAL LEVEL TILT", 66: "GIMBAL LEVEL ROLL",
    67: "GIMBAL CENTER", 68: "GIMBAL HEADTRACKER",
}
BOX_ID_ARM = 0
BOX_ID_ANGLE = 1
BOX_ID_HORIZON = 2
BOX_ID_MANUAL = 12          # spelled "MANUAL" here; boxId_e calls it BOXMANUAL

# Which boxes actually put the level controller into the control law -- and it
# is not "every ANGLE-ish box". fc_core.c:690-701 enables ANGLE_MODE for
# BOXANGLE and HORIZON_MODE for BOXHORIZON, but enables ANGLEHOLD_MODE only
# under STATE(AIRPLANE); this airframe is a multirotor, so that branch can never
# fire no matter what the switch does.
LEVEL_LOOP_IDS = (BOX_ID_ANGLE, BOX_ID_HORIZON)

# Only the modes worth a line in output that is already carrying numbers.
NOTABLE_BOX_IDS = (0, 1, 2, 3, 12, 27, 29, 35, 64)

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

# How often the in-flight monitor asks for body rates and the loop time. Each
# sample is two more MSP round trips on a link already carrying --rate stick
# packets, so it roughly doubles the traffic; --monitor-hz 0 turns it off.
#
# 25 Hz decides oscillations up to about 9 Hz cleanly and flags the rest as "at
# or above the readable band". That is the right default because this is a rate
# loop, and a rate loop that rings does it in the single digits. It is also the
# honest default: 25 Hz cannot tell 5 Hz from 20 Hz, and a reading of 5 Hz from
# this setting means "5 Hz, or whatever folds onto it". When a run comes back
# with a number that matters, run it again at --monitor-hz 50 -- a real mode
# reports the same frequency, an alias moves.
#
# The worst a silent flight controller can stall the stick stream is two 50 ms
# request timeouts per sample. At 25 Hz that stays well inside the 200 ms
# receiver timeout; going higher eats that margin, so it is a deliberate step
# rather than a default.
DEFAULT_MONITOR_HZ = 25

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

# Named because it is the flag a stale sim link wears. The firmware raises it
# when a sensor it needs is not healthy, and a ue.c that subscribed to nothing
# feeds it no gyro at all -- so this, not RC_LINK and not ARM_SWITCH, is what a
# stranded container looks like from the client's side. SIMULATOR_MODE_SITL
# stays set throughout, which is what makes it worth its own name: the
# "is there a simulator link" test passes while the link carries nothing.
ARMING_DISABLED_HARDWARE_FAILURE = 1 << 15


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
        #
        # Clamped in both directions, because this schedule is read off
        # time.time() and time.time() can step. Neither direction is
        # hypothetical on this host - the WSL2 guest's clock is corrected in
        # multi-second steps, not slewed - and both of them end the flight:
        #
        #   forward step -> slack goes deeply negative, the wait is skipped
        #     for every remaining packet at once, and the debt is repaid as a
        #     burst of RC frames a second or two of packets long.
        #   backward step -> slack goes deeply positive, and the loop parks
        #     here for the length of the step. RX_TIMEOUT_MS of silence is
        #     enough for the firmware's MSP receiver to give up on us, which
        #     is the failsafe this class exists to avoid.
        #
        # So the sleep is capped at one frame, and a schedule that has fallen
        # more than a frame behind is re-anchored rather than repaid. Re-anchor
        # and not restart(): sent drives the settle/arm staging above, and
        # zeroing it mid-flight would replay that staging under the aircraft.
        # Same idiom as Monitor.sample, which skips its schedule forward.
        slack = (self.epoch + self.sent * self.period) - time.time()
        if slack > 0:
            time.sleep(min(slack, self.period))
        elif slack < -self.period:
            self.epoch = time.time() - self.sent * self.period
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


def describe_frequency(values, span):
    """How fast one axis is moving, from the crossings inside the window.

    Crossings are taken about the window's own mean, never about zero. A sway
    riding on a trim offset -- which is exactly what a one-sided lean looks like
    -- does not cross zero at all, so counting about zero would report the
    liveliest axis in the flight as "not oscillating", the opposite of the truth.

    Peak to peak alone cannot tell a 2 Hz sway from a 12 Hz buzz, and those two
    want opposite fixes, so the crossing count is what makes the range readable.
    Fewer than two crossings in a window cannot give a period at all, and that is
    reported as such rather than as a frequency.

    Sampling caps it. At 25 Hz a one-second window holds 24 gaps, so 12 Hz is the
    top of the readable band, and anything up against that ceiling is reported as
    a floor -- "at least this fast" is true where a precise number would be
    invented.
    """
    if span <= 0 or len(values) < 3:
        return "?"
    step = span / float(len(values) - 1)
    mean = sum(values) / float(len(values))

    times = []
    for i in range(len(values) - 1):
        a, b = values[i], values[i + 1]
        if (a < mean) != (b < mean):
            # Where between the two samples it actually crossed, so the period
            # comes out of the data instead of off the sample grid.
            frac = 0.0 if b == a else (mean - a) / float(b - a)
            times.append((i + frac) * step)

    # The period is the mean gap between crossings, not the count divided by the
    # window. Dividing by the window loses the crossing that sits just outside
    # it, and a signal with a whole number of cycles in the window then reads
    # exactly 0.5 Hz slow -- a bias that points the wrong way every time.
    if not times:
        return "0.0Hz"
    if len(times) < 2:
        return "<1Hz"
    gaps = [b - a for a, b in zip(times, times[1:])]
    gap = sum(gaps) / len(gaps)
    if gap <= 0:
        return "?"

    # The ceiling test is on the crossing SPACING rather than on the frequency
    # the arithmetic produced: crossings landing one per sample means the signal
    # is at the sample rate's limit whatever number falls out, and quoting that
    # number would be inventing one.
    #
    # Note what this cannot do. Sampling at 25 Hz folds down everything above
    # 12.5 Hz, and a 20 Hz signal sampled at 25 Hz is bit for bit the sequence a
    # 5 Hz signal would have produced. No arithmetic recovers the 20. So every
    # reading under the ceiling means "this, or something that aliases onto it",
    # and sampling twice as fast to see whether the number stays put is the only
    # way to tell those apart.
    step = span / float(len(values) - 1)
    if gap <= 1.3 * step:
        return ">=%.0fHz" % ((len(values) - 1) / 2.0 / span)

    return "%.1fHz" % (1.0 / (2.0 * gap))


def format_loop_window(elapsed, span, rates, cycles, flags, missed, asked, fresh):
    """One line describing what the flight loop did over the last window.

    Range and frequency per axis, not just a mean. Range because the failure this
    watches for is an oscillation, and an oscillation's mean is zero exactly when
    it is at its worst; frequency because the rate of the oscillation is what
    tells the two candidate mechanisms apart.

    The mean is still printed alongside, since "swings both ways around zero" and
    "sits off to one side and wobbles" have identical ranges and completely
    different causes.

    fresh is how many samples differed from the one before them, which is this
    client's only handle on the rate the simulator is really producing sensor
    data at, and it is reported because cycleTime cannot answer that question.
    The control law runs on the scheduler, not on the simulator: the sim-frame
    token at fc_core.c:951-961 gates gyroFilter and imuUpdateAttitude, while
    pidController at :993 is outside it and runs at getLooptime() whatever the
    simulator is doing. So a tidy cycleTime is exactly what a throttled sim
    looks like too. What a low sim rate costs is staleness in the feedback path
    -- one frame every 333 ms is a third of a second of pure delay -- and a
    delayed loop with no damping to spare rings. That is why this number is
    printed rather than left to the loop time to reveal.
    """
    lead = "  %5.1fs  loop:" % elapsed
    if not rates:
        return "%s no reply (%d of %d samples lost)" % (lead, missed, asked)
    axes = []
    for i, name in enumerate(("roll", "pitch", "yaw")):
        seen = [r[i] for r in rates]
        axes.append("%s %+d..%+d avg%+d %s"
                    % (name, min(seen), max(seen),
                       int(round(sum(seen) / float(len(seen)))),
                       describe_frequency(seen, span)))
    text = "%s %s dps" % (lead, "  ".join(axes))
    if cycles:
        text += "  |  cycle %d..%d us" % (min(cycles), max(cycles))
    if span > 0 and rates:
        # The ceiling is this client's own sampling rate: at 25 Hz it cannot see
        # a 60 Hz sim, only that it is not obviously slow. Anything under the
        # ceiling is a direct reading; anything pinned at it means "at least
        # this fast, and this instrument cannot say more".
        if fresh >= 0.9 * len(rates):
            text += "  |  sens>=%.0fHz" % (len(rates) / span)
        else:
            text += "  |  sens %.1fHz (%d new)" % (fresh / span, fresh)
    if flags is None:
        state = "status ?"
    else:
        state = "ARMED" if flags & ARMED else "disarmed"
        blocked = describe_flags(flags & ARMING_DISABLED_ALL_FLAGS)
        if blocked != "none set":
            state += " blocked:" + blocked
    return "%s  |  %s  |  %d/%d" % (text, state, len(rates), asked)


def format_attitude_window(elapsed, span, attitudes):
    """One line describing the attitude estimate over the last window.

    The rates above are the inner loop; this is the outer one. ANGLE mode
    commands an angle, so the angle is what "it sways" means to whoever is
    watching the airframe, and the printed rate line cannot answer it: a ring in
    the rate loop can decay to a couple of dps while the attitude still walks,
    because an angle is the integral of a rate and a small bias integrates into
    a large excursion. Print both or the two get blamed for each other.

    Degrees, not decidegrees -- the firmware has already converted (see
    read_attitude). Yaw gets no frequency readout: a heading that wraps through
    zero puts a +359..+1 range around a number that is not oscillating, and the
    wrap is in the instrument rather than in the airframe.
    """
    if not attitudes:
        return "  %5.1fs  att:  no reply" % elapsed
    parts = []
    for i, name in enumerate(("roll", "pitch", "yaw")):
        seen = [a[i] for a in attitudes]
        text = "%s %+.1f..%+.1f avg%+.1f" % (name, min(seen), max(seen),
                                            sum(seen) / float(len(seen)))
        if i < 2:
            text += " %s" % describe_frequency(seen, span)
        parts.append(text)
    # The sample count is printed because this is the one axis of the window
    # that can come back short without the rate line above showing it.
    return "  %5.1fs  att: %s  |  n=%d" % (elapsed, "  ".join(parts), len(attitudes))


class CsvRecorder:
    """A row sink that lands on disk as it goes.

    csv.writer owns the formatting and the file object owns the buffer, so the
    two travel together here rather than as a pair of parameters threaded
    through LoopMonitor.

    Every row is flushed on purpose. The runs worth recording are the ones that
    end with the airframe on its back or the container restarting underneath it,
    and a buffered recorder would lose exactly the last second -- the part being
    looked for.
    """

    HEADER = (["t"] + ["gyro_%s" % a for a in "xyz"]
              + ["acc_%s" % a for a in "xyz"]
              + ["att_roll", "att_pitch", "att_yaw"]
              + ["m%d" % i for i in range(1, 9)])

    def __init__(self, path):
        self.file = open(path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.HEADER)
        self.file.flush()

    def writerow(self, row):
        self.writer.writerow(row)
        self.file.flush()

    def close(self):
        self.file.close()


class LoopMonitor:
    """Samples the body rates and the flight loop time while the sticks stream.

    This exists to catch a limit cycle, and one sample a second cannot: a 2-5 Hz
    oscillation aliases into noise at that rate. So it samples quickly and
    prints a peak-to-peak summary once a second, which keeps the terminal
    readable without throwing away the thing being measured.

    Both reads are extra traffic between stick packets. They borrow a slice of a
    frame interval like every other wait in this file, and a missing reply is
    counted rather than raised -- a monitor that gave up on a dropped sample
    would be least reliable exactly when it mattered.
    """

    def __init__(self, link, rate, record=None, clock=None):
        self.link = link
        self.period = 1.0 / rate
        self.next = time.time() + self.period
        self.window_start = time.time()
        self.rates = []
        self.attitudes = []
        self.cycles = []
        self.flags = None
        self.missed = 0
        # Deliberately not cleared per window by take(): the last sample of one
        # window and the first of the next are adjacent in time, so forgetting
        # the previous value there would count that first sample as fresh and
        # inflate every window's reading.
        self.last_rates = None
        self.fresh = 0
        # The CSV sink, when one was asked for, and the origin its timestamps
        # are measured from. Every row is flushed as it is written: the runs
        # worth recording are the ones that end with the airframe on its back or
        # the container restarting under it, and a recorder holding the last
        # second in a buffer would lose exactly the part being looked for.
        self.record = record
        self.clock = clock if clock is not None else self.window_start

    def due(self, now):
        return now >= self.next

    def sample(self, now):
        # Skip the schedule forward rather than firing a burst of catch-up
        # samples: the schedule is there to space them out, not to repay them.
        while self.next <= now:
            self.next += self.period
        imu = read_imu(self.link)
        rates = None
        if imu is None:
            self.missed += 1
        else:
            rates = imu[0]
            # The gyro only moves when the simulator produces a new frame -- the
            # firmware copies the injected sample through and does not invent
            # intermediate ones -- so a reading equal to the previous reading
            # means no frame arrived in between. Counting the ones that differ
            # is therefore a direct measurement of the sim rate, and it is the
            # only one available over this link.
            #
            # It can undercount, and the way it fails is worth knowing: if two
            # consecutive frames happened to carry the same integer count, the
            # second is scored as a repeat. During a live oscillation the
            # readings move every frame, so that needs the airframe to be almost
            # still, where the answer does not matter.
            if rates != self.last_rates:
                self.fresh += 1
                self.last_rates = rates
            self.rates.append(rates)
        # The attitude the ANGLE loop is fed. Read here rather than only under
        # --record, because it is the half that answers "is it swaying" and a
        # run without a CSV is exactly the run someone is watching.
        #
        # A missing reply is not counted in self.missed: that counter is the
        # denominator of the rate line's "n of m samples", and folding a second
        # command's losses into it would report a link fault on a window where
        # every rate sample arrived. The attitude line carries its own count.
        attitude = read_attitude(self.link)
        if attitude is not None:
            self.attitudes.append(attitude)
        cycle, flags = read_status(self.link)
        if cycle is not None:
            self.cycles.append(cycle)
            self.flags = flags
        if self.record is not None:
            self.write_row(now, imu, attitude)

    def write_row(self, now, imu, attitude):
        """One CSV row: what the loop was told, and what it did about it.

        One more read per sample (motors), so this stays off unless --record
        asked for it. It supplies the quantity no print can: whether the mixer
        still had anywhere to go. A loop that oscillates with motors in the
        middle of their range is a tuning fault; the same oscillation with a
        pair pinned at a rail is an authority fault, and the printed summary
        looks identical either way.

        The attitude comes in as an argument rather than being read again here,
        because sample() already read it for the printed line and a second
        request would be a duplicate round trip on a link whose budget is
        measured in them.
        """
        motors = read_motors(self.link)
        row = [round(now - self.clock, 4)]
        row += list(imu[0]) if imu is not None else [None, None, None]
        row += list(imu[1]) if imu is not None else [None, None, None]
        row += list(attitude) if attitude is not None else [None, None, None]
        row += list(motors) if motors is not None else [None] * 8
        # No flush here: CsvRecorder flushes on every row, and it owns both
        # halves of that. One place decides how durable a row is.
        self.record.writerow(row)

    def take(self, now):
        """Close the window and hand back what it saw, plus how long it really was.

        The span is measured here rather than assumed from the print cadence:
        it is the denominator of the frequency reading, so guessing it would put
        a wrong number behind a confident label.
        """
        out = (self.rates, self.attitudes, self.cycles, self.flags, self.missed,
               now - self.window_start, self.fresh)
        self.rates, self.attitudes, self.cycles, self.flags, self.missed, \
            self.fresh = [], [], [], None, 0, 0
        self.window_start = now
        return out


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
    return struct.unpack_from("<I", payload, STATUS_ARMING_FLAGS_OFFSET)[0]


def read_raw_imu(link):
    """The nine raw sensor words, or None. Used only as a liveness probe."""
    cmd, payload, link.buf, err = link.request(MSP_RAW_IMU, timeout=0.05)
    if cmd != MSP_RAW_IMU or err or len(payload) < 18:
        return None
    return payload[:18]


def read_imu(link, timeout=0.05):
    """(rates in deg/s, accel in G) from one MSP_RAW_IMU, or None.

    One request for both halves. The accel words sit in front of the gyro words
    in the same packet, and this rides between stick packets on a link whose
    budget is measured in them, so the second quantity has to be free to be
    worth having -- and it is.

    The timeout is short deliberately. This rides between stick packets, and the
    firmware's receiver gives up after RX_TIMEOUT_MS of silence -- a slow read
    would cause the failsafe it is there to observe.
    """
    cmd, payload, link.buf, err = link.request(MSP_RAW_IMU, timeout=timeout)
    if cmd != MSP_RAW_IMU or err or len(payload) < 18:
        return None
    rates = struct.unpack_from("<3h", payload, RAW_IMU_RATES_OFFSET)
    raw_acc = struct.unpack_from("<3h", payload, RAW_IMU_ACC_OFFSET)
    return rates, tuple(v / ACC_LSB_PER_G for v in raw_acc)


def read_rates(link, timeout=0.05):
    """The three body rates in deg/s as (roll, pitch, yaw), or None."""
    imu = read_imu(link, timeout=timeout)
    return imu[0] if imu is not None else None


def read_attitude(link, timeout=0.05):
    """(roll, pitch, yaw) in degrees as the firmware estimates them, or None.

    roll and pitch arrive in decidegrees and yaw in degrees (fc_msp.c:678-681,
    where only yaw is passed through DECIDEGREES_TO_DEGREES). All three are read
    signed: yaw runs -180..180 and the firmware writes it as a u16, so a
    westward heading comes back as a huge positive number unless it is unpacked
    as int16.

    This is the estimate the ANGLE loop is fed, and it comes from the same
    fusion the controller uses, so it answers "is the attitude estimate itself
    swinging" separately from "is the airframe swinging".
    """
    cmd, payload, link.buf, err = link.request(MSP_ATTITUDE, timeout=timeout)
    if cmd != MSP_ATTITUDE or err or len(payload) < 6:
        return None
    roll, pitch, yaw = struct.unpack_from("<3h", payload, 0)
    return roll / 10.0, pitch / 10.0, float(yaw)


def read_motors(link, timeout=0.05):
    """The eight motor outputs as the mixer left them, or None.

    Raw 1000..2000 PWM, before any protocol conversion -- so a value sitting at
    either rail is the mixer out of authority, and that is a different fault
    from a loop that is merely oscillating with room to spare.
    """
    cmd, payload, link.buf, err = link.request(MSP_MOTOR, timeout=timeout)
    if cmd != MSP_MOTOR or err or len(payload) < 16:
        return None
    return struct.unpack_from("<8H", payload, 0)


def read_status(link, timeout=0.05):
    """(cycleTime in us, armingFlags) from MSP2_INAV_STATUS, or (None, None)."""
    cmd, payload, link.buf, err = link.request(MSP2_INAV_STATUS, timeout=timeout)
    if cmd != MSP2_INAV_STATUS or err or len(payload) < 13:
        return None, None
    cycle = struct.unpack_from("<H", payload, STATUS_CYCLE_TIME_OFFSET)[0]
    flags = struct.unpack_from("<I", payload, STATUS_ARMING_FLAGS_OFFSET)[0]
    return cycle, flags


def read_box_flags(link, timeout=0.2):
    """The 32 box-mode bits from MSP_STATUS, or None."""
    cmd, payload, link.buf, err = link.request(MSP_STATUS, timeout=timeout)
    if cmd != MSP_STATUS or err or len(payload) < STATUS_BOX_FLAGS_OFFSET + 4:
        return None
    return struct.unpack_from("<I", payload, STATUS_BOX_FLAGS_OFFSET)[0]


def read_box_ids(link, timeout=0.2):
    """The permanent box ids; position i is bit i of the mode mask, or None.

    THIS IS THE MAPPING, and it is not a constant. initActiveBoxIds
    (fc_msp_box.c:185-378) fills activeBoxIds[] with a run of ADD_ACTIVE_BOX
    calls, most of them behind runtime feature and sensor checks, and
    packBoxModeFlags sets bit i when activeBoxes[activeBoxIds[i]] is set
    (fc_msp_box.c:463-467). serializeBoxReply reports that same array, one
    permanentId per entry and in the same order, so index i of this reply is bit
    i of the mask -- whatever the build happens to have compiled in.

    Transcribing the list by hand instead is how a mode readout lies. On this
    build bits 0-2 are ARM / PREARM / MULTI FUNCTION and ANGLE is bit 3, but a
    hand-written table that began at ANGLE still decoded bit 0 as ANGLE -- which
    is the very same string a genuine ANGLE switch produces. Nothing about the
    output looks wrong; the only reason it was caught at all is the ARM bit
    disagreeing with armingFlags.
    """
    cmd, payload, link.buf, err = link.request(MSP_BOXIDS, timeout=timeout)
    if cmd != MSP_BOXIDS or err or not payload:
        return None
    return list(payload)


def describe_modes(ids, boxflags, arming):
    """The active flight modes, or None if either read failed.

    MSP_STATUS carries only the low 32 bits of a boxBitmask_t that is wider than
    that (sbufWriteData(dst, &mspBoxModeFlags, 4), fc_msp.c:487 -- the mask is
    8 bytes, BITARRAY_DECLARE over CHECKBOX_ITEM_COUNT = 60, so bits 32-59 are
    never sent; MSP2_INAV_STATUS and MSP_ACTIVEBOXES do send all 8). Ids past
    position 31 are therefore dropped rather than guessed at.

    The ARM cross-check earns the call it costs: BOXARM is the one mode whose
    state MSP2_INAV_STATUS reports separately, and a wrong bit order fails
    silently in exactly the way described above. A disagreement means the ids
    and the mask did not come from the same instant, or the id-to-name map is
    wrong -- and either way the mode string must not be believed.
    """
    if ids is None or boxflags is None:
        return None
    if arming is not None and BOX_ID_ARM in ids:
        arm_bit = ids.index(BOX_ID_ARM)
        if arm_bit < 32 and bool(boxflags & (1 << arm_bit)) != bool(arming & ARMED):
            return "MASK MISMATCH (ids and mask disagree on ARM)"
    names = [BOX_NAMES_BY_ID.get(box_id, "BOX%d" % box_id)
             for i, box_id in enumerate(ids)
             if i < 32 and box_id in NOTABLE_BOX_IDS and boxflags & (1 << i)]
    return "|".join(names) if names else "none"


def levels_the_loop(ids, boxflags):
    """True if the level controller runs, False if it does not, None if unknown.

    fc_core.c:692-699 gives ANGLE priority over HORIZON inside a block guarded by
    autoEnableAngle, which is `failsafeRequiresAngleMode() ||
    navigationRequiresAngleMode() || emergRearmAngleEnforce` (fc_core.c:684) --
    none of which this can read. So True is solid, and False means specifically
    that no ANGLE/HORIZON box is active: that is the same as "no level loop" only
    while autoEnableAngle is clear, since a failsafe can force ANGLE_MODE on with
    no box at all. MANUAL (ACRO) suppresses both, again subject to
    autoEnableAngle, which is why ANGLE-plus-MANUAL answers unknown rather than
    guessing either way.

    Note the gate itself is not multirotor-specific -- pid.c:1281 is a flat four
    way OR and ANGLE/HORIZON exist on every platform with an accelerometer. What
    is multirotor-specific is the route list: ANGLEHOLD_MODE needs
    STATE(AIRPLANE) and a flight-axis angle override needs a programmed logic
    condition, so on this airframe those two cannot open it.
    """
    if ids is None or boxflags is None:
        return None
    active = {box_id for i, box_id in enumerate(ids)
              if i < 32 and boxflags & (1 << i)}
    if not active.intersection(LEVEL_LOOP_IDS):
        return False
    if BOX_ID_MANUAL in active:
        return None
    return True


def read_altitude_cm(link, timeout=0.2):
    """Estimated altitude in cm, or None. Signed: below the start reads negative."""
    cmd, payload, link.buf, err = link.request(MSP_ALTITUDE, timeout=timeout)
    if cmd != MSP_ALTITUDE or err or len(payload) < 4:
        return None
    return struct.unpack_from("<i", payload, 0)[0]


def read_altitude_vel_cm(link, timeout=0.2):
    """(estimated altitude cm, estimated vertical velocity cm/s), or None.

    The same reply as read_altitude_cm with its second word kept:
    lrintf(getEstimatedActualVelocity(Z)) (fc_msp.c:688), written through
    sbufWriteU16 (streambuf.h:34, uint16_t) so a descent arrives as two's
    complement and has to be unpacked signed.

    Worth reading rather than differentiating the position. Field 1 is a
    position *estimate*: whole centimetres, whatever latency the estimator and
    the round trip added, and no timestamp saying which instant it describes.
    Its derivative therefore mixes estimator noise with this client's read
    jitter, and on a steady climb that comes out as a stutter that is not there.
    """
    cmd, payload, link.buf, err = link.request(MSP_ALTITUDE, timeout=timeout)
    if cmd != MSP_ALTITUDE or err or len(payload) < 6:
        return None
    return struct.unpack_from("<ih", payload, 0)


def read_pid_table(link, timeout=1.0):
    """The PID table as a list of mutable [P, I, D, FF] lists, or None.

    Every item comes back as four bytes whatever the bank's real scale
    (fc_msp.c:795-802), and MSP2_SET_PID reads them back the same way, so a
    read-modify-write round trip is lossless.
    """
    cmd, payload, link.buf, err = link.request(MSP2_PID, timeout=timeout)
    if cmd != MSP2_PID or err or len(payload) < PID_ITEM_COUNT * 4:
        return None
    return [list(payload[i * 4:(i + 1) * 4]) for i in range(PID_ITEM_COUNT)]


def write_pid_table(link, table, timeout=0.5):
    """Write a whole PID table back, and say nothing about whether it worked.

    MSP2_SET_PID's success path writes no reply body at all (fc_msp.c:2032-2043
    just breaks), so whether an ACK comes back is up to the MSP layer rather than
    to this command, and waiting on one would turn a successful write into a
    timeout. The caller confirms with read_pid_table instead, which is the
    stronger check anyway: it proves the bank holds the new numbers rather than
    that something said so.
    """
    payload = bytes(b for item in table for b in item)
    link.request(MSP2_SET_PID, payload, timeout=timeout)


def apply_level_p_scale(link, scale):
    """Scale the level controller's P gain. Returns (new [P,I,D,FF] or None, note).

    This is the experiment that tells the level loop apart from the rate loop.
    pid.c:1280-1284 runs the level controller over the roll and pitch axes only,
    and only under ANGLE, HORIZON or ANGLEHOLD; the rate controller runs
    underneath it on every axis either way. Setting this gain to zero therefore
    removes exactly one of the two and leaves the other, and which one comes
    back is the answer. It is also the only lever available here: reaching the
    CLI would mean sending '#' into a live session, which latches
    ARMING_DISABLED_CLI until the container is restarted.

    Called from the setup path, never from the flight loop. The write can block
    for its whole timeout waiting on a reply that may never arrive, and a second
    of silence on this link is a receiver timeout five times over.

    Nothing is saved to the eeprom, so the next reboot undoes it. The caller is
    expected to have just done one, which is what keeps this from outliving the
    run that asked for it.
    """
    table = read_pid_table(link)
    if table is None:
        return None, "could not read the PID table"

    before = table[PID_LEVEL][:4]
    table[PID_LEVEL][0] = int(max(0, min(255, round(before[0] * scale))))
    after = table[PID_LEVEL][:4]

    if after == before:
        return after, "already at that value"

    write_pid_table(link, table)

    # The read-back is the only evidence there is. See write_pid_table.
    check = read_pid_table(link)
    if check is None:
        return None, "wrote it, but could not read the table back to confirm"
    if check[PID_LEVEL][:4] != after:
        return check[PID_LEVEL][:4], (
            "write did NOT take: asked for %r, bank holds %r"
            % (after, check[PID_LEVEL][:4]))
    return after, None


def parse_pid_spec(text):
    """'ROLL:40,30,0,60' -> ('ROLL', [40, 30, 0, 60]). Raises ValueError.

    The axis is named rather than numbered on purpose: the bank order is a
    firmware detail (pid.h:59-71) and the names are what the terminal output
    above it already uses, so a mistyped index would silently retune the wrong
    loop -- POS_Z for ROLL, say -- and the run would come back looking like a
    result.
    """
    if ":" not in text:
        raise ValueError("expected AXIS:P,I,D,FF, got %r" % text)
    name, _, rest = text.partition(":")
    name = name.strip().upper()
    if name not in PID_NAMES:
        raise ValueError("unknown axis %r; known: %s" % (name, ", ".join(PID_NAMES)))
    fields = [f.strip() for f in rest.split(",")]
    if len(fields) != 4:
        raise ValueError("expected four gains after %s:, got %d" % (name, len(fields)))
    values = []
    for f in fields:
        value = int(f)
        if not 0 <= value <= 255:
            raise ValueError("%s: %d is outside the 0..255 the table stores" % (name, value))
        values.append(value)
    return name, values


def apply_pid_setting(link, specs):
    """Write gains into the live PID table. Returns (changes, note).

    changes is a list of (name, before, after) for every bank that moved. Same
    read-modify-write and same read-back confirmation as apply_level_p_scale,
    and for the same reason: the write has no acknowledgement of its own, so a
    table read afterwards is the only proof it landed.

    Nothing is saved. The caller is expected to have just rebooted, which is
    what keeps a run's tuning from outliving the run.
    """
    table = read_pid_table(link)
    if table is None:
        return None, "could not read the PID table"

    changes = []
    for name, values in specs:
        index = PID_NAMES.index(name)
        before = table[index][:4]
        table[index][:4] = values
        if before != values:
            changes.append((name, before, list(values)))

    if not changes:
        return [], "already at those values"

    write_pid_table(link, table)

    check = read_pid_table(link)
    if check is None:
        return None, "wrote it, but could not read the table back to confirm"
    for name, _before, after in changes:
        index = PID_NAMES.index(name)
        if check[index][:4] != after:
            return changes, ("write did NOT take: %s asked for %r, bank holds %r"
                             % (name, after, check[index][:4]))
    return changes, None


def read_rc_tuning(link, timeout=1.0):
    """The live profile's rate/TPA block as 11 raw bytes, or None.

    Byte 0 is rcRate8, which the firmware writes as a constant 100 and ignores
    on the way back in (fc_msp.c:761, :2088). It is carried through verbatim
    rather than reasoned about, which is also why the block is handed around as
    bytes instead of a parsed struct -- every field except the two this changes
    is somebody else's to own.
    """
    cmd, payload, link.buf, err = link.request(MSP_RC_TUNING, timeout=timeout)
    if cmd != MSP_RC_TUNING or err or len(payload) < 11:
        return None
    return bytearray(payload[:11])


def write_rc_tuning(link, block, timeout=0.5):
    """Write the block back. See write_pid_table on why there is no ACK check."""
    link.request(MSP_SET_RC_TUNING, bytes(block), timeout=timeout)


def tpa_of(block):
    """(tpa_rate, tpa_breakpoint) out of a block."""
    return (block[RC_TUNING_TPA_RATE],
            block[RC_TUNING_TPA_BREAKPOINT][0]
            | (block[RC_TUNING_TPA_BREAKPOINT][1] << 8))


def describe_rc_tuning(block):
    """One line naming every field this touches, in the units the firmware uses.

    rates are deca-degrees per second (settings.yaml:1433), so they are printed
    both ways: the raw byte is what a write has to send, and the dps figure is
    the maximum rate the pilot can command on that axis at all.
    """
    r = block[RC_TUNING_RATES]
    rate, breakpoint = tpa_of(block)
    return ("rates R/P/Y %d/%d/%d (%d/%d/%d dps max), "
            "tpa_rate %d/200, tpa_breakpoint %d"
            % (r[0], r[1], r[2], r[0] * 10, r[1] * 10, r[2] * 10,
               rate, breakpoint))


def apply_tpa(link, rate=None, breakpoint=None):
    """Set throttle PID attenuation. Returns (new block or None, note).

    TPA is a loop-gain knob that is a function of throttle and of nothing else.
    It is the one lever here that can change how hard the loop pushes without
    touching an airframe constant, and on this airframe that distinction is the
    whole point: the plant is thrust-vectored with no rotational damping, so its
    gain rises with throttle, while the shipped profile has tpa_rate = 0 --
    nothing attenuates anything.

    What it scales, read rather than assumed: on multirotor the factor is
    applied to kP, kD and kCD (the FF term that multiplies D) and NOT to kI
    (pid.c:592-597 -- iTermFactor is only used on the PID_TYPE_PIFF branch).
    Yaw is exempt unless dynPID_on_YAW is set, and it is 0 here.

    At full stick the arithmetic is exact and has no intermediate terms. The
    throttle TPA works from is rcCommand[THROTTLE], unfiltered on multirotor
    (pid.c:524), which at full stick is MAX_THROTTLE = 2000 (mixer.c:60), so
    `throttle < getMaxThrottle()` is false and the factor is the else branch:
    (100 - min(tpa_rate, 100)) / 100 (pid.c:506). So `--throttle 1.0 --tpa 50`
    means "run the roll and pitch rate loop at exactly half gain", and nothing
    else about the aircraft changes.

    Nothing is saved to the eeprom, so the next reboot undoes it. The caller is
    expected to have just done one, which is what keeps this from outliving the
    run that asked for it.

    Called from the setup path, never from the flight loop, for the same reason
    apply_level_p_scale is: the write can block for its whole timeout.
    """
    block = read_rc_tuning(link)
    if block is None:
        return None, "could not read the rate/TPA block"

    before = bytes(block)
    if rate is not None:
        block[RC_TUNING_TPA_RATE] = max(0, min(TPA_RATE_MAX, int(rate)))
    if breakpoint is not None:
        bp = max(TPA_BREAKPOINT_MIN, min(TPA_BREAKPOINT_MAX, int(breakpoint)))
        block[RC_TUNING_TPA_BREAKPOINT] = bytes([bp & 0xFF, (bp >> 8) & 0xFF])

    if bytes(block) == before:
        return before, "already at that value"

    write_rc_tuning(link, block)

    # The read-back is the only evidence there is. See write_pid_table.
    check = read_rc_tuning(link)
    if check is None:
        return None, "wrote it, but could not read the block back to confirm"
    if bytes(check) != bytes(block):
        return bytes(check), (
            "write did NOT take: asked for %r, the profile holds %r"
            % (bytes(block), bytes(check)))
    return bytes(check), None


def dump_pid_table(link):
    """Print the live PID table. Returns True if it read.

    Printed because the rate-loop gains are the other half of any phase-margin
    argument about this airframe, and reading them is the only way to have them
    as numbers rather than as the struct's factory defaults.
    """
    table = read_pid_table(link)
    if table is None:
        print("error: could not read the PID table", file=sys.stderr)
        return False
    for i, item in enumerate(table):
        name = PID_NAMES[i] if i < len(PID_NAMES) else "?"
        print("  PID %-12s P=%-5d I=%-5d D=%-5d FF=%-5d"
              % ((name,) + tuple(item[:4])))
    return True


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
    ap.add_argument("--monitor-hz", type=float, default=DEFAULT_MONITOR_HZ,
                    help="Hz to sample the body rates and the flight loop time "
                         "during the flight, summed up once a second; 0 turns it "
                         "off (default %g)" % DEFAULT_MONITOR_HZ)
    ap.add_argument("--record", default=None, metavar="FILE",
                    help="write every monitor sample to FILE as CSV: time since "
                         "start, body rates, body accel in G, attitude in degrees, "
                         "then the eight motor outputs. Costs two extra reads per "
                         "sample, and the printed summary cannot show either of "
                         "the last two.")
    ap.add_argument("--pid", action="append", default=None, metavar="AXIS:P,I,D,FF",
                    help="set a gain bank in the live PID table, e.g. "
                         "--pid ROLL:40,30,0,60. Repeatable, and applied before "
                         "--level-p-scale so the two compose. Never saved.")
    ap.add_argument("--level-p-scale", type=float, default=1.0,
                    help="multiply the ANGLE/HORIZON level controller's P gain by "
                         "this before flying. 0 removes the level loop and leaves "
                         "the rate loop running underneath it, which is what tells "
                         "the two apart. Applied after --reboot and never saved, so "
                         "a reboot undoes it (default 1.0 = leave it alone)")
    ap.add_argument("--tpa", type=int, metavar="RATE",
                    help="set throttle PID attenuation (tpa_rate, 0..%d) on the "
                         "live control profile before flying. TPA scales the rate "
                         "loop's P and D (not I) against throttle, so it is a "
                         "gain knob that changes no airframe constant. At "
                         "--throttle 1 that factor is exactly (100-RATE)/100, so "
                         "--tpa 50 is precisely half gain. Applied after --reboot "
                         "and never saved, so a reboot undoes it (default: leave "
                         "the stored value, which is 0 -- i.e. off -- in the "
                         "shipped eeprom)"
                         % TPA_RATE_MAX)
    ap.add_argument("--tpa-breakpoint", type=int, metavar="PWM",
                    help="set tpa_breakpoint in RC units, %d..%d; TPA is off below "
                         "it. Only meaningful with --tpa, but read-modify-write, so "
                         "passing it alone leaves tpa_rate untouched"
                         % (TPA_BREAKPOINT_MIN, TPA_BREAKPOINT_MAX))
    ap.add_argument("--dump-pids", action="store_true",
                    help="print the live PID table (all %d banks) after --reboot and "
                         "before flying, then carry on with the run"
                         % PID_ITEM_COUNT)
    ap.add_argument("--seconds", type=float, default=0,
                    help="how long to fly; 0 means until interrupted (default 0)")
    ap.add_argument("--arm", action="store_true",
                    help="raise AUX1 and hold it, to arm")
    ap.add_argument("--throttle", type=float, default=0.0,
                    help="0..1, scaled from the bottom of the stick range (default 0)")
    ap.add_argument("--roll", type=float, default=0.0, help="-1..1 (default 0)")
    ap.add_argument("--pitch", type=float, default=0.0, help="-1..1 (default 0)")
    ap.add_argument("--pitch-after", type=float, default=None, metavar="SECONDS",
                    help="apply --pitch only after this many seconds of stick "
                         "time, so the airframe is airborne and trimmed before "
                         "the command lands. Without it a pitch command is "
                         "applied from the first frame, which on a tailsitter "
                         "means commanding a tilt while it is still on its "
                         "tail -- a ground interaction, not a loop test.")
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

    if args.monitor_hz < 0:
        print("error: --monitor-hz cannot be negative; 0 turns it off.",
              file=sys.stderr)
        return 1

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

    # ---- Which flight mode, read rather than assumed ------------------------
    # pid.c:1281 gates the entire level controller on an ANGLE-family box being
    # active, so in ACRO there is no level loop and every number downstream about
    # it is meaningless. The arming flags cannot answer this question: the
    # NOT_LEVEL check at fc_core.c:248 sits in a flat list of disarmed checks
    # with no mode guard, so it fires in ACRO exactly the same.
    #
    # The ids and the mask have to come from the same instant, which is why they
    # are read back to back here rather than cached from earlier in the session.
    box_ids = read_box_ids(link)
    boxflags = read_box_flags(link)
    _, arming_now = read_status(link, timeout=0.5)
    modes = describe_modes(box_ids, boxflags, arming_now)
    if modes is None:
        print("  flight mode: ? (MSP_STATUS or MSP_BOXIDS gave nothing back)")
    elif modes.startswith("MASK MISMATCH"):
        # Loud, and in the place the mode string would have been. Everything
        # downstream keys off this line, and a mode decoded through a wrong
        # mapping reads exactly like a real one -- so it does not get a quiet
        # footnote, it gets the line.
        print("  flight mode: UNREADABLE -- %s" % modes)
        print("    the mask cannot be trusted; treat every mode below as unknown")
    else:
        print("  flight mode: %s" % modes)
        if levels_the_loop(box_ids, boxflags) is False:
            print("    note: no ANGLE/HORIZON box is active, so the level "
                  "controller is very likely NOT running and zeroing "
                  "PID_LEVEL.P would change nothing at all. The one way it can "
                  "still run is a failsafe forcing ANGLE on, which no box shows.")

    # ---- Gains written straight into the table, if asked for ----------------
    if args.pid:
        try:
            specs = [parse_pid_spec(text) for text in args.pid]
        except ValueError as err:
            print("error: --pid: %s" % err, file=sys.stderr)
            link.close()
            return 10
        changes, problem = apply_pid_setting(link, specs)
        if changes is None:
            print("error: --pid: %s" % problem, file=sys.stderr)
            link.close()
            return 10
        for name, before, after in changes:
            print("  pid %-12s: P=%d I=%d D=%d FF=%d -> P=%d I=%d D=%d FF=%d"
                  % ((name,) + tuple(before) + tuple(after)))
        if problem is not None:
            print("    %s" % problem)
        if changes:
            print("    not saved: the next reboot restores the stored value.")

    # ---- The level-P experiment, if asked for -------------------------------
    if args.level_p_scale != 1.0:
        after, problem = apply_level_p_scale(link, args.level_p_scale)
        if after is None:
            print("error: --level-p-scale: %s" % problem, file=sys.stderr)
            link.close()
            return 6
        print("  level P: PID_LEVEL scaled x%g -> P=%d I=%d D=%d FF=%d"
              % (args.level_p_scale, after[0], after[1], after[2], after[3]))
        if problem is not None:
            print("    %s" % problem)
        print("    not saved: the next reboot restores the stored value.")

    # ---- The PID table, if asked for ----------------------------------------
    if args.dump_pids:
        if not dump_pid_table(link):
            link.close()
            return 7

    # ---- The TPA experiment, if asked for -----------------------------------
    if args.tpa is not None or args.tpa_breakpoint is not None:
        after, problem = apply_tpa(link, args.tpa, args.tpa_breakpoint)
        if after is None:
            print("error: --tpa: %s" % problem, file=sys.stderr)
            link.close()
            return 8
        print("  tpa: %s" % describe_rc_tuning(after))
        if problem is not None:
            print("    %s" % problem)
        print("    not saved: the next reboot restores the stored value.")

    # Built after the --reboot prologue on purpose: that prologue holds the link
    # to itself while it waits for the reconnect, and a monitor sampling into it
    # would be reading a socket that is deliberately going away.
    recorder = None
    if args.record:
        if args.monitor_hz <= 0:
            print("error: --record samples the monitor's schedule, and "
                  "--monitor-hz 0 removes it. Pick a rate.", file=sys.stderr)
            link.close()
            return 9
        try:
            recorder = CsvRecorder(args.record)
        except OSError as err:
            print("error: --record: cannot write %s: %s" % (args.record, err),
                  file=sys.stderr)
            link.close()
            return 9

    monitor = (LoopMonitor(link, args.monitor_hz, recorder)
               if args.monitor_hz > 0 else None)

    # Throttle is one-sided: 0 is the bottom of the range, not the centre.
    throttle = int(round(RC_MIN + args.throttle * (RC_MAX - RC_MIN)))

    channels = [RC_MID] * 8
    channels[ROLL] = stick(args.roll)
    channels[PITCH] = stick(args.pitch)
    channels[THROTTLE] = throttle
    channels[YAW] = stick(args.yaw)
    channels[4] = RC_MAX if args.arm else RC_MIN

    # A staged pitch command is the same frame with the pitch stick moved, so
    # it is held here and swapped in by the loop rather than rebuilt per frame.
    # Taken before the settle/arm frames are copied below, so those keep the
    # centred pitch -- the command must not start moving the airframe during
    # the arm window.
    late_channels = None
    if args.pitch_after is not None:
        late_channels = list(channels)
        channels[PITCH] = RC_MID

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
    if monitor is not None:
        print("monitoring body rates and loop time at %.0f Hz, summed up once a "
              "second." % args.monitor_hz)

    # Taken here, after the --reboot prologue, so --seconds measures stick time
    # and a long revive does not eat the flight window.
    started = repeater.restart()
    if monitor is not None:
        # Same origin as the per-second prints below, so a row in the CSV and a
        # line in the log can be laid against each other without arithmetic.
        monitor.clock = started
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
                # Measured from the same origin --seconds uses, so the two
                # describe one timeline.
                if (late_channels is not None
                        and (time.time() - started) >= args.pitch_after):
                    frame = late_channels
            repeater.send(frame)

            # Sampled between sends, on the monitor's own schedule, so a slow
            # reply eats into the wait rather than into the stick stream.
            now = time.time()
            if monitor is not None and monitor.due(now):
                monitor.sample(now)

            if repeater.sent % int(args.rate) == 0:
                flags = None
                if monitor is not None:
                    rates, attitudes, cycles, flags, missed, span, fresh = \
                        monitor.take(now)
                    asked = len(rates) + missed
                    print(format_loop_window(now - started, span, rates, cycles,
                                             flags, missed, asked, fresh))
                    print(format_attitude_window(now - started, span, attitudes))
                seen = read_channels(link)
                if seen is not None:
                    # Deliberately shown in the firmware's own order. It will
                    # not match what was sent above -- see the note on AETR.
                    print("  %5.1fs  fc sees: %s" % (now - started, seen[:6]))

                # Altitude, because "it sways" and "it sways while pinned to the
                # ground" are different faults with different causes, and nothing
                # else in this output separates them. A ground contact is a
                # nonlinear constraint that will hold a saturated loop in a limit
                # cycle on its own, roll and pitch especially, and the airframe
                # leaning on something is exactly where that would show up.
                altitude = read_altitude_cm(link, timeout=0.05)
                if altitude is None:
                    print("  %5.1fs  altitude ?" % (now - started))
                else:
                    print("  %5.1fs  altitude %+.2f m%s"
                          % (now - started, altitude / 100.0,
                             "   <- still at ground level"
                             if abs(altitude) < 30 else ""))

                # Short timeout on both of these on purpose. Each one costs what
                # it costs only when the firmware is not answering, and three
                # quarter-second stalls in a row would cross the 200 ms receiver
                # timeout and drop the link -- which would look exactly like the
                # fault being investigated.
                # The ids are not re-read: initActiveBoxIds runs once at boot off
                # feature and sensor state, so that list cannot change without a
                # reboot. The mask is the half that moves, and re-reading it is
                # what makes a switch flip visible at all.
                now_box = read_box_flags(link, timeout=0.05)
                if now_box is not None and now_box != boxflags:
                    boxflags = now_box
                    # flags is the arming flags off the last monitor sample, so
                    # the cross-check runs in flight too and not only at connect.
                    # A mode switch is precisely when a stale mapping would start
                    # producing confident nonsense, which is the failure this is
                    # here to catch.
                    print("  %5.1fs  flight mode now: %s"
                          % (now - started,
                             describe_modes(box_ids, now_box, flags)))
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
        if recorder is not None:
            recorder.close()
        print("sent %d packets, sticks released." % repeater.sent)

    return 0


if __name__ == "__main__":
    sys.exit(main())
