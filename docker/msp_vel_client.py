#!/usr/bin/env python3
"""Drive a SITL container with the velocity loop -- MSP2_INAV_SET_VELOCITY (0x203D).

Why the sticks in msp_rc_client.py do nothing here: this firmware's
taskVelocityControl() (velocity_control.c:296) takes over the throttle and the
attitude channels the moment the aircraft arms, and never hands them back, not
even when the command stream stops (the comment says why: letting go would
return rcCommand to neutral, which is idle throttle, which is a free fall).
Measured: with --throttle 1 (2000us) the FC really does read back 2000 on the
channel, but the mixer is fed hover throttle, and the aircraft sits on the
ground at full stick.

So the only input it listens to is 0x203D: three float32, cm/s, H frame --
x forward, y right, z up (fc_msp.c:2104). This client streams that instead,
with the stick frames kept alive only to hold AUX1 high and to keep the MSP
receiver from timing out (RX_TIMEOUT_MS = 200).

The transport, the diagnostics and the whole --reboot evidence chain are
imported from msp_rc_client.py rather than reimplemented; only the flight
itself is different. Anything about the wire format belongs over there.

  # climb at 1.5 m/s for six seconds, then hover in place for 20
  python3 msp_vel_client.py --port 5760 --reboot --arm --climb-seconds 6 --hold 20

  # fly forward at 1.5 m/s for ten seconds (no climb phase)
  python3 msp_vel_client.py --arm --climb-seconds 0 --vx 150 --hold 10

  # one constant setpoint, no phases
  python3 msp_vel_client.py --arm --vx 150 --vz 50 --seconds 10

  # the same climb, sampled fast enough to see the estimator's own velocity
  python3 msp_vel_client.py --arm --vz 1500 --seconds 20 --print-every 0.1

Three vertical rates are printed and they are not the same measurement. dz/dt
differences this client's samples of the position *estimate* across whatever
wall-clock interval elapsed between prints -- at 1 Hz that interval includes up
to four blocking MSP reads, so a steady climb reads back as a stutter, and the
dt column is the denominator it actually used. dz/dtNs is the same difference
over a longer window, which is the timescale an eye would call a stutter rather
than centimetres of estimate resolution. fcVz is the second word of the same
MSP_ALTITUDE reply (read_altitude_vel_cm), is the estimator's own velocity, and
needs no differencing at all. The skew column is how far after the top of the
loop that reply landed.

The stick frames go nowhere after arming, so a mistake in them cannot change
how the aircraft flies -- but they still have to be sent, on schedule, or the
receiver failsafes and disarms mid-air.
"""

import argparse
import struct
import sys
import time
from collections import deque

from msp_rc_client import (
    ARMING_DISABLED_ARM_SWITCH, ARMING_DISABLED_HARDWARE_FAILURE, ARM_HOLD_S,
    ARM_SETTLE_S, ARMED, MspLink,
    RC_MAX, RC_MID, RC_MIN, Repeater, SIMULATOR_MODE_SITL, THROTTLE,
    build_request, describe_flags, read_altitude_vel_cm, read_arming_flags,
    read_attitude, read_motors, read_status, reboot_and_revive, wait_for_fc,
)

MSP2_INAV_SET_VELOCITY = 0x203D

# velocity_control.c:54. The loop keeps flying the last target until this much
# silence, so the refresh rate only has to beat it -- it does not set the
# dynamics. At the default 50 Hz stick rate we refresh every third frame,
# ~17 Hz, which is 30x redundant.
VEL_TIMEOUT_S = 0.5

# AUX1 sits at channel index 4 in the 8-channel frame this client sends; it is
# driven in msp_rc_client.py to arm, and has to keep being driven once armed.
AUX1 = 4


def set_velocity(link, vx, vy, vz):
    """Velocity target in cm/s, H frame: x forward / y right / z up."""
    link.sock.sendall(build_request(MSP2_INAV_SET_VELOCITY,
                                    struct.pack("<3f", vx, vy, vz)))


def arm(link, rep, sticks_down, timeout):
    """Hold AUX1 low, then high, and poll until the FC says ARMED.

    Counting seconds instead of polling is what makes a run silently worthless:
    AUX1 going high is a request, not a state change, and the FC lands it about
    a second later. A phase clock started at the raise therefore spends its
    first second commanding an aircraft that is still on the ground -- start a
    climb window too early and the whole flight is missed while the telemetry
    looks like a perfect hover (altitude flat, dz/dt 0.000).

    Returns (flags, elapsed, last_seen, replies); flags is None on timeout.
    last_seen is the most recent frame that actually parsed, which is the one
    worth reporting -- whether the final attempt of a run comes back is luck,
    and reporting it as "unknown" hides a blocker that was read a moment ago.
    """
    armed_frame = list(sticks_down)
    armed_frame[AUX1] = RC_MAX

    until = time.time() + ARM_SETTLE_S + ARM_HOLD_S
    while time.time() < until:
        rep.send(sticks_down)

    deadline = time.time() + timeout
    flags = last = None
    replies = 0
    while time.time() < deadline:
        rep.send(armed_frame)
        flags = read_arming_flags(link, timeout=0.15)
        if flags is not None:
            last, replies = flags, replies + 1
            if flags & ARMED:
                return flags, time.time() - until, last, replies
    return None, time.time() - until, last, replies


def main():
    ap = argparse.ArgumentParser(
        description="Fly a SITL container with the velocity loop (MSP2_INAV_SET_VELOCITY).",
        epilog="Requires: Play-In-Editor already running (the firmware handshakes "
               "once, for ten seconds, and only at boot), and the container started "
               "with `--receiver msp`. Reach the port from inside WSL -- wslrelay "
               "only binds [::1].")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5760)
    ap.add_argument("--reboot", action="store_true",
                    help="reboot the FC first (clears any parameter not saved, "
                         "and re-runs the simulator handshake)")
    ap.add_argument("--arm", action="store_true", help="arm via AUX1")
    ap.add_argument("--rate", type=float, default=50.0,
                    help="stick frame rate in Hz; the velocity target rides "
                         "along every third frame (default 50)")
    ap.add_argument("--arm-timeout", type=float, default=15.0,
                    help="seconds to wait for the ARMED bit after raising AUX1")

    ap.add_argument("--climb", type=float, default=150.0,
                    help="climb rate for the first phase, cm/s (default 150)")
    ap.add_argument("--climb-seconds", type=float, default=6.0,
                    help="length of the climb phase; 0 skips it (default 6)")
    ap.add_argument("--max-alt", type=float, default=1500.0,
                    help="end the climb phase early above this altitude, cm "
                         "(default 1500 = 15 m)")

    ap.add_argument("--vx", type=float, default=0.0,
                    help="hold-phase velocity, cm/s forward (default 0)")
    ap.add_argument("--vy", type=float, default=0.0,
                    help="hold-phase velocity, cm/s right (default 0)")
    ap.add_argument("--vz", type=float, default=0.0,
                    help="hold-phase vertical velocity, cm/s (default 0 = hover)")
    ap.add_argument("--hold", type=float, default=15.0,
                    help="length of the hold phase, s (default 15)")
    ap.add_argument("--seconds", type=float, default=None,
                    help="fly one constant (--vx --vy --vz) setpoint for this "
                         "long instead of running the two phases")
    ap.add_argument("--phase", action="append", default=[], metavar="T:VX:VY:VZ",
                    help="one entry of a timeline: the velocity to hold from T "
                         "seconds on, in cm/s. Repeat for a sequence, e.g. "
                         "--phase 0:0:0:150 --phase 6:1000:0:0. Overrides "
                         "--climb/--hold; the flight ends --hold seconds after "
                         "the last entry")
    ap.add_argument("--print-every", type=float, default=1.0,
                    help="telemetry interval, s (default 1)")
    ap.add_argument("--extras-every", type=float, default=1.0,
                    help="interval for the attitude/motor/status columns, s "
                         "(default 1). Those cost three more blocking MSP "
                         "reads, and printing them at 10 Hz would slow the "
                         "stick stream down -- which is a change to the flight, "
                         "not to the measurement. The altitude row is unaffected.")
    args = ap.parse_args()

    link = MspLink(args.host, args.port)
    variant = wait_for_fc(link)
    print("connected (%s)" % variant)
    rep = Repeater(link, args.rate)

    if args.reboot:
        # reboot_and_revive wants a namespace with the timeouts on it. It is
        # reused rather than reimplemented: each step in it is the only
        # evidence for the next one, and a shortcut reports success against an
        # aircraft that never rebooted.
        shim = argparse.Namespace(
            port=args.port,
            reboot_disarm_timeout=20.0,
            reboot_drop_timeout=15.0,
            reboot_connect_timeout=30.0,
            reboot_revive_timeout=60.0,
        )
        code = reboot_and_revive(link, shim, rep, variant)
        if code is not None:
            link.close()
            return code

    sticks_down = [RC_MID] * 8
    sticks_down[THROTTLE] = RC_MIN
    sticks_down[AUX1] = RC_MIN

    if args.arm:
        flags, elapsed, last, replies = arm(link, rep, sticks_down, args.arm_timeout)
        if flags is None:
            print("error: never armed in %.0fs (%d status replies)."
                  % (elapsed, replies), file=sys.stderr)
            if last is None:
                print("       not one MSP2_INAV_STATUS reply came back, so this "
                      "is not a blocker -- the flight controller is not "
                      "answering. Look for '[STALE PEER]' in the UE log.",
                      file=sys.stderr)
            else:
                print("       last flags=0x%08x (%s)"
                      % (last, describe_flags(last)), file=sys.stderr)
                if not (last & SIMULATOR_MODE_SITL):
                    print("       SIMULATOR_MODE_SITL is clear: the FC has no "
                          "simulator link, so it cannot arm and no stick frame "
                          "will change that. ue.c subscribes once, on the way "
                          "into UE_CONNECTED, and has no path back to its "
                          "connecting state -- anything that restarts the UE "
                          "side (Play-In-Editor, a scene load) strands the "
                          "container this way. Restart the container, with "
                          "Play-In-Editor already running:\n"
                          "         docker restart inav-sitl-0", file=sys.stderr)
                elif last & ARMING_DISABLED_HARDWARE_FAILURE:
                    print("       HARDWARE_FAILURE is set alongside "
                          "SIMULATOR_MODE_SITL, so the simulator link is up but "
                          "the firmware is getting no sensors over it. That is "
                          "the stale-peer state: ue.c subscribes once, on the "
                          "way into UE_CONNECTED, and has no path back to its "
                          "connecting state, so anything that restarts the UE "
                          "side (Play-In-Editor, a scene load) leaves it "
                          "subscribed to nothing. Restart the container, with "
                          "Play-In-Editor already running:\n"
                          "         docker restart inav-sitl-0", file=sys.stderr)
                    print("       (the UE log line to confirm it names the "
                          "reason: look for '[STALE PEER]' next to "
                          "'0 datarefs, 0 Hz flight loop')", file=sys.stderr)
                elif last & ARMING_DISABLED_ARM_SWITCH:
                    print("       ARM_SWITCH is latched. It sets on any blocker "
                          "present the instant AUX1 goes high and then holds "
                          "itself, so waiting cannot clear it: stop this client "
                          "(sticks go low), then re-run.", file=sys.stderr)
                else:
                    print("       a blocker that clears by waiting "
                          "(SENSORS_CALIBRATING) is worth a retry; the rest "
                          "are not.", file=sys.stderr)
            link.close()
            return 1
        print("armed after %.1fs" % elapsed)

    if args.phase:
        # A timeline given outright, so a test that needs a sequence -- fly off
        # an edge, then reverse into what is below it -- is one process with one
        # clock instead of two processes guessing at each other's timing. The
        # last entry is flown for --hold seconds, which is what keeps --hold
        # meaning "how long to keep doing the last thing".
        plan = []
        for spec in args.phase:
            parts = spec.split(":")
            if len(parts) != 4:
                print("error: --phase wants T:VX:VY:VZ, got %r" % spec,
                      file=sys.stderr)
                link.close()
                return 2
            at, a, b, c = (float(p) for p in parts)
            plan.append((at, a, b, c, "phase %5.1fs (vx %.0f vy %.0f vz %.0f)"
                         % (at, a, b, c)))
        # The loop takes the last entry whose timestamp has passed, so the list
        # has to be in time order however it was typed.
        plan.sort(key=lambda entry: entry[0])
        if plan[0][0] > 0.0:
            plan.insert(0, (0.0, 0.0, 0.0, 0.0, "phase   0.0s (vx 0 vy 0 vz 0)"))
        duration = plan[-1][0] + args.hold
    elif args.seconds is not None:
        plan = [(0.0, args.vx, args.vy, args.vz, "constant")]
        duration = args.seconds
    else:
        plan = [(0.0, 0.0, 0.0, args.climb, "climb %.0f cm/s" % args.climb)]
        if args.hold > 0:
            plan.append((args.climb_seconds, args.vx, args.vy, args.vz,
                         "hold (vx %.0f vy %.0f vz %.0f)" % (args.vx, args.vy, args.vz)))
        duration = args.climb_seconds + args.hold

    vx = vy = vz = 0.0
    label = "idle"
    next_print = 0.0
    next_extras = 0.0
    sent_vel = -1
    prev = (None, None, None)   # (stamp at read, estimated z cm, estimated vz cm/s)
    # Five samples, ~0.6 s at --print-every 0.1: long enough that the estimate's
    # 1 cm resolution stops mattering (1 cm over 0.6 s is 2 cm/s, against 17 cm/s
    # over 0.12 s) and what is left is the trend the eye would call a stutter.
    hist = deque(maxlen=5)
    started = time.time()

    try:
        while True:
            t = time.time() - started
            if t >= duration:
                break

            for at, a, b, c, text in plan:
                if t >= at:
                    vx, vy, vz, label = a, b, c, text

            # 爬到上限就提前收工，别等窗口走完
            if label.startswith("climb") and prev[1] is not None and prev[1] >= args.max_alt:
                vz, label = 0.0, "hold (climb cut off at %.0f cm)" % args.max_alt

            rep.send(sticks_down if not args.arm else _armed_frame(sticks_down))
            if rep.sent % 3 == 0:
                set_velocity(link, vx, vy, vz)
                sent_vel += 1

            if t >= next_print:
                next_print = t + args.print_every

                # The stamp is taken the instant this reply lands, and it is
                # *that* stamp the derivative below divides by. The old code
                # stamped at the top of the loop and read the altitude four
                # blocking MSP reads later, so the denominator was a different
                # interval from the numerator's: at 15.5 m/s, 13 ms of read
                # jitter is 0.2 m/s of apparent ripple, and at a 1 Hz print the
                # aliasing makes it alternate sign every row.
                alt = read_altitude_vel_cm(link, timeout=0.05)
                at_read = time.time() - started

                extras = ""
                if t >= next_extras:
                    next_extras = t + args.extras_every
                    att = read_attitude(link, timeout=0.05)
                    mot = read_motors(link, timeout=0.05)
                    _, flags = read_status(link, timeout=0.05)
                    extras = "  att %-18s mot4 %-22s %s" % (
                        "?" if att is None else "%.1f/%.1f/%.1f" % att,
                        "?" if mot is None else "/".join(str(m) for m in mot[:4]),
                        "?" if flags is None else describe_flags(flags))

                dt_alt = 0.0
                rate = ""
                if alt is not None and prev[0] is not None:
                    dt_alt = at_read - prev[0]
                    if dt_alt > 0.0:
                        rate = "dz/dt %+.3f m/s" % (
                            (alt[0] - prev[1]) / 100.0 / dt_alt)
                if alt is not None:
                    prev = (at_read, alt[0], alt[1])
                    hist.append((at_read, alt[0]))

                window = ""
                if len(hist) == hist.maxlen:
                    span = hist[-1][0] - hist[0][0]
                    if span > 0.0:
                        window = "  dz/dt%4.1fs %+.3f m/s" % (
                            span, (hist[-1][1] - hist[0][1]) / 100.0 / span)

                print("%5.1fs  %-22s alt %-9s dt %4.0fms  %-16s fcVz %-9s "
                      "skew %4.0fms%s%s" % (
                          t, label,
                          "?" if alt is None else "%+.2fm" % (alt[0] / 100.0),
                          dt_alt * 1000.0,
                          rate,
                          "?" if alt is None else "%+.2f m/s" % (alt[1] / 100.0),
                          (at_read - t) * 1000.0,
                          extras, window))
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        # Neutral sticks low, AUX1 low, velocity target zero. The target has to
        # go to zero rather than stop being sent: the loop holds its last
        # command through VEL_TIMEOUT_S and would otherwise keep flying whatever
        # was last asked for.
        for _ in range(20):
            try:
                link.send_raw_rc(sticks_down)
                set_velocity(link, 0.0, 0.0, 0.0)
            except OSError:
                break
            time.sleep(0.02)
        link.close()
        print("released after %d packets, %d velocity targets"
              % (rep.sent, sent_vel))

    return 0


_armed_frame_cache = None


def _armed_frame(sticks_down):
    """The stick frame with AUX1 high. Cached; it never changes."""
    global _armed_frame_cache
    if _armed_frame_cache is None:
        _armed_frame_cache = list(sticks_down)
        _armed_frame_cache[AUX1] = RC_MAX
    return _armed_frame_cache


if __name__ == "__main__":
    sys.exit(main())
