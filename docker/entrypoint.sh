#!/bin/bash
#
# Entry point for the INAV SITL runtime image.
#
# Environment:
#   SIM         simulator backend to connect to (rf | xp | ue). Unset means
#               configurator-only mode: the SITL just listens on 5760.
#               "msp" is accepted as an alias for "ue" -- see RECEIVER below.
#   SIM_IP      simulator host address. Required whenever SIM is set, except
#               on WSL2 where the default gateway *is* the Windows host and
#               is used as a fallback. See the guard below.
#   SIM_PORT    simulator host port (49000 for the X-Plane/UE UDP protocol).
#   RECEIVER    where the RC sticks come from: sim (default) or msp. It picks
#               which of the two baked-in eeproms to seed from and does
#               nothing else -- the simulator link is identical either way.
#
# SIM and RECEIVER answer different questions and both are needed. SIM picks
# the backend, i.e. which datarefs are subscribed and where the sensors and
# motor outputs go. RECEIVER picks who reads the sticks. They are independent:
# with SIM=ue the firmware still receives inav_ue/rc/channel[*] and still hands
# it to the SIM receiver -- but that receiver only exists when receiver_type is
# SIM (rx.c:299). Set receiver_type to MSP and the datarefs keep arriving,
# still get converted, and are written to a buffer nothing reads.
#   EEPROM_PATH where the live eeprom lives (default /data/eeprom.bin).
#   EEPROM_TEMPLATE  explicit eeprom to seed from. Wins over RECEIVER, and is
#               only needed for a template that is not one of the two in the
#               image.
#   RESEED      1 to overwrite an existing eeprom from the template.
#   ALLOW_GATEWAY_FALLBACK  auto (default) uses the default gateway as SIM_IP
#               only on a WSL host; 1 forces it on, 0 forces it off.
#
# Any further arguments are passed through to the SITL verbatim, so
#   docker run ... inav-sitl:9.1 --sim=ue --chanmap=M01-01,...
# works as well.
#
# Exit codes are a contract the launcher decodes:
#   1  no SITL binary in the image
#   2  SIM_IP missing or unusable, and this host cannot infer it
#   3  no usable eeprom: either nowhere writable to put it, or a RECEIVER
#      value that names no template
#
# All three are fast and loud on purpose. The SITL's own behaviour in the
# latter two cases is a container that reports Up, logs nothing alarming,
# and never binds a port.
#
# Every diagnostic below goes to stderr, which is not just style: stdout is
# block-buffered when the container's log is a pipe, so a message left there
# surfaces *after* the stderr lines that explain it and the log reads
# back-to-front.

set -e

BIN=$(find /opt/inav -maxdepth 1 -name '*_SITL' -type f | head -n 1)
if [ -z "$BIN" ]; then
    echo "entrypoint: no *_SITL binary found in /opt/inav" >&2
    exit 1
fi

# --- eeprom ------------------------------------------------------------------
#
# The firmware opens the eeprom with fopen(path, "r+") -- it needs write
# access to the *file*, not just the directory. There is no graceful failure
# if it cannot, and the two ways it breaks look nothing alike:
#
#   unopenable (no such file and the directory is unwritable, or the file
#   itself is read-only) -> config_streamer_file.c notes the failed "w+" but
#   does not act on it, then calls fwrite() with the NULL FILE* anyway. That
#   is a SIGSEGV: no ports bound, container exits 139.
#
#   openable but unwritable (ENOSPC, I/O error) -> streamerLocked stays set,
#   writeEEPROM() fails, config_eeprom.c:353 calls
#   failureMode(FLASH_WRITE_FAILED), which on SITL is `while (true) delay(1000)`.
#   That one binds no ports either, but the container still reports Up.
#
# So: find somewhere writable, or refuse to start.

# --- receiver ----------------------------------------------------------------
#
# Which of the two eeproms to seed from. They differ in exactly one field,
# receiver_type, and that field is the whole question of where the sticks come
# from: the simulator's datarefs, or an MSP client on the TCP port. Everything
# else -- the physics, the sensors, the dataref subscription -- is identical.
#
# RECEIVER is the switch that names the concept: where do the sticks come
# from. SIM is the switch that names the backend, and its values are rf|xp|ue.
# They are orthogonal -- SIM=xp RECEIVER=msp is a perfectly sensible pairing,
# and is not something a single variable could express.
#
# "SIM=msp" is accepted anyway, as an alias, because it is the thing an
# operator reaches for. It is an alias and not a fourth backend: target.c
# accepts only rf|xp|ue for --sim and answers anything else with
# "[SIM] Unsupported simulator msp." followed by sitlSim = SITL_SIM_NONE -- a
# flight controller with no simulator link at all, which is the exact opposite
# of what someone asking for MSP sticks wants. So "msp" is rewritten to "ue"
# here and never reaches --sim.
#
# EEPROM_TEMPLATE, when set, wins over both: an operator naming a file has said
# something more specific than a mode name.

RECEIVER=${RECEIVER:-}
case "${SIM:-}" in
    msp|MSP)
        SIM=ue
        if [ -z "$RECEIVER" ]; then
            RECEIVER=msp
        elif [ "$RECEIVER" != msp ]; then
            # Only a genuine disagreement is worth a warning. SIM=msp with
            # RECEIVER=msp says the same thing twice and is not a conflict --
            # warning there would train the operator to ignore this line.
            echo "entrypoint: WARNING SIM=msp means 'ue backend, MSP receiver', but" >&2
            echo "entrypoint: WARNING RECEIVER=$RECEIVER says the sticks come from the" >&2
            echo "entrypoint: WARNING simulator. Taking RECEIVER, which is the explicit one." >&2
        fi
        ;;
esac

# The image bakes these in; an image built without them is a packaging mistake,
# and the symptom would otherwise be a silent fall back to the factory eeprom.
if [ -z "${EEPROM_TEMPLATE:-}" ]; then
    case "$RECEIVER" in
        "")    EEPROM_TEMPLATE=/opt/inav/eeprom.default.bin ;;
        sim)   EEPROM_TEMPLATE=/opt/inav/eeprom.default.bin ;;
        msp)   EEPROM_TEMPLATE=/opt/inav/eeprom.msp.bin ;;
        *)
            echo "entrypoint: RECEIVER must be 'sim' or 'msp', got '$RECEIVER'." >&2
            echo "entrypoint: sim: RC from the simulator's datarefs (the default)." >&2
            echo "entrypoint: msp: RC from an MSP client on the TCP port; the simulator" >&2
            echo "entrypoint:      still supplies the physics and the sensors." >&2
            exit 3
            ;;
    esac
fi

# sizeof(eepromData) in a SITL build.
EEPROM_SIZE=32768
EEPROM_PATH="${EEPROM_PATH:-/data/eeprom.bin}"

needs_seed() {
    [ -f "$1" ] || return 0
    # A wrong-sized file is worse than a missing one. config_streamer_file.c
    # accepts the load whenever the bytes read equal the file size, so an
    # 8 KiB file logs "Loaded (8192 of 32768 bytes)" and passes -- leaving
    # the rest of the buffer zeroed. Half a configuration is the worst kind.
    [ "$(stat -c%s "$1" 2>/dev/null)" = "$EEPROM_SIZE" ] || return 0
    return 1
}

if [ "${RESEED:-0}" = "1" ] || needs_seed "$EEPROM_PATH"; then
    if [ ! -f "$EEPROM_TEMPLATE" ]; then
        echo "entrypoint: eeprom template '$EEPROM_TEMPLATE' is missing." >&2
        echo "entrypoint: refusing to boot on a factory eeprom -- its receiver_type" >&2
        echo "entrypoint: is NONE, so the aircraft could never arm." >&2
        exit 3
    fi
    # The chmod is not belt-and-braces. cp creates the destination with the
    # *source's* mode bits, and the template is deliberately 0444 -- so a bare
    # cp yields a read-only eeprom, and the firmware dies on it before binding
    # a port. Both callers go through here so neither can forget.
    seed_from_template() {
        cp "$EEPROM_TEMPLATE" "$1" 2>/dev/null || return 1
        chmod u+w "$1" 2>/dev/null || return 1
        return 0
    }

    if seed_from_template "$EEPROM_PATH"; then
        echo "entrypoint: seeded $EEPROM_PATH from $EEPROM_TEMPLATE" >&2
    else
        # Root-owned bind mount, a :ro mount, or an SELinux label. An eeprom
        # that works but does not outlive the container beats one the firmware
        # segfaults on, so fall back and say so loudly.
        FALLBACK=/tmp/eeprom.bin
        if seed_from_template "$FALLBACK"; then
            echo "entrypoint: WARNING $EEPROM_PATH is not writable by uid $(id -u)" >&2
            echo "entrypoint: WARNING using $FALLBACK instead -- eeprom changes will" >&2
            echo "entrypoint: WARNING not survive this container." >&2
            EEPROM_PATH="$FALLBACK"
        else
            echo "entrypoint: no writable eeprom location (tried $EEPROM_PATH, then $FALLBACK)." >&2
            echo "entrypoint: a read-only eeprom is not a degraded mode -- the firmware" >&2
            echo "entrypoint: dereferences the failed fopen and dies on SIGSEGV, having" >&2
            echo "entrypoint: bound no ports. Give uid $(id -u) a writable directory." >&2
            exit 3
        fi
    fi
fi

args=("--path=${EEPROM_PATH}")

# --- simulator ---------------------------------------------------------------

if [ -n "${SIM:-}" ]; then
    # The default gateway is the Windows host ONLY under WSL2 NAT. Anywhere
    # else it is a router, and --simip=<router> is a valid unicast address
    # that simply nothing answers: every subscription is black-holed and the
    # only symptom is a handshake timing out ten seconds later. Containers
    # share the host kernel, so /proc/version answers the question that
    # actually matters -- is the *docker host* WSL.
    is_wsl() {
        grep -qi microsoft /proc/version 2>/dev/null
    }

    # Tri-state so an explicit setting always wins over the detection. Both
    # directions are real: 1 for a host whose gateway genuinely is the
    # simulator, 0 for WSL2 *mirrored* networking, where loopback is shared
    # with Windows and the gateway is a router like anywhere else.
    use_gateway_fallback() {
        case "${ALLOW_GATEWAY_FALLBACK:-auto}" in
            1|yes|true)  return 0 ;;
            0|no|false)  return 1 ;;
            *)           is_wsl ;;
        esac
    }

    if [ -z "${SIM_IP:-}" ] && use_gateway_fallback; then
        SIM_IP=$(ip route show default 2>/dev/null | awk '{print $3; exit}')
        if [ -n "$SIM_IP" ]; then
            echo "entrypoint: SIM_IP unset; falling back to the default gateway $SIM_IP." >&2
            echo "entrypoint: this is the Windows host only under --network host on a" >&2
            echo "entrypoint: WSL2 NAT host. Under the default bridge the gateway is" >&2
            echo "entrypoint: the docker bridge, which reaches this VM and not the" >&2
            echo "entrypoint: simulator -- the packets just go unanswered." >&2
        fi
    fi

    if [ -z "${SIM_IP:-}" ]; then
        echo "entrypoint: SIM_IP is required when SIM is set on a non-WSL host." >&2
        echo "entrypoint: point it at the machine running the simulator, e.g." >&2
        echo "entrypoint:   docker run ... -e SIM_IP=192.168.1.50 ..." >&2
        exit 2
    fi

    case "$SIM_IP" in
        0.0.0.0|255.255.255.255)
            echo "entrypoint: SIM_IP=$SIM_IP is not a host address." >&2
            exit 2
            ;;
        127.*)
            echo "entrypoint: WARNING SIM_IP=$SIM_IP is loopback -- only correct if" >&2
            echo "entrypoint: WARNING the simulator shares this network namespace." >&2
            ;;
    esac

    # Never omitted. Leaving --simip out entirely makes the SITL fall back to
    # 127.0.0.1 and talk to itself.
    args+=("--sim=${SIM}" "--simip=${SIM_IP}")

    if [ -n "${SIM_PORT:-}" ]; then
        args+=("--simport=${SIM_PORT}")
    fi
    echo "entrypoint: ${BIN} ${args[*]} (simulator: ${SIM} @ ${SIM_IP}:${SIM_PORT:-default})" >&2
else
    echo "entrypoint: ${BIN} ${args[*]} (configurator-only)" >&2
fi

exec "${BIN}" "${args[@]}" "$@"
