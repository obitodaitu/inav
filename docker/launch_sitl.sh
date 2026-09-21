#!/usr/bin/env bash
#
# One INAV SITL container per interceptor.
#
# Run this AFTER pressing Play. The plugin prints the port table when the
# level is up:
#
#   LogInavSim: INAV Sim: 4 vehicles -- 0:udp/49000 BP_InavInterceptorPawn1 | ...
#               Start containers now.
#
# That line is the signal, and the order is not a suggestion: ue.c handshakes
# once, inside a ten second window, and never retries. A container that comes
# up before the level is listening stays deaf for the rest of its life. Running
# this script again replaces its containers, which is the cheap recovery.
#
# Usage:  ./launch_sitl.sh [count]              start VehicleIndex 0..count-1
#         ./launch_sitl.sh --index 2            start just VehicleIndex 2
#         ./launch_sitl.sh --index 0,2,3        start an explicit set
#         ./launch_sitl.sh --index 2 --simip 192.168.1.50
#         ./launch_sitl.sh --stop               stop the configured fleet
#
# VehicleIndex here is the *same number* as UInavSimBridge.VehicleIndex on the
# pawn, and the ports are derived from it: udp/(49000 + index) for the sim
# link, tcp/(5760 + 10*index) for MSP. Nothing has to be converted, which is
# the point: that one number decides which aircraft a container talks to, and
# an off-by-one puts a container on someone else's port with no visible symptom
# until something moves that should not have.
#
# The fleet's address book lives in sitl.conf beside this script -- at minimum
# SIM_IP and INDICES. Values resolve as: the flags above, then the config file,
# then the environment, then the defaults below -- the file is sourced, so it
# deliberately wins over an exported variable of the same name. sitl.conf.example
# is the template, and is also baked into the image at /opt/inav/.
#
# Nothing has to exist on the host: the image carries the SITL, a configured
# eeprom and this script. DATA_ROOT opts back into per-vehicle persistence.
#
#   IMAGE             inav-sitl:9.1
#   IMAGE_TAR         tar.gz to docker load when IMAGE is not present
#   SIM_BASE_PORT     49000     must equal InavSimSettings > BaseListenPort
#   TCP_BASE_PORT     5760      MSP; each vehicle strides by TCP_STRIDE
#   CHANMAP           M01-01,M02-02,M03-03,M04-04
#   SIM_IP            simulator host; required unless the docker host is WSL
#   DATA_ROOT         unset -> no volume, eeprom lives in the container
#   DOCKER            docker; "sudo docker" on a WSL daemon
set -euo pipefail

# Anchored to this script, not to the working directory: the containers often
# run under a different docker than the shell this is launched from, and a
# $PWD-relative default quietly resolves to whatever tree you happened to be
# standing in.
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# --- configuration -----------------------------------------------------------
#
# Sourced before the defaults, so the file wins over the environment and the
# flags win over the file. Keeping one editable address book per deployment is
# the point: SIM_IP and INDICES are the only two values that differ between
# machines, and retyping them as flags is how they drift.

SITL_CONF=${SITL_CONF:-$SCRIPT_DIR/sitl.conf}
if [ -f "$SITL_CONF" ]; then
    # shellcheck disable=SC1090
    . "$SITL_CONF"
fi

DOCKER=${DOCKER:-docker}
IMAGE=${IMAGE:-inav-sitl:9.1}
IMAGE_TAR=${IMAGE_TAR:-}
SIM_BASE_PORT=${SIM_BASE_PORT:-49000}
TCP_BASE_PORT=${TCP_BASE_PORT:-5760}
CHANMAP=${CHANMAP:-M01-01,M02-02,M03-03,M04-04}
SIM_IP=${SIM_IP:-}
DATA_ROOT=${DATA_ROOT:-}
INDICES_CONF=${INDICES:-}
RECEIVER=${RECEIVER:-sim}

# serial_tcp.c binds base + id - 1 for SERIAL_PORT_COUNT (8) uarts, so the next
# vehicle has to start at least 8 above this one. 10 leaves a little room.
TCP_STRIDE=10

# ue.c gives up after UE_HANDSHAKE_TIMEOUT_MS (10000). Sit just past that so a
# container which is never going to connect is reported as such rather than
# merely "still waiting".
HANDSHAKE_WAIT=12

# Windows hands out ephemeral ports from here up, so a vehicle that lands in
# the range can lose its port to any outgoing connection that picks the same
# number. Mirrors the check InavSimSubsystem makes on the other side.
EPHEMERAL_PORT_FLOOR=49152

# --- arguments ---------------------------------------------------------------

COUNT=
INDEX_ARG=
INDEX_GIVEN=
SIM_PORT_OVERRIDE=
STOP_ONLY=
RECEIVER_GIVEN=

while (( $# > 0 )); do
    case $1 in
        --index|--simip|--simport|--receiver)
            # Checked before the shift, because under 'set -e' a shift past the
            # end kills the script with no message at all.
            if (( $# < 2 )); then
                echo "error: $1 needs a value." >&2
                exit 1
            fi
            case $1 in
                --index)    INDEX_ARG=$2; INDEX_GIVEN=1 ;;
                --simip)    SIM_IP=$2 ;;
                --simport)  SIM_PORT_OVERRIDE=$2 ;;
                --receiver) RECEIVER=$2; RECEIVER_GIVEN=1 ;;
            esac
            shift 2 ;;
        --stop)    STOP_ONLY=1; shift ;;
        -h|--help)
            cat <<'USAGE'
launch_sitl.sh [count]              start VehicleIndex 0..count-1 (default 4)
launch_sitl.sh --index N[,M...]     start an explicit set of VehicleIndex values
                --simip IP          simulator host (default: sitl.conf, else the
                                    WSL gateway)
                --simport PORT      override the listen port (single --index only)
                --receiver sim|msp  where the RC sticks come from (default sim)
                --stop              stop the configured fleet instead of starting

With no flags at all, INDICES and SIM_IP come from sitl.conf beside this
script. That file is the deployment's address book -- the two values that
differ between machines. Its template:

    SIM_IP=192.168.1.50      # the machine running the Unreal editor
    INDICES=0,1,2,3          # which aircraft to start, 0-based

    # IMAGE=inav-sitl:9.1
    # IMAGE_TAR=inav-sitl-9.1.tar.gz
    # SIM_BASE_PORT=49000
    # TCP_BASE_PORT=5760
    # RECEIVER=sim             # or msp, to fly it from an MSP client
    #                            # (SIM=msp works too, for docker run by hand)
    # DATA_ROOT=/srv/sitl-data   # uncomment to persist per-vehicle eeproms
    # DOCKER=sudo\ docker        # on WSL

All of it is also at /opt/inav/sitl.conf.example inside the image.
USAGE
            exit 0 ;;
        -*)        echo "error: unknown option '$1'. Try --help." >&2; exit 1 ;;
        *)
            if [[ -n $COUNT ]]; then
                echo "error: '$1': count is already '$COUNT'. [count] and --index are alternatives." >&2
                exit 1
            fi
            COUNT=$1; shift ;;
    esac
done

if [[ -n $INDEX_GIVEN && -n $COUNT ]]; then
    echo "error: give either [count] or --index, not both." >&2
    exit 1
fi

# Caught before the list is split, because the otherwise-identical failure --
# '--index "$FLEET"' with FLEET unset -- would fall through to the count branch
# and quietly start the default four on indices nobody asked for.
if [[ -n $INDEX_GIVEN && -z $INDEX_ARG ]]; then
    echo "error: --index was given an empty list." >&2
    echo "       Pass one or more VehicleIndex values, e.g. --index 2 or --index 0,2,3." >&2
    exit 1
fi

# Which eeprom the container seeds from. The image carries both; they differ in
# exactly one field, receiver_type, and that field is the whole question of
# where the sticks come from -- the simulator's datarefs, or an MSP client on
# the TCP port. The physics link and the sensor path are identical either way,
# so this is not "MSP instead of the simulator", it is "MSP instead of the
# simulator's RC".
#
# Checked here only so a typo is caught before N containers are started;
# entrypoint.sh does the mapping itself and is the authority on it.
case $RECEIVER in
    sim|msp) ;;
    *)  echo "error: --receiver must be 'sim' or 'msp', got '$RECEIVER'." >&2
        echo "       sim: RC from the simulator's datarefs (the default)." >&2
        echo "       msp: RC from an MSP client on the TCP port; the simulator" >&2
        echo "            still supplies the physics and the sensors." >&2
        exit 1 ;;
esac

# Which vehicles to act on, in precedence order: flag, positional count,
# sitl.conf, then the same default of four the script has always had.
IDX_LIST=()
if [[ -n $INDEX_GIVEN ]]; then
    IFS=',' read -r -a IDX_LIST <<<"$INDEX_ARG"
elif [[ -n $COUNT ]]; then
    if ! [[ $COUNT =~ ^[0-9]+$ ]] || (( COUNT < 1 )); then
        echo "error: count must be a positive integer, got '$COUNT'" >&2
        exit 1
    fi
    for ((i = 0; i < COUNT; i++)); do
        IDX_LIST+=("$i")
    done
elif [[ -n $INDICES_CONF ]]; then
    IFS=',' read -r -a IDX_LIST <<<"$INDICES_CONF"
else
    IDX_LIST=(0 1 2 3)
fi

if (( ${#IDX_LIST[@]} == 0 )); then
    echo "error: no vehicles selected." >&2
    exit 1
fi

for IDX in "${IDX_LIST[@]}"; do
    if ! [[ $IDX =~ ^[0-9]+$ ]]; then
        echo "error: VehicleIndex values are 0-based; '$IDX' is not one." >&2
        echo "       --index 0 is the first aircraft, --index 2 the third." >&2
        exit 1
    fi
    if (( SIM_BASE_PORT + IDX >= EPHEMERAL_PORT_FLOOR )); then
        echo "error: --index $IDX would listen on udp/$((SIM_BASE_PORT + IDX)), inside the" >&2
        echo "       system's ephemeral port range ($EPHEMERAL_PORT_FLOOR+)." >&2
        exit 1
    fi
done

if [[ -n $SIM_PORT_OVERRIDE && ${#IDX_LIST[@]} -ne 1 ]]; then
    echo "error: --simport names a single simulator port, so it needs exactly one --index." >&2
    exit 1
fi

# --- the docker side ---------------------------------------------------------

# Checked up front because every later failure would otherwise be reported as
# something else: an unreachable daemon and a missing image are the same "no"
# from 'docker image inspect'.
if ! $DOCKER info >/dev/null 2>&1; then
    cat >&2 <<EOF
error: cannot talk to the docker daemon through '$DOCKER'.

On WSL the daemon normally needs root, so either run this under sudo or set
DOCKER='sudo docker' in $SITL_CONF.
EOF
    exit 1
fi

if [[ -n $STOP_ONLY ]]; then
    for IDX in "${IDX_LIST[@]}"; do
        NAME="inav-sitl-$IDX"
        if $DOCKER inspect "$NAME" >/dev/null 2>&1; then
            $DOCKER rm -f "$NAME" >/dev/null
            echo "  stopped $NAME"
        else
            echo "  $NAME: not present"
        fi
    done
    exit 0
fi

if ! $DOCKER image inspect "$IMAGE" >/dev/null 2>&1; then
    if [[ -n $IMAGE_TAR && -f $IMAGE_TAR ]]; then
        echo "loading $(basename "$IMAGE_TAR") ..."
        $DOCKER load -i "$IMAGE_TAR" >/dev/null
    fi
fi

if ! $DOCKER image inspect "$IMAGE" >/dev/null 2>&1; then
    cat >&2 <<EOF
error: image '$IMAGE' is not loaded here.

Transfer it and load it first:

    gunzip -c inav-sitl-9.1.tar.gz | $DOCKER load

or set IMAGE_TAR in $SITL_CONF and this script will do that for you.
EOF
    exit 1
fi

# Asked of the *daemon*, not of this shell. The script is just as often launched
# from the Windows side, where uname describes the client and not the machine
# that will actually run the container.
docker_is_wsl() {
    $DOCKER info --format '{{.KernelVersion}}' 2>/dev/null | grep -qi microsoft
}

if [[ -z $SIM_IP ]]; then
    # The default gateway is the Windows host ONLY under WSL2 NAT. Anywhere else
    # it is a router, and packets sent there are not rejected -- they are simply
    # never answered, so the only symptom is a handshake that times out ten
    # seconds later. Fail here rather than start N deaf containers.
    if docker_is_wsl; then
        echo "note: SIM_IP is unset; the containers will use the WSL default gateway,"
        echo "note: which is the Windows host under WSL2 NAT."
    else
        cat >&2 <<EOF
error: SIM_IP is unset and the docker host is not WSL.

Point it at the machine running the simulator. On a plain Linux host there is
no safe default -- the best guess available is the default gateway, which is a
router, and packets sent there vanish without an error.

    SIM_IP=192.168.1.50   in $SITL_CONF
    ./launch_sitl.sh --simip 192.168.1.50
EOF
        exit 1
    fi
fi

# --- the eeprom --------------------------------------------------------------
#
# Nothing to do here any more, and that is deliberate. The image carries a
# *configured* eeprom and entrypoint.sh seeds /data/eeprom.bin from it on first
# boot, so there is no host-side file to get wrong. What used to live here --
# a template beside the script, copied per vehicle -- was one more thing that
# silently produced an unarmable aircraft when it was missing.
#
# The launcher's job is now only to notice: see the "[EEPROM] Loaded" check
# below, which is the single signal that separates a configured eeprom from a
# factory one.

if [[ -n $DATA_ROOT ]]; then
    mkdir -p "$DATA_ROOT"
fi

# --- start -------------------------------------------------------------------

echo "starting ${#IDX_LIST[@]} container(s) from $(basename "$IMAGE")"
if [[ $RECEIVER == msp ]]; then
    echo "RC: MSP -- the simulator supplies physics and sensors, an MSP client supplies sticks."
fi
echo

PIDS=()
for IDX in "${IDX_LIST[@]}"; do
    PORT=$((SIM_BASE_PORT + IDX))                 # VehicleIndex 0 -> 49000
    MSP=$((TCP_BASE_PORT + TCP_STRIDE * IDX))     # 5760, 5770, 5780, ...
    NAME="inav-sitl-$IDX"

    # --simport points this container somewhere other than the listener its
    # index names -- another machine's editor, or a probe. Only the destination
    # moves: the index still decides the name and the MSP port, so overriding
    # it does not quietly renumber the vehicle.
    if [[ -n $SIM_PORT_OVERRIDE ]]; then
        PORT=$SIM_PORT_OVERRIDE
    fi

    RUN_ARGS=(-d --name "$NAME" --network host)

    if [[ -n $DATA_ROOT ]]; then
        DIR="$DATA_ROOT/$IDX"
        mkdir -p "$DIR"

        # The template only ever seeds a *missing* eeprom, so a persisted one
        # wins over --receiver -- including when the two disagree. That is the
        # silent case worth naming: the fleet comes up looking healthy while
        # still taking its RC from the simulator, and the MSP client sees an
        # aircraft that never responds. RESEED=1 is the way out, and it is a
        # blunt instrument (it discards anything changed over MSP too).
        if [[ -s "$DIR/eeprom.bin" && -n $RECEIVER_GIVEN ]]; then
            echo "note: $DIR/eeprom.bin already exists, so --receiver $RECEIVER is not"
            echo "note: applied -- the template only seeds a missing eeprom. To make it"
            echo "note: take effect, discard that file or re-run with RESEED=1."
        fi

        # Writability is probed with the image itself. The host's -w answers
        # about the wrong user -- the container runs as uid 1000 -- and a /data
        # the container cannot write is not a degraded eeprom: the firmware
        # never checks its failed fopen before the fwrite, so it takes SIGSEGV
        # with no ports bound. (entrypoint.sh catches this one anyway and falls
        # back to /tmp; the probe is here so an explicitly configured DATA_ROOT
        # fails loudly instead of silently not persisting.)
        if ! $DOCKER run --rm --entrypoint sh -v "$DIR:/data" "$IMAGE" \
                -c 'touch /data/.wtest && rm -f /data/.wtest' >/dev/null 2>&1; then
            echo "error: $DIR is not writable by uid 1000 inside the container." >&2
            echo "       chown 1000:1000 '$DIR'    (or drop DATA_ROOT from $SITL_CONF)" >&2
            exit 1
        fi
        RUN_ARGS+=(-v "$DIR:/data")
    fi

    # RECEIVER rather than EEPROM_TEMPLATE: entrypoint.sh owns the mapping from
    # mode name to file, so the launcher does not have to keep a second copy of
    # which path means what.
    RUN_ARGS+=(-e SIM=ue -e SIM_PORT="$PORT" -e RECEIVER="$RECEIVER")
    if [[ -n $SIM_IP ]]; then
        RUN_ARGS+=(-e SIM_IP="$SIM_IP")
    fi
    # Forwarded rather than reimplemented: entrypoint.sh already owns what
    # RESEED means, and the note above points at it by name.
    if [[ -n ${RESEED:-} ]]; then
        RUN_ARGS+=(-e RESEED="$RESEED")
    fi

    # --tcpbaseport is what keeps each vehicle's MSP ports off the others'. The
    # entrypoint derives the sim side from SIM_PORT, but nothing derives this
    # one: leave it out and every container races for 5760, the losers log
    # 'Unable to bind socket' and their sim link still comes up perfectly --
    # so the failure hides behind a green UE handshake.
    #
    # No --rm on purpose: a container that exits on a bad configuration would
    # have its log deleted within the second, and the exit code below would
    # have nothing left to explain it. Re-running this script removes them.
    $DOCKER run "${RUN_ARGS[@]}" "$IMAGE" \
        --tcpbaseport="$MSP" \
        --chanmap="$CHANMAP" \
        >/dev/null

    # A leftover container held the name; replacing it is what re-running means.
    PIDS+=("$NAME|$IDX|$PORT|$MSP")
    echo "  $NAME: VehicleIndex $IDX -> udp/$PORT (sim)   tcp/$MSP (MSP)"
done

# --- did they actually connect? ----------------------------------------------
#
# ue.c handshakes once inside a ten second window and never retries, so this is
# the only chance the link gets. Poll rather than sleep a fixed amount: the
# common case settles in well under a second, and the failure case is worth
# waiting the full deadline for.
#
# 'Waiting for connection' in the log below means the window was missed --
# the level was not in Play when the container came up.

echo
echo "waiting up to ${HANDSHAKE_WAIT}s for the handshake..."

DEADLINE=$((SECONDS + HANDSHAKE_WAIT))
while (( SECONDS < DEADLINE )); do
    ALL_SETTLED=1
    for ENTRY in "${PIDS[@]}"; do
        IFS='|' read -r NAME IDX PORT MSP <<<"$ENTRY"

        # A container that already exited has said everything it is going to.
        # Without this the loop would sit out the whole deadline on a refused
        # start -- the one case where the answer is known immediately.
        if [[ $($DOCKER inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null) != true ]]; then
            continue
        fi

        LOG=$($DOCKER logs "$NAME" 2>&1 || true)
        if ! grep -qE "Connection with UE simulator (successfully )?(NOT )?established|No UE simulator handshake within" <<<"$LOG"; then
            ALL_SETTLED=0
        fi
    done
    if (( ALL_SETTLED )); then
        break
    fi
    sleep 0.5
done

FAILED=0
for ENTRY in "${PIDS[@]}"; do
    IFS='|' read -r NAME IDX PORT MSP <<<"$ENTRY"

    # Pulled into a variable rather than piped into grep: 'grep -q' exits at the
    # match and closes the pipe, and under pipefail that SIGPIPE would be read
    # as a failure on exactly the runs that succeeded.
    LOG=$($DOCKER logs "$NAME" 2>&1 || true)
    STATE=$($DOCKER inspect -f '{{.State.Status}} {{.State.ExitCode}}' "$NAME" 2>/dev/null || echo "unknown -1")
    read -r STATUS EXITCODE <<<"$STATE"

    PROBLEM=
    if [[ $STATUS == exited || $STATUS == dead ]]; then
        case $EXITCODE in
            1) PROBLEM="exit 1: the image has no SITL binary" ;;
            2) PROBLEM="exit 2: the entrypoint was given no usable SIM_IP" ;;
            3) PROBLEM="exit 3: the entrypoint found nowhere to put the eeprom" ;;
            # The firmware does not check the failed fopen before its fwrite, so
            # an eeprom it cannot open is a SIGSEGV rather than the failureMode()
            # hang you would expect. Only reachable by bypassing entrypoint.sh
            # (an explicit --path, say), but it is the least self-explaining exit
            # code in the set, so it gets named.
            139) PROBLEM="SIGSEGV: the firmware could not open its eeprom" ;;
            *) PROBLEM="exited with code $EXITCODE" ;;
        esac
    # The eeprom check comes first because it is the one failure the handshake
    # check cannot see. A factory-default eeprom still links to the simulator
    # perfectly -- datarefs subscribed, frames flowing -- while the flight
    # controller sits on ARMING_DISABLED_RC_LINK forever, because
    # receiver_type = NONE means the SIM receiver is never selected.
    elif ! grep -qF "[EEPROM] Loaded" <<<"$LOG"; then
        PROBLEM="eeprom was never loaded (see below)"
    elif grep -q "No UE simulator handshake within" <<<"$LOG"; then
        PROBLEM="missed the handshake window on udp/$PORT"
    elif ! grep -q "Connection with UE simulator successfully established" <<<"$LOG"; then
        PROBLEM="no handshake on udp/$PORT"
    elif ! grep -qF "Bind TCP [::]:$MSP to UART1" <<<"$LOG"; then
        PROBLEM="MSP port $MSP never bound"
    fi

    if [[ -z $PROBLEM ]]; then
        echo "  ok    $NAME  udp/$PORT  tcp/$MSP"
        if grep -qF "[EEPROM] Created" <<<"$LOG"; then
            echo "        warn: booted on a factory eeprom -- seeding did not happen"
        fi
    else
        FAILED=$((FAILED + 1))
        echo "  FAIL  $NAME  -- $PROBLEM"
        grep -E "EEPROM|SYSTEM|SOCKET|SIM|entrypoint" <<<"$LOG" | tail -6 | sed 's/^/          /'
    fi
done

echo
if (( FAILED > 0 )); then
    cat <<EOF
$FAILED container(s) came up wrong. Fix the cause and run this script again --
it replaces containers rather than adding to them, which is also how a vehicle
that missed its one-shot handshake is recovered.

Keep the level in Play while restarting: a container that connects while
nothing is listening stays deaf for the rest of its life.
EOF
    exit 1
fi

echo "all ${#IDX_LIST[@]} connected. Logs:   $DOCKER logs -f inav-sitl-${IDX_LIST[0]}"
