#!/bin/bash -p
#
# Copyright (c) 2026, BlackBerry Limited. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

set -Eeuo pipefail

# `-p` is a pre-body boundary: it prevents Bash from importing exported shell
# functions or evaluating BASH_ENV before the sanitization below can execute.
# The service manager must still supply a clean environment because the dynamic
# loader necessarily processes loader variables before Bash itself starts.

ROBOT_ID="${ROBOT_ID:-robot1}"
ROBOT_INSTALL_PREFIX="${ROBOT_INSTALL_PREFIX:-/opt/baby_robot_arm/current}"
ROS_SECURITY_KEYSTORE="${ROS_SECURITY_KEYSTORE:-/etc/baby_robot_arm/security}"
ENABLE_EXPERIMENTAL_IK="${ENABLE_EXPERIMENTAL_IK:-0}"
ALLOW_SOFTWARE_ONLY_OUTPUT="${ALLOW_SOFTWARE_ONLY_OUTPUT:-0}"

# Resolve even the privilege check from administrator-controlled locations.
# QNX Python entry points use /system/bin/env, so /system/bin must remain in the
# trusted path even though inherited path components are discarded.
export PATH="/opt/ros/jazzy/bin:/system/bin:/usr/bin:/bin"
export PYTHONNOUSERSITE=1
unset LD_PRELOAD LD_LIBRARY_PATH LD_AUDIT LD_PROFILE LD_PROFILE_OUTPUT \
    LD_DEBUG LD_DEBUG_OUTPUT GLIBC_TUNABLES \
    PYTHONHOME PYTHONSTARTUP PYTHONINSPECT PYTHONPLATLIBDIR \
    PYTHONWARNINGS PYTHONBREAKPOINT PYTHONOPTIMIZE BROWSER \
    BASH_ENV CDPATH ENV \
    AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX AMENT_PYTHON_EXECUTABLE AMENT_SHELL \
    CMAKE_PREFIX_PATH COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX \
    COLCON_PYTHON_EXECUTABLE COLCON_TRACE \
    ROS_SECURITY_ENCLAVE_OVERRIDE ROS_DISCOVERY_SERVER \
    ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS \
    CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE \
    FASTDDS_DEFAULT_PROFILES_FILE FASTDDS_ENVIRONMENT_FILE SKIP_DEFAULT_XML \
    RMW_IMPLEMENTATION
umask 077

# Bash -p blocks function import into this shell, but it deliberately leaves
# raw BASH_FUNC_* and non-portable names in the process environment. Reject
# them now so a later non-privileged Bash or interpreter child cannot consume
# an entry that this shell cannot represent and unset.
ENV_BIN="$(type -P env || true)"
case "${ENV_BIN}" in
    /system/bin/env|/usr/bin/env|/bin/env) ;;
    *)
        echo "Error: env was not found in the trusted runtime PATH." >&2
        exit 1
        ;;
esac
readonly ENV_BIN
while IFS='=' read -r entry_name _; do
    case "${entry_name}" in
        BASH_FUNC_*)
            echo "Error: refusing an exported Bash function environment entry." >&2
            exit 1
            ;;
    esac
    if [[ ! "${entry_name}" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]]; then
        echo "Error: refusing an environment entry with an unsafe name." >&2
        exit 1
    fi
done < <("${ENV_BIN}")
unset entry_name

case "${ROBOT_ID}" in
    ''|[!A-Za-z_]*|*[!A-Za-z0-9_]*)
        echo "Error: ROBOT_ID must be a valid ROS name token." >&2
        exit 2
        ;;
esac
if [ "${#ROBOT_ID}" -gt 63 ]; then
    echo "Error: ROBOT_ID must not exceed 63 characters." >&2
    exit 2
fi

case "${ROBOT_INSTALL_PREFIX}" in
    /opt/baby_robot_arm/*) ;;
    *)
        echo "Error: ROBOT_INSTALL_PREFIX must be under /opt/baby_robot_arm/." >&2
        exit 2
        ;;
esac

case "${ROS_SECURITY_KEYSTORE}" in
    /etc/baby_robot_arm/*) ;;
    *)
        echo "Error: ROS_SECURITY_KEYSTORE must be under /etc/baby_robot_arm/." >&2
        exit 2
        ;;
esac

case "${ROBOT_INSTALL_PREFIX}/" in
    *'//'*|*'/../'*|*'/./'*)
        echo "Error: ROBOT_INSTALL_PREFIX contains an unsafe path component." >&2
        exit 2
        ;;
esac

case "${ROS_SECURITY_KEYSTORE}/" in
    *'//'*|*'/../'*|*'/./'*)
        echo "Error: ROS_SECURITY_KEYSTORE contains an unsafe path component." >&2
        exit 2
        ;;
esac

case "${ENABLE_EXPERIMENTAL_IK}" in
    0|1) ;;
    *)
        echo "Error: ENABLE_EXPERIMENTAL_IK must be 0 or 1." >&2
        exit 2
        ;;
esac

case "${ALLOW_SOFTWARE_ONLY_OUTPUT}" in
    0|1) ;;
    *)
        echo "Error: ALLOW_SOFTWARE_ONLY_OUTPUT must be 0 or 1." >&2
        exit 2
        ;;
esac

if [ "${ENABLE_EXPERIMENTAL_IK}" -eq 1 ]; then
    readonly ENABLE_IK_PARAMETER=true
else
    readonly ENABLE_IK_PARAMETER=false
fi

if [ "${ALLOW_SOFTWARE_ONLY_OUTPUT}" -eq 1 ]; then
    readonly SOFTWARE_ONLY_PARAMETER=true
    echo "DANGER: bench-only software output override enabled; process/I2C loss cannot assert OE." >&2
else
    readonly SOFTWARE_ONLY_PARAMETER=false
fi

ID_BIN="$(type -P id || true)"
if [ -z "${ID_BIN}" ]; then
    echo "Error: id was not found in the trusted runtime PATH." >&2
    exit 1
fi
readonly ID_BIN
if [ "$("${ID_BIN}" -u)" -eq 0 ]; then
    echo "Error: do not run the ROS graph as root." >&2
    echo "Grant the service account narrowly scoped I2C/HIDDI permissions instead." >&2
    exit 1
fi

if [ -z "${ROS_DOMAIN_ID:-}" ]; then
    echo "Error: set a unique ROS_DOMAIN_ID for this robot before launch." >&2
    exit 1
fi

case "${ROS_DOMAIN_ID}" in
    *[!0-9]*|'')
        echo "Error: ROS_DOMAIN_ID must be an integer from 0 through 232." >&2
        exit 2
        ;;
esac
if [ "${#ROS_DOMAIN_ID}" -gt 3 ]; then
    echo "Error: ROS_DOMAIN_ID must be an integer from 0 through 232." >&2
    exit 2
fi
readonly DOMAIN_ID_DECIMAL=$((10#${ROS_DOMAIN_ID}))
if [ "${DOMAIN_ID_DECIMAL}" -gt 232 ]; then
    echo "Error: ROS_DOMAIN_ID must be an integer from 0 through 232." >&2
    exit 2
fi

if [ ! -d "${ROS_SECURITY_KEYSTORE}" ]; then
    echo "Error: SROS2 keystore not found: ${ROS_SECURITY_KEYSTORE}" >&2
    exit 1
fi

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "Error: ROS2 setup file not found at /opt/ros/jazzy/setup.bash" >&2
    exit 1
fi

if [ ! -f "${ROBOT_INSTALL_PREFIX}/local_setup.bash" ]; then
    echo "Error: promoted robot install not found: ${ROBOT_INSTALL_PREFIX}" >&2
    exit 1
fi

if [ ! -d /var/run/baby_robot_arm ] || [ -L /var/run/baby_robot_arm ]; then
    echo "Error: administrator-managed actuator lock directory is missing or unsafe." >&2
    exit 1
fi

# Resolve directory symlinks before sourcing executable deployment metadata and
# reject a promoted/current link that escapes the administrator-owned tree.
CANONICAL_INSTALL_PREFIX="$(cd -- "${ROBOT_INSTALL_PREFIX}" && pwd -P)"
CANONICAL_SECURITY_KEYSTORE="$(cd -- "${ROS_SECURITY_KEYSTORE}" && pwd -P)"
readonly CANONICAL_INSTALL_PREFIX CANONICAL_SECURITY_KEYSTORE
case "${CANONICAL_INSTALL_PREFIX}" in
    /opt/baby_robot_arm/*) ;;
    *)
        echo "Error: promoted install resolves outside /opt/baby_robot_arm/." >&2
        exit 1
        ;;
esac
case "${CANONICAL_SECURITY_KEYSTORE}" in
    /etc/baby_robot_arm/*) ;;
    *)
        echo "Error: keystore resolves outside /etc/baby_robot_arm/." >&2
        exit 1
        ;;
esac
if [ -L "${ROBOT_INSTALL_PREFIX}/local_setup.bash" ]; then
    echo "Error: local_setup.bash must not be a symlink." >&2
    exit 1
fi

# Security-sensitive launches must not inherit user-controlled import or loader
# paths. An empty path component would make the current directory executable.
# Generated setup hooks also honor CURRENT_PREFIX variables; clear them before
# sourcing so an inherited value cannot redirect setup to an attacker path.
export LD_LIBRARY_PATH="/opt/ros/jazzy/lib:${CANONICAL_INSTALL_PREFIX}/lib"
export PYTHONPATH="/opt/ros/jazzy/usr/lib/python3.11/site-packages"
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX \
    AMENT_PYTHON_EXECUTABLE AMENT_SHELL \
    COLCON_PYTHON_EXECUTABLE COLCON_TRACE \
    PYTHONSTARTUP PYTHONINSPECT PYTHONPLATLIBDIR \
    PYTHONWARNINGS PYTHONBREAKPOINT PYTHONOPTIMIZE BROWSER

# shellcheck disable=SC1091
set +u
. /opt/ros/jazzy/setup.bash
set -Eeuo pipefail
# Upstream setup templates may assign interpreter selectors. The promoted
# overlay must resolve its own helpers from the trusted PATH, not inherit a
# selector from either the service environment or the upstream setup.
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX \
    AMENT_PYTHON_EXECUTABLE AMENT_SHELL \
    COLCON_PYTHON_EXECUTABLE COLCON_TRACE \
    PYTHONSTARTUP PYTHONINSPECT PYTHONPLATLIBDIR \
    PYTHONWARNINGS PYTHONBREAKPOINT PYTHONOPTIMIZE BROWSER
# shellcheck disable=SC1090
set +u
. "${CANONICAL_INSTALL_PREFIX}/local_setup.bash"
set -Eeuo pipefail

# Setup hooks may prepend utility locations for interactive use. Runtime node
# discovery does not need them because exact installed paths are used below.
export PATH="/opt/ros/jazzy/bin:/system/bin:/usr/bin:/bin"
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX \
    AMENT_PYTHON_EXECUTABLE AMENT_SHELL \
    COLCON_PYTHON_EXECUTABLE COLCON_TRACE \
    PYTHONSTARTUP PYTHONINSPECT PYTHONPLATLIBDIR \
    PYTHONWARNINGS PYTHONBREAKPOINT PYTHONOPTIMIZE BROWSER \
    ROS_SECURITY_ENCLAVE_OVERRIDE ROS_DISCOVERY_SERVER \
    ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS \
    CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE \
    FASTDDS_DEFAULT_PROFILES_FILE FASTDDS_ENVIRONMENT_FILE SKIP_DEFAULT_XML

export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_KEYSTORE="${CANONICAL_SECURITY_KEYSTORE}"
# All control nodes are local; blocking off-host discovery reduces exposure even
# if a DDS policy is accidentally broadened.
export ROS_LOCALHOST_ONLY=1
# Clear all inherited discovery/profile inputs, then express the same local-only
# policy through Jazzy's discovery controls. SKIP_DEFAULT_XML prevents Fast DDS
# from auto-loading DEFAULT_FASTRTPS_PROFILES.xml from an untrusted service cwd.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=""
export SKIP_DEFAULT_XML=1

readonly ROS2_BIN="/opt/ros/jazzy/bin/ros2"
readonly ROBOT_NAMESPACE="/baby_robot_arm/${ROBOT_ID}"
readonly JOY_NODE_BIN="${CANONICAL_INSTALL_PREFIX}/lib/joy_teleop_hiddi/joy_teleop_node"
readonly IK_NODE_BIN="${CANONICAL_INSTALL_PREFIX}/lib/ik_solver/ik_solver_node"
readonly CONTROLLER_NODE_BIN="${CANONICAL_INSTALL_PREFIX}/lib/arm_controller/arm_controller_node.py"

if [ ! -x "${ROS2_BIN}" ]; then
    echo "Error: ros2 executable not found at ${ROS2_BIN}" >&2
    exit 1
fi

for required_binary in "${JOY_NODE_BIN}" "${CONTROLLER_NODE_BIN}"; do
    if [ ! -x "${required_binary}" ]; then
        echo "Error: required installed node is not executable: ${required_binary}" >&2
        exit 1
    fi
done
if [ "${ENABLE_EXPERIMENTAL_IK}" -eq 1 ] && [ ! -x "${IK_NODE_BIN}" ]; then
    echo "Error: required installed IK node is not executable: ${IK_NODE_BIN}" >&2
    exit 1
fi

required_enclaves=(joy controller)
if [ "${ENABLE_EXPERIMENTAL_IK}" -eq 1 ]; then
    required_enclaves+=(ik)
fi

for enclave_name in "${required_enclaves[@]}"; do
    enclave_dir="${CANONICAL_SECURITY_KEYSTORE}/enclaves${ROBOT_NAMESPACE}/${enclave_name}"
    if [ ! -d "${enclave_dir}" ]; then
        echo "Error: SROS2 enclave is missing: ${enclave_dir}" >&2
        exit 1
    fi
done

# Calibrated actuator limits remain the final non-bypassable envelope in every
# mode. Cartesian limits can narrow motion but never replace these joint limits.
readonly SERVO_MIN_LIMITS="[25.0, 0.0, 50.0, 0.0, 0.0, 15.0]"
readonly SERVO_MAX_LIMITS="[75.0, 50.0, 100.0, 100.0, 100.0, 65.0]"
readonly CART_MIN_LIMITS="[-0.16, 0.10, -0.145]"
readonly CART_MAX_LIMITS="[0.16, 0.21, -0.014]"

declare -a CHILD_PIDS=()
cleanup_started=0

stop_children() {
    if [ "${cleanup_started}" -ne 0 ]; then
        return
    fi
    cleanup_started=1
    trap - INT TERM

    local pid
    for pid in "${CHILD_PIDS[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            kill -TERM "${pid}" 2>/dev/null || true
        fi
    done

    # Cleanup is deliberately bounded: a wedged node must not keep sibling
    # publishers or the actuator process alive after the control unit fails.
    local attempt
    local alive
    for attempt in {1..50}; do
        alive=0
        for pid in "${CHILD_PIDS[@]}"; do
            if kill -0 "${pid}" 2>/dev/null; then
                alive=1
                break
            fi
        done
        [ "${alive}" -eq 0 ] && break
        sleep 0.1
    done

    for pid in "${CHILD_PIDS[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            kill -KILL "${pid}" 2>/dev/null || true
        fi
        wait "${pid}" 2>/dev/null || true
    done
}

trap stop_children EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_node() {
    "$@" &
    CHILD_PIDS+=("$!")
}

echo "Starting the secured robot control unit in namespace ${ROBOT_NAMESPACE}..."

# Catchable shell/child failures are handled below. SIGKILL cannot run a shell
# trap, so the production service manager must own this complete process group
# and kill every member if the launcher itself disappears.

start_node \
    "${JOY_NODE_BIN}" \
    --ros-args \
    --enclave "${ROBOT_NAMESPACE}/joy" \
    -r __ns:="${ROBOT_NAMESPACE}"

if [ "${ENABLE_EXPERIMENTAL_IK}" -eq 1 ]; then
    # IK is opt-in because the checked-in URDF is incomplete. The hardened
    # solver intentionally refuses that model instead of publishing unsafe data.
    start_node \
        "${IK_NODE_BIN}" \
        --ros-args \
        --enclave "${ROBOT_NAMESPACE}/ik" \
        -r __ns:="${ROBOT_NAMESPACE}" \
        -p cart_min_limits:="${CART_MIN_LIMITS}" \
        -p cart_max_limits:="${CART_MAX_LIMITS}"
fi

start_node \
    "${CONTROLLER_NODE_BIN}" \
    --ros-args \
    --enclave "${ROBOT_NAMESPACE}/controller" \
    -r __ns:="${ROBOT_NAMESPACE}" \
    -p servo_min_limits:="${SERVO_MIN_LIMITS}" \
    -p servo_max_limits:="${SERVO_MAX_LIMITS}" \
    -p enable_autonomous_mode:=false \
    -p allow_software_only_output:="${SOFTWARE_ONLY_PARAMETER}" \
    -p enable_ik_mode:="${ENABLE_IK_PARAMETER}"

echo "Required nodes started. Loss of any active node stops the complete unit."

set +e
wait -n "${CHILD_PIDS[@]}"
child_status=$?
set -e

if [ "${child_status}" -eq 0 ]; then
    # A required node exiting cleanly while its peers still run is still a
    # control-system fault and must be visible to the service supervisor.
    child_status=1
fi

echo "A required node exited; stopping the robot control unit." >&2
stop_children
trap - EXIT
exit "${child_status}"
