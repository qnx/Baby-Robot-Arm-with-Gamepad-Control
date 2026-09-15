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
umask 077

readonly ROS2_HOST_INSTALLATION_PATH="/opt/ros/jazzy"
readonly ROS_SETUP="${ROS2_HOST_INSTALLATION_PATH}/local_setup.bash"

# Anchor the build before evaluating sourced metadata. Invoking this script
# from another directory must not build or promote an unrelated workspace.
# Privileged Bash mode prevents inherited function exports and BASH_ENV from
# executing before this file. A later check also rejects the still-raw
# exported-function environment entries before any descendant shell can import
# them. Resolve the path using builtins so command lookup cannot precede the
# trusted PATH established below.
script_dir="${BASH_SOURCE[0]%/*}"
if [ "${script_dir}" = "${BASH_SOURCE[0]}" ]; then
    script_dir=.
fi
repo_root="$(cd -- "${script_dir}" && pwd -P)"
readonly repo_root
if [ -z "${repo_root}" ] || [ "${repo_root}" = "/" ] || \
   [ ! -e "${repo_root}/.git" ] || [ ! -d "${repo_root}/src" ] || \
   [ ! -f "${repo_root}/build.sh" ]; then
    echo "Error: refusing to build from an unverified repository root: ${repo_root}" >&2
    exit 1
fi
cd -- "${repo_root}"

# Workspace defaults and metadata can inject build arguments despite an
# otherwise trusted command line. They are not part of this repository's
# approved build contract, so refuse them instead of silently consuming them.
for workspace_metadata in \
    "${repo_root}/colcon_defaults.yaml" \
    "${repo_root}/colcon.meta"; do
    if [ -e "${workspace_metadata}" ] || [ -L "${workspace_metadata}" ]; then
        echo "Error: refusing unapproved colcon workspace metadata: ${workspace_metadata}" >&2
        exit 1
    fi
done

if [ ! -f "${ROS_SETUP}" ]; then
    echo "Error: ROS2 Jazzy was not found at ${ROS2_HOST_INSTALLATION_PATH}." >&2
    exit 1
fi

readonly BUILD_ENV_SANITIZER="${repo_root}/cmake/sanitize-build-environment.bash"
if [ ! -f "${BUILD_ENV_SANITIZER}" ] || [ -L "${BUILD_ENV_SANITIZER}" ]; then
    echo "Error: trusted build-environment sanitizer is missing or is a symlink." >&2
    exit 1
fi
# shellcheck disable=SC1090
. "${BUILD_ENV_SANITIZER}"
readonly -f baby_robot_clear_untrusted_cmake_environment
readonly -f baby_robot_normalize_trusted_prefix_path
readonly -f baby_robot_reject_unsafe_raw_environment
readonly -f baby_robot_require_fresh_output_base

# Limit command discovery to administrator-managed locations before sourcing
# executable build metadata. Generated setup hooks can honor CURRENT_PREFIX
# variables, so clear them before sourcing and again afterwards rather than
# permitting an inherited path to redirect setup processing.
export PATH="${ROS2_HOST_INSTALLATION_PATH}/bin:/system/bin:/usr/local/bin:/usr/bin:/bin"
export PYTHONNOUSERSITE=1
unset PYTHONPATH PYTHONHOME PYTHONSTARTUP PYTHONINSPECT PYTHONPLATLIBDIR \
    PYTHONWARNINGS PYTHONBREAKPOINT PYTHONOPTIMIZE BROWSER \
    VIRTUAL_ENV CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_EXE PYENV_VERSION \
    _CE_CONDA _CE_M \
    LD_PRELOAD LD_LIBRARY_PATH LD_AUDIT LD_PROFILE LD_PROFILE_OUTPUT \
    LD_DEBUG LD_DEBUG_OUTPUT GLIBC_TUNABLES BASH_ENV ENV CDPATH \
    AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX CMAKE_PREFIX_PATH \
    COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX COLCON_HOME \
    COLCON_DEFAULTS_FILE COLCON_DEFAULT_EXECUTOR COLCON_EXTENSION_BLOCKLIST \
    COLCON_LOG_LEVEL COLCON_WARNINGS COLCON_PYTHON_EXECUTABLE COLCON_TRACE \
    AMENT_PYTHON_EXECUTABLE AMENT_SHELL POWERSHELL_COMMAND PSModulePath \
    ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION \
    CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE RMW_IMPLEMENTATION \
    CC CXX CFLAGS CXXFLAGS LDFLAGS \
    GCC_COMPARE_DEBUG GCC_COMPARE_DEBUG_OPT CCC_OVERRIDE_OPTIONS \
    DEPENDENCIES_OUTPUT SUNPRO_DEPENDENCIES \
    MAKE MAKE_COMMAND MAKEFILES MAKEFLAGS MAKEOVERRIDES GNUMAKEFLAGS \
    MAKESILENT MFLAGS VERBOSE COLOR VPATH GPATH
baby_robot_reject_unsafe_raw_environment
baby_robot_clear_untrusted_cmake_environment
# Generated colcon prefix templates expand unset variables directly and are not
# nounset-clean. Relax only that option while evaluating the trusted,
# administrator-owned ROS setup, then restore the complete strict policy.
set +u
# shellcheck disable=SC1090
. "${ROS_SETUP}"
set -Eeuo pipefail
# Setup hooks may prepend interactive or user tool directories. Resolve colcon
# only after restoring the administrator-controlled build PATH.
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX \
    AMENT_PYTHON_EXECUTABLE AMENT_SHELL \
    COLCON_PYTHON_EXECUTABLE COLCON_TRACE \
    PYTHONWARNINGS PYTHONBREAKPOINT PYTHONOPTIMIZE BROWSER
# CMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH=FALSE blocks open-ended PackageName_DIR
# injection, but it also blocks the legitimate ROS CMAKE_PREFIX_PATH. Convert
# only the post-sanitization, trusted setup result into an explicit cache list;
# inherited CMAKE_PREFIX_PATH was cleared before the setup file ran.
TRUSTED_CMAKE_PREFIX_PATH="$(baby_robot_normalize_trusted_prefix_path \
    "${CMAKE_PREFIX_PATH:-}" "${ROS2_HOST_INSTALLATION_PATH}")"
readonly TRUSTED_CMAKE_PREFIX_PATH
unset CMAKE_PREFIX_PATH
# Setup hooks are executable deployment metadata and can set CMake controls of
# their own. Reassert the same allowlist after sourcing so a hook cannot restore
# an inherited toolchain, generator, project include, install prefix, or DESTDIR.
baby_robot_clear_untrusted_cmake_environment
export PATH="${ROS2_HOST_INSTALLATION_PATH}/bin:/system/bin:/usr/local/bin:/usr/bin:/bin"
hash -r

# Colcon otherwise reads ~/.colcon and workspace files relative to its current
# directory. Use a fresh 0700 configuration/cwd and remove it after this build.
TRUSTED_COLCON_HOME="$(mktemp -d /tmp/baby-robot-colcon.XXXXXXXX)"
readonly TRUSTED_COLCON_HOME
chmod 700 "${TRUSTED_COLCON_HOME}"
cleanup_colcon_home() {
    case "${TRUSTED_COLCON_HOME}" in
        /tmp/baby-robot-colcon.*)
            rm -rf -- "${TRUSTED_COLCON_HOME}" || \
                echo "Warning: unable to remove temporary COLCON_HOME." >&2
            ;;
        *)
            echo "Error: refusing to clean unexpected COLCON_HOME path." >&2
            ;;
    esac
}
trap cleanup_colcon_home EXIT
export COLCON_HOME="${TRUSTED_COLCON_HOME}"
export COLCON_DEFAULTS_FILE=/dev/null

readonly BUILD_BASE="${repo_root}/build"
readonly INSTALL_BASE="${repo_root}/install"
readonly LOG_BASE="${repo_root}/log"
for output_base in "${BUILD_BASE}" "${INSTALL_BASE}" "${LOG_BASE}"; do
    baby_robot_require_fresh_output_base "${output_base}"
done

COLCON_BIN="$(type -P colcon || true)"
if [ -z "${COLCON_BIN}" ]; then
    echo "Error: colcon was not found in the trusted build PATH." >&2
    exit 1
fi
readonly COLCON_BIN

# Launch the actual build in a positive allowlist instead of trusting that a
# denylist can enumerate every variable interpreted by GNU make, CMake, GCC,
# Python, or a future colcon extension. Inputs required by the QNX/ROS toolchain
# remain explicit so release CI can source them from its versioned manifest.
TRUSTED_ENV_BIN="$(type -P env || true)"
case "${TRUSTED_ENV_BIN}" in
    /system/bin/env|/usr/bin/env|/bin/env) ;;
    *)
        echo "Error: env was not found in the trusted build PATH." >&2
        exit 1
        ;;
esac
readonly TRUSTED_ENV_BIN

declare -a TRUSTED_BUILD_ENVIRONMENT=(
    "PATH=${PATH}"
    "HOME=${TRUSTED_COLCON_HOME}"
    "TMPDIR=${TRUSTED_COLCON_HOME}"
    "LANG=C"
    "LC_ALL=C"
    "PYTHONNOUSERSITE=1"
    "PYTHONUTF8=1"
    "COLCON_HOME=${TRUSTED_COLCON_HOME}"
    "COLCON_DEFAULTS_FILE=/dev/null"
)
for variable_name in \
    QNX_HOST QNX_TARGET QNX_CONFIGURATION QNX_CONFIGURATION_EXCLUSIVE \
    QNX_LICENSE_FILE LM_LICENSE_FILE ARCH CPUVAR CPUVARDIR NDDSHOME \
    ROS_EXTERNAL_DEPS_INSTALL ROS2_HOST_INSTALLATION_PATH \
    ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION RMW_IMPLEMENTATION \
    AMENT_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH; do
    if [[ -v "${variable_name}" ]]; then
        TRUSTED_BUILD_ENVIRONMENT+=("${variable_name}=${!variable_name}")
    fi
done
readonly -a TRUSTED_BUILD_ENVIRONMENT

echo "ROS build environment:"
for variable_name in ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION RMW_IMPLEMENTATION; do
    printf '  %s=%s\n' "${variable_name}" "${!variable_name:-<unset>}"
done

# Run from the empty configuration directory so colcon cannot auto-load a
# workspace colcon_defaults.yaml or ./colcon.meta. Package discovery and every
# writable output base are explicit and absolute. RelWithDebInfo keeps target
# diagnostics while enabling the optimization required for _FORTIFY_SOURCE.
(
    cd -- "${TRUSTED_COLCON_HOME}"
    "${TRUSTED_ENV_BIN}" -i "${TRUSTED_BUILD_ENVIRONMENT[@]}" \
        "${COLCON_BIN}" --log-base "${LOG_BASE}" build \
        --base-paths "${repo_root}/src" \
        --ignore-user-meta \
        --build-base "${BUILD_BASE}" \
        --install-base "${INSTALL_BASE}" \
        --merge-install \
        --cmake-force-configure \
        --cmake-args \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_PREFIX_PATH="${TRUSTED_CMAKE_PREFIX_PATH}" \
        -DCMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH=FALSE \
        -DCMAKE_FIND_USE_PACKAGE_ROOT_PATH=FALSE \
        -DCMAKE_FIND_USE_PACKAGE_REGISTRY=FALSE \
        -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=FALSE \
        -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=TRUE \
        -DPython_FIND_VIRTUALENV=STANDARD \
        -DPython3_FIND_VIRTUALENV=STANDARD
)
echo "Build completed successfully."
