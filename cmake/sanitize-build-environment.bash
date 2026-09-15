#!/bin/bash
#
# Copyright (c) 2026, BlackBerry Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# CMake initializes these settings from the process environment or they are
# consumed by common CMake/colcon build helpers. In particular,
# CMAKE_TOOLCHAIN_FILE, program search paths, compiler launchers, and project
# include variables can execute attacker-selected code during configure. The
# compiler and pkg-config search variables can silently substitute headers or
# libraries. GNU make's inherited MAKEFILES family can auto-include an
# attacker-selected makefile, so those controls are part of the same denylist.
# The repository's convenience build has no
# supported ambient override for them; approved settings belong on its explicit
# command line instead.
readonly -a BABY_ROBOT_UNTRUSTED_CMAKE_ENVIRONMENT=(
    AMENT_PYTHON_EXECUTABLE
    AMENT_SHELL
    CMAKE_ARGS
    CMAKE_BUILD_PARALLEL_LEVEL
    CMAKE_BUILD_TYPE
    CMAKE_COMMAND
    CMAKE_CONFIG_TYPE
    CMAKE_EXPORT_COMPILE_COMMANDS
    CMAKE_GENERATOR
    CMAKE_GENERATOR_INSTANCE
    CMAKE_GENERATOR_PLATFORM
    CMAKE_GENERATOR_TOOLSET
    CMAKE_INCLUDE_PATH
    CMAKE_INSTALL_MODE
    CMAKE_INSTALL_PREFIX
    CMAKE_LIBRARY_PATH
    CMAKE_C_COMPILER_LAUNCHER
    CMAKE_C_LINKER_LAUNCHER
    CMAKE_CXX_COMPILER_LAUNCHER
    CMAKE_CXX_LINKER_LAUNCHER
    CMAKE_CONFIG_DIR
    CMAKE_CROSSCOMPILING_EMULATOR
    CMAKE_MAKE_PROGRAM
    CMAKE_PROGRAM_PATH
    CMAKE_PROJECT_INCLUDE
    CMAKE_PROJECT_INCLUDE_BEFORE
    CMAKE_PROJECT_TOP_LEVEL_INCLUDES
    CMAKE_STAGING_PREFIX
    CMAKE_SYSROOT
    CMAKE_TEST_LAUNCHER
    CMAKE_TOOLCHAIN_FILE
    CMAKE_USER_MAKE_RULES_OVERRIDE
    CMAKE_USER_MAKE_RULES_OVERRIDE_C
    CMAKE_USER_MAKE_RULES_OVERRIDE_CXX
    COLOR
    COMPILER_PATH
    CPATH
    C_INCLUDE_PATH
    CPLUS_INCLUDE_PATH
    CCC_OVERRIDE_OPTIONS
    CONDA_DEFAULT_ENV
    CONDA_EXE
    CONDA_PREFIX
    COLCON_PYTHON_EXECUTABLE
    COLCON_TRACE
    CTEST_COMMAND
    DESTDIR
    DEPENDENCIES_OUTPUT
    GCC_EXEC_PREFIX
    GCC_COMPARE_DEBUG
    GCC_COMPARE_DEBUG_OPT
    GPATH
    GNUMAKEFLAGS
    LD_RUN_PATH
    LIBRARY_PATH
    MAKE
    MAKE_COMMAND
    MAKEFILES
    MAKEFLAGS
    MAKEOVERRIDES
    MAKESILENT
    MFLAGS
    OBJC_INCLUDE_PATH
    PKG_CONFIG
    PKG_CONFIG_ARGN
    PKG_CONFIG_EXECUTABLE
    PKG_CONFIG_LIBDIR
    PKG_CONFIG_PATH
    PKG_CONFIG_SYSROOT_DIR
    POWERSHELL_COMMAND
    PSModulePath
    BROWSER
    PYTHONBREAKPOINT
    PYTHONEXECUTABLE
    PYTHONINSPECT
    PYTHONOPTIMIZE
    PYTHONPLATLIBDIR
    PYTHONSTARTUP
    PYTHONWARNINGS
    PYENV_VERSION
    Python_EXECUTABLE
    Python_ROOT_DIR
    Python3_EXECUTABLE
    Python3_ROOT_DIR
    QCC_CONF_PATH
    SUNPRO_DEPENDENCIES
    VERBOSE
    VPATH
    VIRTUAL_ENV
    _CE_CONDA
    _CE_M
)

baby_robot_clear_untrusted_cmake_environment() {
    unset "${BABY_ROBOT_UNTRUSTED_CMAKE_ENVIRONMENT[@]}"
}

baby_robot_reject_unsafe_raw_environment() {
    local env_bin entry_name

    # Bash -p prevents function import into this process, but raw entries such
    # as BASH_FUNC_* and GNU make's dot-prefixed special-variable names can
    # survive into later descendants. Dot names cannot be represented or unset
    # as Bash variables, so reject every non-portable identifier before sourcing
    # build metadata.
    env_bin="$(type -P env || true)"
    case "${env_bin}" in
        /system/bin/env|/usr/bin/env|/bin/env) ;;
        *)
            echo "Error: env was not found in the trusted build PATH." >&2
            return 1
            ;;
    esac

    while IFS='=' read -r entry_name _; do
        case "${entry_name}" in
            BASH_FUNC_*)
                echo "Error: refusing an exported Bash function environment entry." >&2
                return 1
                ;;
        esac
        if [[ ! "${entry_name}" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]]; then
            echo "Error: refusing an environment entry with an unsafe name." >&2
            return 1
        fi
    done < <("${env_bin}")
}

baby_robot_normalize_trusted_prefix_path() {
    local raw_prefix_path="$1"
    local allowed_root="$2"
    local canonical_root canonical_prefix normalized=""
    local -a prefix_entries=()

    # CMake list separators and control characters must not be smuggled through
    # the environment-to-cache conversion below. Empty elements are also
    # rejected because they can select the current working directory.
    case "${raw_prefix_path}" in
        ''|:*|*:|*::*|*';'*|*$'\n'*|*$'\r'*)
            echo "Error: trusted CMAKE_PREFIX_PATH is empty or malformed." >&2
            return 1
            ;;
    esac
    case "${allowed_root}" in
        /*) ;;
        *)
            echo "Error: trusted prefix root must be absolute." >&2
            return 1
            ;;
    esac
    canonical_root="$(cd -- "${allowed_root}" 2>/dev/null && pwd -P)" || {
        echo "Error: trusted prefix root does not resolve to a directory." >&2
        return 1
    }

    IFS=':' read -r -a prefix_entries <<< "${raw_prefix_path}"
    for prefix in "${prefix_entries[@]}"; do
        case "${prefix}" in
            /*) ;;
            *)
                echo "Error: CMAKE_PREFIX_PATH contains a non-absolute entry." >&2
                return 1
                ;;
        esac
        canonical_prefix="$(cd -- "${prefix}" 2>/dev/null && pwd -P)" || {
            echo "Error: CMAKE_PREFIX_PATH entry does not resolve to a directory." >&2
            return 1
        }
        case "${canonical_prefix}" in
            "${canonical_root}"|"${canonical_root}"/*) ;;
            *)
                echo "Error: CMAKE_PREFIX_PATH escapes the trusted ROS installation." >&2
                return 1
                ;;
        esac
        if [ -n "${normalized}" ]; then
            normalized+=";"
        fi
        normalized+="${canonical_prefix}"
    done
    printf '%s\n' "${normalized}"
}

baby_robot_require_fresh_output_base() {
    local output_base="$1"
    if [ -L "${output_base}" ] || { [ -e "${output_base}" ] && [ ! -d "${output_base}" ]; }; then
        echo "Error: build output base is not a safe directory: ${output_base}" >&2
        return 1
    fi
    if [ -d "${output_base}" ] && \
       [ -n "$(find "${output_base}" -mindepth 1 -maxdepth 1 -print -quit)" ]; then
        # --cmake-force-configure retains security-sensitive cache entries.
        # Refuse every pre-existing artifact rather than trying to enumerate
        # all current and future CMake/colcon injection points.
        echo "Error: hardened builds require an empty output base: ${output_base}" >&2
        echo "Run ./clean.sh, review the workspace, and build again." >&2
        return 1
    fi
}
