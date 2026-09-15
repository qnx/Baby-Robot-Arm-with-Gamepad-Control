#!/bin/bash

set -Eeuo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
# shellcheck disable=SC1091
. "${repo_root}/cmake/sanitize-build-environment.bash"

# Poison every forbidden input exactly as an inherited service/interactive
# environment could, then exercise the same helper called on both sides of the
# ROS setup script in build.sh.
for variable_name in "${BABY_ROBOT_UNTRUSTED_CMAKE_ENVIRONMENT[@]}"; do
    printf -v "${variable_name}" '%s' "/tmp/untrusted-${variable_name}"
    export "${variable_name}"
done

# The ROS overlay path is intentionally managed separately and must survive the
# post-setup CMake sanitization.
export CMAKE_PREFIX_PATH=/opt/ros/jazzy
baby_robot_clear_untrusted_cmake_environment

for variable_name in "${BABY_ROBOT_UNTRUSTED_CMAKE_ENVIRONMENT[@]}"; do
    if [[ -v "${variable_name}" ]]; then
        echo "forbidden build variable survived: ${variable_name}" >&2
        exit 1
    fi
done

if [ "${CMAKE_PREFIX_PATH}" != /opt/ros/jazzy ]; then
    echo "approved ROS CMAKE_PREFIX_PATH was unexpectedly cleared" >&2
    exit 1
fi

# MAKEFILES is consumed implicitly by GNU make and may contain top-level shell
# expansion. Demonstrate the threat with a harmless marker, then prove that the
# same inherited control cannot reach the build tool after sanitization.
make_bin="$(type -P make || true)"
if [ -n "${make_bin}" ]; then
    make_probe_root="$(mktemp -d /tmp/baby-robot-make-probe.XXXXXXXX)"
    make_probe_file="${make_probe_root}/inherited.mk"
    make_probe_marker="${make_probe_root}/executed"
    make_env_bin="$(type -P env)"

    assert_make_vector_sanitized() {
        local vector_name="$1"
        local vector_payload="$2"
        local fixture_file="$3"
        shift 3
        local -a extra_environment=("$@")

        rm -f -- "${make_probe_marker}"
        "${make_env_bin}" "${extra_environment[@]}" \
            "${vector_name}=${vector_payload}" \
            "${make_bin}" -f "${fixture_file}" >/dev/null 2>&1 || true
        if [ ! -e "${make_probe_marker}" ]; then
            echo "${vector_name} threat fixture did not execute as expected" >&2
            return 1
        fi

        rm -- "${make_probe_marker}"
        printf -v "${vector_name}" '%s' "${vector_payload}"
        export "${vector_name}"
        baby_robot_clear_untrusted_cmake_environment
        "${make_env_bin}" "${extra_environment[@]}" \
            "${make_bin}" -f "${fixture_file}" >/dev/null 2>&1 || true
        if [ -e "${make_probe_marker}" ]; then
            echo "${vector_name} survived build sanitization" >&2
            return 1
        fi
    }
    printf '$(shell touch %s)\n' "${make_probe_marker}" > "${make_probe_file}"
    MAKEFILES="${make_probe_file}" "${make_bin}" -f /dev/null >/dev/null 2>&1 || true
    if [ ! -e "${make_probe_marker}" ]; then
        echo "MAKEFILES threat fixture did not execute as expected" >&2
        rm -rf -- "${make_probe_root}"
        exit 1
    fi
    rm -- "${make_probe_marker}"
    export MAKEFILES="${make_probe_file}"
    baby_robot_clear_untrusted_cmake_environment
    "${make_bin}" -f /dev/null >/dev/null 2>&1 || true
    if [ -e "${make_probe_marker}" ]; then
        echo "MAKEFILES survived build sanitization" >&2
        rm -rf -- "${make_probe_root}"
        exit 1
    fi

    # GNUMAKEFLAGS accepts command-line syntax from the environment; --eval is
    # therefore another parse-time execution path even without MAKEFILES.
    GNUMAKEFLAGS="--eval=\$(shell touch ${make_probe_marker})" \
        "${make_bin}" -f /dev/null >/dev/null 2>&1 || true
    if [ ! -e "${make_probe_marker}" ]; then
        echo "GNUMAKEFLAGS threat fixture did not execute as expected" >&2
        rm -rf -- "${make_probe_root}"
        exit 1
    fi
    rm -- "${make_probe_marker}"
    export GNUMAKEFLAGS="--eval=\$(shell touch ${make_probe_marker})"
    baby_robot_clear_untrusted_cmake_environment
    "${make_bin}" -f /dev/null >/dev/null 2>&1 || true
    if [ -e "${make_probe_marker}" ]; then
        echo "GNUMAKEFLAGS survived build sanitization" >&2
        rm -rf -- "${make_probe_root}"
        exit 1
    fi

    # CMake-generated Unix Makefiles invoke $(MAKE) for recursive stages. An
    # inherited MAKE value can therefore replace the recursive build command.
    recursive_probe_file="${make_probe_root}/recursive.mk"
    printf 'all:\n\t@$(MAKE)\n' > "${recursive_probe_file}"
    (
        cd -- "${make_probe_root}"
        MAKE="touch ${make_probe_marker}" \
            "${make_bin}" -f "${recursive_probe_file}" >/dev/null 2>&1
    )
    if [ ! -e "${make_probe_marker}" ]; then
        echo "MAKE threat fixture did not execute as expected" >&2
        rm -rf -- "${make_probe_root}"
        exit 1
    fi
    rm -- "${make_probe_marker}"
    export MAKE="touch ${make_probe_marker}"
    baby_robot_clear_untrusted_cmake_environment
    (
        cd -- "${make_probe_root}"
        "${make_bin}" -f "${recursive_probe_file}" >/dev/null 2>&1 || true
    )
    if [ -e "${make_probe_marker}" ]; then
        echo "MAKE survived build sanitization" >&2
        rm -rf -- "${make_probe_root}"
        exit 1
    fi

    # Recursive make propagates MAKEOVERRIDES through MAKEFLAGS. A crafted
    # variable definition is expanded while the child parses its command line.
    recursive_override_file="${make_probe_root}/recursive-override.mk"
    printf 'all:\n\t@$(MAKE) -f /dev/null\n' > "${recursive_override_file}"
    makeoverrides_payload='EVIL=$$(shell touch '"${make_probe_marker}"')'
    MAKEOVERRIDES="${makeoverrides_payload}" \
        "${make_bin}" -f "${recursive_override_file}" >/dev/null 2>&1 || true
    if [ ! -e "${make_probe_marker}" ]; then
        echo "MAKEOVERRIDES threat fixture did not execute as expected" >&2
        rm -rf -- "${make_probe_root}"
        exit 1
    fi
    rm -- "${make_probe_marker}"
    export MAKEOVERRIDES="${makeoverrides_payload}"
    baby_robot_clear_untrusted_cmake_environment
    "${make_bin}" -f "${recursive_override_file}" >/dev/null 2>&1 || true
    if [ -e "${make_probe_marker}" ]; then
        echo "MAKEOVERRIDES survived build sanitization" >&2
        rm -rf -- "${make_probe_root}"
        exit 1
    fi

    # VPATH and GPATH expand make functions during prerequisite lookup, before
    # any recipe executes. Exercise each implicit search-path control.
    search_probe_file="${make_probe_root}/search.mk"
    printf 'all: missing-prerequisite\n\t@:\n' > "${search_probe_file}"
    make_search_payload='$(shell touch '"${make_probe_marker}"')'
    VPATH="${make_search_payload}" \
        "${make_bin}" -f "${search_probe_file}" >/dev/null 2>&1 || true
    if [ ! -e "${make_probe_marker}" ]; then
        echo "VPATH threat fixture did not execute as expected" >&2
        rm -rf -- "${make_probe_root}"
        exit 1
    fi
    rm -- "${make_probe_marker}"
    export VPATH="${make_search_payload}"
    baby_robot_clear_untrusted_cmake_environment
    "${make_bin}" -f "${search_probe_file}" >/dev/null 2>&1 || true
    if [ -e "${make_probe_marker}" ]; then
        echo "VPATH survived build sanitization" >&2
        rm -rf -- "${make_probe_root}"
        exit 1
    fi

    GPATH="${make_search_payload}" VPATH=. \
        "${make_bin}" -f "${search_probe_file}" >/dev/null 2>&1 || true
    if [ ! -e "${make_probe_marker}" ]; then
        echo "GPATH threat fixture did not execute as expected" >&2
        rm -rf -- "${make_probe_root}"
        exit 1
    fi
    rm -- "${make_probe_marker}"
    export GPATH="${make_search_payload}"
    baby_robot_clear_untrusted_cmake_environment
    VPATH=. "${make_bin}" -f "${search_probe_file}" >/dev/null 2>&1 || true
    if [ -e "${make_probe_marker}" ]; then
        echo "GPATH survived build sanitization" >&2
        rm -rf -- "${make_probe_root}"
        exit 1
    fi

    # These ordinary environment names become GNU make variables. CMake's
    # generated Unix Makefiles expand each one in recursive or recipe paths.
    make_command_fixture="${make_probe_root}/make-command.mk"
    printf 'all:\n\t@$(MAKE) -f /dev/null\n' > "${make_command_fixture}"
    assert_make_vector_sanitized \
        MAKE_COMMAND '$(shell touch '"${make_probe_marker}"')' \
        "${make_command_fixture}"

    verbose_fixture="${make_probe_root}/verbose.mk"
    printf '$(VERBOSE)MAKESILENT = -s\n$(VERBOSE).SILENT:\nall:\n\t@:\n' \
        > "${verbose_fixture}"
    assert_make_vector_sanitized \
        VERBOSE '$(shell touch '"${make_probe_marker}"')' \
        "${verbose_fixture}"

    color_fixture="${make_probe_root}/color.mk"
    printf 'all:\n\t@: --switch=$(COLOR)\n' > "${color_fixture}"
    assert_make_vector_sanitized \
        COLOR '$(shell touch '"${make_probe_marker}"')' \
        "${color_fixture}"

    makesilent_fixture="${make_probe_root}/makesilent.mk"
    printf 'all:\n\t@: $(MAKESILENT)\n' > "${makesilent_fixture}"
    assert_make_vector_sanitized \
        MAKESILENT '$(shell touch '"${make_probe_marker}"')' \
        "${makesilent_fixture}" VERBOSE=1
    rm -rf -- "${make_probe_root}"
fi

# GCC_COMPARE_DEBUG accepts compiler options for a comparison pass. Prove that
# an inherited plugin selector reaches GCC before sanitization and disappears
# before the build compiler is invoked.
cc_bin="$(type -P cc || true)"
if [ -n "${cc_bin}" ]; then
    gcc_probe_root="$(mktemp -d /tmp/baby-robot-gcc-probe.XXXXXXXX)"
    gcc_probe_plugin="${gcc_probe_root}/untrusted-plugin.so"
    gcc_probe_output="$(
        GCC_COMPARE_DEBUG="-fplugin=${gcc_probe_plugin}" \
            "${cc_bin}" -x c -c /dev/null -o "${gcc_probe_root}/unsafe.o" 2>&1 || true
    )"
    if [[ "${gcc_probe_output}" != *"${gcc_probe_plugin}"* ]]; then
        echo "GCC_COMPARE_DEBUG threat fixture did not reach GCC as expected" >&2
        rm -rf -- "${gcc_probe_root}"
        exit 1
    fi
    export GCC_COMPARE_DEBUG="-fplugin=${gcc_probe_plugin}"
    baby_robot_clear_untrusted_cmake_environment
    gcc_probe_output="$(
        "${cc_bin}" -x c -c /dev/null -o "${gcc_probe_root}/safe.o" 2>&1 || true
    )"
    if [[ "${gcc_probe_output}" == *"${gcc_probe_plugin}"* ]]; then
        echo "GCC_COMPARE_DEBUG survived build sanitization" >&2
        rm -rf -- "${gcc_probe_root}"
        exit 1
    fi
    rm -rf -- "${gcc_probe_root}"
fi

# Privileged Bash suppresses import into itself, but must not pass the raw
# exported-function entry to a later ordinary Bash process.
bash_bin="$(type -P bash)"
if env 'BASH_FUNC_baby_robot_probe%%=() { echo UNSAFE_CHILD_IMPORT; }' \
    "${bash_bin}" -p -c \
    'source "$1"; export PATH=/system/bin:/usr/bin:/bin; baby_robot_reject_unsafe_raw_environment' \
    _ "${repo_root}/cmake/sanitize-build-environment.bash" 2>/dev/null; then
    echo "raw exported Bash function was accepted by build sanitization" >&2
    exit 1
fi

# GNU make recognizes dot-prefixed special-variable names that Bash cannot
# import or unset. Reject all raw non-identifier names at the process boundary.
for unsafe_entry_name in .SHELLFLAGS .EXTRA_PREREQS .LIBPATTERNS; do
    if env "${unsafe_entry_name}=untrusted" \
        "${bash_bin}" -p -c \
        'source "$1"; export PATH=/system/bin:/usr/bin:/bin; baby_robot_reject_unsafe_raw_environment' \
        _ "${repo_root}/cmake/sanitize-build-environment.bash" 2>/dev/null; then
        echo "unsafe raw environment name was accepted: ${unsafe_entry_name}" >&2
        exit 1
    fi
done

unsafe_launcher_output="$(
    env '.SHELLFLAGS=untrusted' "${repo_root}/target_scripts/start_robot.sh" 2>&1 || true
)"
if [[ "${unsafe_launcher_output}" != *"environment entry with an unsafe name"* ]]; then
    echo "launcher did not reject an unsafe raw environment name" >&2
    exit 1
fi

launcher_output="$(
    env 'BASH_FUNC_baby_robot_probe%%=() { echo UNSAFE_CHILD_IMPORT; }' \
        "${repo_root}/target_scripts/start_robot.sh" 2>&1 || true
)"
if [[ "${launcher_output}" != *"refusing an exported Bash function environment entry"* ]]; then
    echo "launcher did not reject the raw exported Bash function entry" >&2
    exit 1
fi
if [[ "${launcher_output}" == *"UNSAFE_CHILD_IMPORT"* ]]; then
    echo "launcher imported the hostile Bash function" >&2
    exit 1
fi

fixture_root="$(mktemp -d /tmp/baby-robot-build-guard.XXXXXXXX)"
cleanup_fixture() {
    rm -rf -- "${fixture_root}"
}
trap cleanup_fixture EXIT

# CPython resolves dotted warning categories during interpreter startup.  That
# import can reach webbrowser and execute an inherited BROWSER command before
# the requested program begins, so prove both the attack precondition and the
# sanitizer that protects generated ROS setup helpers.
python_bin="$(type -P python3 || true)"
if [ -n "${python_bin}" ]; then
    python_startup_marker="${fixture_root}/python-startup-executed"
    python_browser_probe="${fixture_root}/browser-probe"
    printf '#!/bin/sh\n: > "%s"\n' "${python_startup_marker}" > "${python_browser_probe}"
    chmod 700 "${python_browser_probe}"

    env PYTHONWARNINGS='ignore::antigravity.Geohash' \
        BROWSER="${python_browser_probe}" \
        "${python_bin}" -c pass >/dev/null 2>&1 || true
    if [ ! -e "${python_startup_marker}" ]; then
        echo "PYTHONWARNINGS/BROWSER threat fixture did not execute as expected" >&2
        exit 1
    fi

    rm -- "${python_startup_marker}"
    export PYTHONWARNINGS='ignore::antigravity.Geohash'
    export BROWSER="${python_browser_probe}"
    baby_robot_clear_untrusted_cmake_environment
    "${python_bin}" -c pass >/dev/null 2>&1 || true
    if [ -e "${python_startup_marker}" ]; then
        echo "Python startup command controls survived build sanitization" >&2
        exit 1
    fi
fi

# The build evaluates Python-backed ROS setup hooks before its final env -i,
# and the launcher starts a Python node after two setup files.  Reassert the
# controls at each executable-metadata boundary, not only in the final child.
if [ "$(grep -Fc 'PYTHONWARNINGS PYTHONBREAKPOINT PYTHONOPTIMIZE BROWSER' \
        "${repo_root}/build.sh")" -lt 2 ]; then
    echo "build does not clear Python startup controls on both sides of setup" >&2
    exit 1
fi
if [ "$(grep -Fc 'PYTHONWARNINGS PYTHONBREAKPOINT PYTHONOPTIMIZE BROWSER' \
        "${repo_root}/target_scripts/start_robot.sh")" -lt 4 ]; then
    echo "launcher does not clear Python startup controls at every setup boundary" >&2
    exit 1
fi

# Direct toolchain configuration must not reuse a preseeded host-interpreter
# cache or ambient search path. QCC's configuration selector is also cleared
# before configure-time compiler probes; the supported build additionally
# isolates the subsequent build command with env -i.
for toolchain_guard in \
    'unset(ENV{QCC_CONF_PATH})' \
    'unset(QNX_HOST_PYTHON_EXECUTABLE CACHE)' \
    'NO_DEFAULT_PATH'; do
    if ! grep -Fq -- "${toolchain_guard}" \
        "${repo_root}/platform/qnx.nto.toolchain.cmake"; then
        echo "QNX toolchain guard is missing: ${toolchain_guard}" >&2
        exit 1
    fi
done

# Model the interpreter selectors consumed by generated ROS/colcon setup
# templates. Sanitization must remove both before a setup file can execute an
# attacker-selected program.
setup_marker="${fixture_root}/setup-interpreter-ran"
setup_interpreter="${fixture_root}/untrusted-python"
printf '#!/bin/sh\ntouch %s\n' "${setup_marker}" > "${setup_interpreter}"
chmod 700 "${setup_interpreter}"
export AMENT_PYTHON_EXECUTABLE="${setup_interpreter}"
export COLCON_PYTHON_EXECUTABLE="${setup_interpreter}"
baby_robot_clear_untrusted_cmake_environment
for selector in AMENT_PYTHON_EXECUTABLE COLCON_PYTHON_EXECUTABLE; do
    if [[ -v "${selector}" ]]; then
        "${!selector}"
    fi
done
if [ -e "${setup_marker}" ]; then
    echo "setup interpreter selector survived sanitization" >&2
    exit 1
fi

unset COLCON_CURRENT_PREFIX
set +u
# shellcheck disable=SC1091
. "${repo_root}/test/fixtures/generated-prefix.bash"
set -Eeuo pipefail
if [ "${generated_prefix_probe}" != "" ]; then
    echo "generated setup fixture saw an unexpected inherited prefix" >&2
    exit 1
fi
# Confirm strict nounset was restored after the narrow setup window.
if ( : "${BABY_ROBOT_MUST_REMAIN_UNSET}" ) 2>/dev/null; then
    echo "nounset was not restored after generated setup fixture" >&2
    exit 1
fi

empty_base="${fixture_root}/empty"
mkdir "${empty_base}"
baby_robot_require_fresh_output_base "${empty_base}"

poisoned_base="${fixture_root}/poisoned"
mkdir "${poisoned_base}"
touch "${poisoned_base}/CMakeCache.txt"
if baby_robot_require_fresh_output_base "${poisoned_base}" 2>/dev/null; then
    echo "poisoned CMake cache was accepted" >&2
    exit 1
fi

trusted_prefix_root="${fixture_root}/trusted-prefix-root"
mkdir -p "${trusted_prefix_root}/one" "${trusted_prefix_root}/two"
normalized_prefixes="$(baby_robot_normalize_trusted_prefix_path \
    "${trusted_prefix_root}/one:${trusted_prefix_root}/two" \
    "${trusted_prefix_root}")"
if [ "${normalized_prefixes}" != \
    "${trusted_prefix_root}/one;${trusted_prefix_root}/two" ]; then
    echo "trusted CMake prefix conversion produced an unexpected cache list" >&2
    exit 1
fi
if baby_robot_normalize_trusted_prefix_path \
    "${trusted_prefix_root}/one:/tmp" "${trusted_prefix_root}" >/dev/null 2>&1; then
    echo "CMake prefix conversion accepted a path outside its trusted root" >&2
    exit 1
fi

# PackageName_ROOT is open-ended and cannot be safely enumerated. The hardened
# colcon invocation must tell every find_package call to ignore that class.
if ! grep -Fq -- '-DCMAKE_FIND_USE_PACKAGE_ROOT_PATH=FALSE' "${repo_root}/build.sh"; then
    echo "build does not disable ambient package-root hints" >&2
    exit 1
fi

# PackageName_DIR is likewise open-ended and CMake consumes it from the
# process environment unless this search category is disabled globally.
if ! grep -Fq -- '-DCMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH=FALSE' "${repo_root}/build.sh"; then
    echo "build does not disable ambient CMake package/config environment paths" >&2
    exit 1
fi

if ! grep -Fq -- '-DCMAKE_PREFIX_PATH="${TRUSTED_CMAKE_PREFIX_PATH}"' "${repo_root}/build.sh"; then
    echo "build does not pass the trusted ROS prefix as an explicit CMake cache value" >&2
    exit 1
fi

for python_policy in \
    '-DPython_FIND_VIRTUALENV=STANDARD' \
    '-DPython3_FIND_VIRTUALENV=STANDARD'; do
    if ! grep -Fq -- "${python_policy}" "${repo_root}/build.sh"; then
        echo "build does not disable active-virtual-environment precedence: ${python_policy}" >&2
        exit 1
    fi
done

# The final build process must receive only the explicit positive allowlist;
# this contains future tool-specific variables even when the denylist has not
# learned their names yet.
if ! grep -Fq -- '"${TRUSTED_ENV_BIN}" -i "${TRUSTED_BUILD_ENVIRONMENT[@]}"' \
    "${repo_root}/build.sh"; then
    echo "colcon is not launched through the isolated build environment" >&2
    exit 1
fi

# Prove the disabled environment search still permits an explicit trusted
# prefix and does not load a same-named package config from the ambient path.
cmake_bin="$(type -P cmake || true)"
if [ -n "${cmake_bin}" ]; then
    env_bin="$(type -P env)"
    "${env_bin}" \
        CMAKE_PREFIX_PATH="${repo_root}/test/fixtures/cmake-prefix/poison" \
        "${cmake_bin}" \
        -S "${repo_root}/test/fixtures/cmake-prefix/project" \
        -B "${fixture_root}/cmake-prefix-build" \
        -DCMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH=FALSE \
        -DCMAKE_PREFIX_PATH="${repo_root}/test/fixtures/cmake-prefix/trusted" \
        >/dev/null
else
    echo "CMake unavailable; explicit-prefix integration fixture skipped" >&2
fi

echo "build environment sanitization tests passed"
