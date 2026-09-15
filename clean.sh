#!/bin/bash -p

set -Eeuo pipefail

# No inherited search path may influence a destructive cleanup command.
export PATH="/system/bin:/usr/bin:/bin"

# Avoid command lookup before the repository root is known. Bash privileged
# mode also prevents inherited function exports/BASH_ENV from running first.
script_dir="${BASH_SOURCE[0]%/*}"
if [ "${script_dir}" = "${BASH_SOURCE[0]}" ]; then
    script_dir=.
fi
repo_root="$(cd -- "${script_dir}" && pwd -P)"
readonly repo_root

# Cleanup is destructive, so prove that the resolved directory is this project
# before constructing any deletion target.
if [ -z "${repo_root}" ] || [ "${repo_root}" = "/" ] || \
   [ ! -e "${repo_root}/.git" ] || [ ! -d "${repo_root}/src" ] || \
   [ ! -f "${repo_root}/build.sh" ]; then
    echo "Error: refusing to clean an unverified repository root: ${repo_root}" >&2
    exit 1
fi

readonly targets=(build install log logs)
RM_BIN="$(type -P rm || true)"
case "${RM_BIN}" in
    /system/bin/rm|/usr/bin/rm|/bin/rm) ;;
    *)
        echo "Error: rm was not found in the trusted cleanup PATH." >&2
        exit 1
        ;;
esac
readonly RM_BIN

for target_name in "${targets[@]}"; do
    target="${repo_root}/${target_name}"
    case "${target}" in
        "${repo_root}/build"|"${repo_root}/install"|"${repo_root}/log"|"${repo_root}/logs") ;;
        *)
        echo "Error: cleanup target escaped repository root: ${target}" >&2
        exit 1
            ;;
    esac
    "${RM_BIN}" -rf -- "${target}"
done
