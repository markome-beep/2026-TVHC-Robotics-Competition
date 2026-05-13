#!/usr/bin/env bash
# Internal helper: invokes `pio` with an environment scrubbed of every
# Nix/ROS leak that would corrupt micro_ros_platformio's nested host-side
# colcon/cmake bootstrap.
#
# Why this exists:
#   * CMake's default find_package() config-mode search treats every
#     directory on PATH as a candidate prefix (via CMAKE_SYSTEM_PREFIX_PATH).
#     The Nix dev shell puts a unified ros-env on PATH, so the nested build
#     would find /nix/store/...-ros-env/share/rosidl_typesupport_cpp and
#     fail with "No 'rosidl_typesupport_cpp' found" (because micro-ROS
#     intentionally ships only the microxrcedds typesupport).
#   * AMENT_PREFIX_PATH / CMAKE_PREFIX_PATH / COLCON_PREFIX_PATH likewise
#     leak the host ROS install into the nested build.
#
# We start from a clean env (`env -i`), preserve only what pio needs to
# work and reach the network, and filter ROS-flavoured entries out of PATH.
#
# Note: PIP_TARGET is intentionally NOT preserved here. The dev shell
# exports it to redirect pio's *own* read-only Nix-python plugin
# bootstrap, but inside this nested invocation pio runs through
# ~/.platformio/penv (writable, --system-site-packages). Forwarding
# PIP_TARGET would make pip re-route the nested rosidl bootstrap's
# `pip install` calls into the cache, where every package already
# exists, producing dozens of "Target directory already exists"
# warnings on every build. PYTHONPATH still carries the cache forward
# so the cached packages are importable; pip just sees them as
# already-satisfied and skips silently.

set -euo pipefail

# Filter out PATH entries pointing at any nix-store ROS env. The ros-env
# path varies by hash, so detect by the literal "-ros-env" suffix that
# nix-ros-overlay's buildEnv produces.
filter_path() {
    local out=""
    local IFS=":"
    for entry in $1; do
        case "$entry" in
            */nix/store/*-ros-env/bin) ;;
            */nix/store/*-ros-env) ;;
            *) out="${out:+$out:}$entry" ;;
        esac
    done
    printf '%s' "$out"
}

CLEAN_PATH="$(filter_path "$PATH")"

exec env -i \
    HOME="$HOME" \
    USER="${USER:-$(id -un)}" \
    PATH="$CLEAN_PATH" \
    TERM="${TERM:-dumb}" \
    LANG="${LANG:-C.UTF-8}" \
    PYTHONPATH="${PYTHONPATH:-}" \
    PIP_DISABLE_PIP_VERSION_CHECK="${PIP_DISABLE_PIP_VERSION_CHECK:-1}" \
    PIP_CONSTRAINT="${PIP_CONSTRAINT:-}" \
    PLATFORMIO_CORE_DIR="${PLATFORMIO_CORE_DIR:-$HOME/.platformio}" \
    HTTP_PROXY="${HTTP_PROXY:-}" \
    HTTPS_PROXY="${HTTPS_PROXY:-}" \
    NO_PROXY="${NO_PROXY:-}" \
    http_proxy="${http_proxy:-}" \
    https_proxy="${https_proxy:-}" \
    no_proxy="${no_proxy:-}" \
    NIX_SSL_CERT_FILE="${NIX_SSL_CERT_FILE:-}" \
    SSL_CERT_FILE="${SSL_CERT_FILE:-${NIX_SSL_CERT_FILE:-}}" \
    GIT_SSL_CAINFO="${GIT_SSL_CAINFO:-${NIX_SSL_CERT_FILE:-}}" \
    pio "$@"
