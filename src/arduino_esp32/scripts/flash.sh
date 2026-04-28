#!/usr/bin/env bash
# Flash the arduino_esp32 firmware to a connected ESP32 Feather V2.
#
# Usage:
#   ./scripts/flash.sh                     # uses /dev/ttyUSB0
#   ESP32_SERIAL_PORT=/dev/ttyACM0 ./scripts/flash.sh
#
# Requires `pio` on PATH (provided by `nix develop`).

set -euo pipefail

PORT="${ESP32_SERIAL_PORT:-/dev/ttyUSB0}"
ENV_NAME="featheresp32-v2"
PKG_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

if ! command -v pio >/dev/null 2>&1; then
    echo "error: 'pio' not found in PATH. Enter the dev shell: nix develop" >&2
    exit 1
fi

if [[ ! -e "$PORT" ]]; then
    echo "error: serial port '$PORT' does not exist." >&2
    echo "       Set ESP32_SERIAL_PORT to override (e.g. /dev/ttyACM0)." >&2
    exit 1
fi

echo "Flashing $ENV_NAME firmware to $PORT..."
exec "$PKG_DIR/scripts/pio_clean.sh" run -d "$PKG_DIR" -e "$ENV_NAME" -t upload --upload-port "$PORT"
