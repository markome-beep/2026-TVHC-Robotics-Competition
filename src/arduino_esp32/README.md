# arduino_esp32

micro-ROS firmware for the **Adafruit ESP32 Feather V2**, built with
PlatformIO + the Arduino framework and integrated into the colcon workspace.

The firmware:

- publishes `std_msgs/Int32` on `/esp32/counter` at ~1 Hz, and
- scaffolds an I2C read path (`Wire.h`) in
  [`src/i2c_sensor.cpp`](src/i2c_sensor.cpp) — currently a stub returning a
  monotonic counter, marked with `TODO`s for swapping in a real device.

## Prerequisites

Everything is provisioned by the workspace flake. Enter the dev shell first:

```sh
nix develop   # or rely on direnv
```

This puts `pio`, `esptool`, `colcon`, and the ROS toolchain on PATH.

> **First build only:** PlatformIO downloads the Xtensa toolchain and the
> precompiled `micro_ros_platformio` library into `~/.platformio` (outside
> the Nix store). Network access is required on the first build.

## Build

From the workspace root:

```sh
colcon build --packages-select arduino_esp32
```

This invokes PlatformIO via [`scripts/pio_clean.sh`](scripts/pio_clean.sh)
(a wrapper that strips Nix/ROS environment leaks which would otherwise
break micro_ros_platformio's nested host-side build) and produces
`firmware.bin`. The artifact is installed into:

```
install/arduino_esp32/share/arduino_esp32/firmware/firmware.bin
```

### Build environment notes

The flake's `shellHook` does a few things to keep `colcon build` quiet:

- Sets `PIP_TARGET=$HOME/.cache/test-ros-humble-pio-pylib` and adds it
  to `PYTHONPATH` so PlatformIO's first-time bootstrap of its own
  plugin deps lands somewhere writable (the Nix Python is read-only).
- Pins `setuptools<80` via `PIP_CONSTRAINT` so the cache can't pull a
  newer setuptools that would conflict with `colcon-core`.
- Creates `~/.platformio/penv` (used by `micro_ros_platformio` to
  bootstrap its host-side rosidl deps).
- Prepends — only for the duration of each `colcon` invocation — a
  filtered `AMENT_PREFIX_PATH` with the Nix ros-env entry removed.
  That entry is a flat sysroot with no `local_setup.*` files (Nix
  ros-overlay's `buildEnv` doesn't generate them), which makes
  colcon-ros warn on every build. Interactive `ros2` tooling still
  sees the full path. This is implemented as a `colcon()` shell
  function exported from the flake's `shellHook`.

If PlatformIO ever needs to install a *new* plugin (e.g. after a
platform upgrade) and complains about a read-only filesystem, run it
once with `PIP_TARGET` re-exported:

```sh
PIP_TARGET="$PIO_PYLIB" pio pkg install -g -p espressif32
```

## Flash

Flashing is **not** wired into `colcon build` — it's a separate step. Two
equivalent options:

```sh
# 1. Convenience script (recommended)
./src/arduino_esp32/scripts/flash.sh

# 2. CMake target produced by colcon
cmake --build build/arduino_esp32 --target flash
```

Both honor the `ESP32_SERIAL_PORT` env var (default `/dev/ttyUSB0`):

```sh
ESP32_SERIAL_PORT=/dev/ttyACM0 ./src/arduino_esp32/scripts/flash.sh
```

## Run

The micro-ROS Arduino transport is serial. On the host, run the agent:

```sh
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

(The agent itself is not part of this package — run it via Docker or a
separate install. See <https://micro.ros.org/docs/tutorials/core/overview/>.)

Then, in another shell:

```sh
source install/setup.bash
ros2 topic echo /esp32/angle
```

## ROS interface

The firmware exposes one publisher and one subscriber:

| Direction  | Topic           | Type                | Notes                                              |
| ---------- | --------------- | ------------------- | -------------------------------------------------- |
| publish    | `/esp32/angle`  | `std_msgs/Float32`  | AS5600 raw shaft angle in degrees (0..360), 100 Hz |
| subscribe  | `/esp32/state`  | `std_msgs/UInt8`    | drives the onboard NeoPixel pattern                |

NeoPixel state table (see [`src/neopixel.cpp`](src/neopixel.cpp) to
extend):

| Value | Pattern                  | Suggested meaning |
| ----- | ------------------------ | ----------------- |
| `0`   | off                      | idle              |
| `1`   | solid green              | ok                |
| `2`   | yellow blink (~2 Hz)     | warn              |
| `3`   | red blink (~5 Hz)        | error             |
| `4`   | rainbow cycle            | busy / demo       |
| other | off (clamped)            | —                 |

Cycle through states from the host:

```sh
for v in 1 2 3 4 0; do
  ros2 topic pub --once /esp32/state std_msgs/msg/UInt8 "{data: $v}"
  sleep 2
done
```

## I2C wiring (AS5600 magnetic encoder)

`i2c_sensor::begin()` initialises `Wire` on the Feather V2's default
Stemma QT pins (SDA = GPIO22, SCL = GPIO20) and configures an Adafruit
AS5600 12-bit contactless rotary position sensor at I2C address `0x36`.

Plug-and-play with a Stemma QT cable. A small diametrically magnetised
magnet must be mounted ~0.5–3 mm above the chip's centre; without one,
`begin()` prints `[as5600] no magnet detected; check alignment` on the
USB serial console and `read_sensor()` returns the chip's last (junk)
reading. If the sensor itself can't be reached on the bus,
`read_sensor()` returns `NaN`.

The chip is configured for the most aggressive response:

- `POWER_MODE = NOM` (always-on internal sampling, ~150 µs period)
- `HYSTERESIS = OFF` (no output dead-band)
- `SLOW_FILTER = 2x` (lightest steady-state filtering)
- `FAST_FILTER_THRESH = 6 LSB` (smallest motion bypasses the slow filter)
- watchdog disabled (no auto-LPM3 fallback)

See [`src/i2c_sensor.cpp`](src/i2c_sensor.cpp) for the register
writes and to retune.

## Editor / LSP (clangd)

**No editor configuration changes are required.** Every
`colcon build --packages-select arduino_esp32` runs PlatformIO's
`compiledb` target, which writes `compile_commands.json` into this
package directory. clangd discovers it automatically via its default
upward search.

A repo-local [`.clangd`](.clangd) does the rest:

- adds the Xtensa GCC system include paths (`libstdc++`, `libc`,
  GCC builtins, `sys-include`) so headers like `machine/endian.h`,
  Arduino's `Stream.h`, and `<string>` resolve correctly without
  needing clangd's `--query-driver` flag;
- strips GCC/Xtensa-only flags clang doesn't understand
  (`-mlongcalls`, `-mfix-esp32-psram-cache-issue`, …);
- suppresses diagnostics under `.pio/` (third-party sources).

Open `src/main.cpp` or `src/i2c_sensor.cpp` after a build and clangd
should report no errors.

### Caveats

- The paths in `.clangd` are absolute (clangd does not expand `~` or
  env vars in `.clangd`). They assume PlatformIO's default install
  location (`$HOME/.platformio`).
- The Xtensa GCC version (`8.4.0`) is pinned by the `espressif32`
  PlatformIO platform. If you ever bump that platform, update the
  version fragments in `.clangd` to match the new
  `~/.platformio/packages/toolchain-xtensa-esp32/lib/gcc/xtensa-esp32-elf/<ver>`
  directory.
- Requires clangd ≥ 14.

`compile_commands.json` is a build artifact and shouldn't be committed
if you put this repo under git.

### Optional: use clangd's `--query-driver` instead

If you'd rather have clangd ask the cross compiler itself for its
include paths (no version pinning in `.clangd`), launch clangd with
`--query-driver=` allowlisting the Xtensa toolchain. This **must** be
set in editor config — clangd refuses the flag from `.clangd` for
security reasons.

If you go this route, also delete the seven `-isystem ...` entries from
`.clangd` so clangd uses only the cross-compiler's reported paths.

**Plain `nvim-lspconfig`:**

```lua
require('lspconfig').clangd.setup({
  cmd = {
    'clangd',
    '--query-driver=' .. vim.env.HOME ..
      '/.platformio/packages/toolchain-xtensa-esp32/bin/xtensa-esp32-elf-*',
    '--background-index',
    '--clang-tidy',
  },
})
```

**[nvf](https://notashelf.github.io/nvf/) (nix-configured neovim):**

nvf wires LSP servers through `vim.languages` / `vim.lsp.servers`. Add
clangd's command line either by setting `cmd` on the server or via
`vim.lsp.lspconfig.sources` for finer control. A minimal example:

```nix
{
  programs.nvf.settings.vim = {
    languages.clang = {
      enable = true;
      lsp.enable = true;
      # Override the default clangd cmd to allowlist the Xtensa driver.
      lsp.server = "clangd";
      lsp.opts = ''
        cmd = {
          "clangd",
          "--query-driver=" .. vim.env.HOME
            .. "/.platformio/packages/toolchain-xtensa-esp32/bin/xtensa-esp32-elf-*",
          "--background-index",
          "--clang-tidy",
        },
      '';
    };
  };
}
```

If your nvf version exposes the option as `vim.lsp.servers.clangd.cmd`
instead, use:

```nix
{
  programs.nvf.settings.vim.lsp.servers.clangd = {
    cmd = [
      "clangd"
      "--query-driver=/home/markop1/.platformio/packages/toolchain-xtensa-esp32/bin/xtensa-esp32-elf-*"
      "--background-index"
      "--clang-tidy"
    ];
  };
}
```

(Use the absolute home path here — Nix evaluates this file at build
time, before `vim.env.HOME` exists.)
