{
  inputs = {
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
  };
  outputs =
    {
      self,
      nix-ros-overlay,
      nixpkgs,
    }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
      in
      {
        devShells.default = pkgs.mkShell {
          name = "Example project";

          # micro_ros_platformio's PlatformIO build script does two things
          # that fight the read-only Nix Python:
          #   1. pio runs `python -m pip install <deps>` using its own bundled
          #      (read-only) interpreter. We redirect those installs to a
          #      writable per-user dir and put it on PYTHONPATH so subsequent
          #      pio subprocesses can import them.
          #   2. micro_ros_platformio itself sources
          #      ~/.platformio/penv/bin/activate to build the host-side
          #      micro-ROS dev deps. The Nix build of platformio-core disables
          #      auto-creation of that venv, so we provision it ourselves.
          #
          # Additionally:
          #   * PIP_CONSTRAINT pins setuptools<80 in the writable cache. Pio's
          #     own bootstrap otherwise pulls setuptools 82, which conflicts
          #     with colcon-core's `setuptools<80,>=30.3.0` requirement and
          #     makes pip's resolver complain on every build.
          #   * AMENT_PREFIX_PATH is supplemented with a writable shim
          #     containing empty `local_setup.{sh,bash,zsh}` files. The Nix
          #     ros-env on AMENT_PREFIX_PATH has no such files (it's a flat
          #     sysroot, not an ament install layout), so colcon-ros warns on
          #     every build. Prepending the shim — whose stubs satisfy the
          #     check — silences the warning while leaving the original entry
          #     in place for runtime ros2 lookups.
          shellHook = ''
            export PIO_PYLIB="$HOME/.cache/test-ros-humble-pio-pylib"
            mkdir -p "$PIO_PYLIB"
            export PIP_TARGET="$PIO_PYLIB"
            export PIP_DISABLE_PIP_VERSION_CHECK=1
            export PYTHONPATH="$PIO_PYLIB''${PYTHONPATH:+:$PYTHONPATH}"

            export PIO_PIP_CONSTRAINT="$PIO_PYLIB/.constraints.txt"
            printf 'setuptools<80\n' > "$PIO_PIP_CONSTRAINT"
            export PIP_CONSTRAINT="$PIO_PIP_CONSTRAINT"

            export PLATFORMIO_PENV="$HOME/.platformio/penv"
            if [ ! -x "$PLATFORMIO_PENV/bin/python" ]; then
              echo "[flake] creating PlatformIO penv at $PLATFORMIO_PENV"
              mkdir -p "$HOME/.platformio"
              ${pkgs.python3}/bin/python3 -m venv --system-site-packages "$PLATFORMIO_PENV"
              # Disable PIP_TARGET inside the venv so pip can install normally.
              PIP_TARGET= "$PLATFORMIO_PENV/bin/python" -m pip install --upgrade pip >/dev/null
            fi

            # Suppress colcon-ros's "no local_setup.*" warning about the
            # Nix ros-env entry on AMENT_PREFIX_PATH. nix-ros-overlay's
            # `buildEnv` is a flat sysroot, not an ament install layout,
            # so it has no `local_setup.*` and colcon-ros warns per-entry
            # on every build. We can't write into the (read-only) Nix
            # store to add stubs, and we can't satisfy colcon-ros with a
            # *separate* shim entry (it checks each entry independently),
            # so we strip the ros-env entry from AMENT_PREFIX_PATH for
            # the duration of the `colcon` invocation only. Interactive
            # ros2 tooling continues to see the full path.
            colcon() {
              local clean_app="" entry IFS=":"
              for entry in ''${AMENT_PREFIX_PATH:-}; do
                case "$entry" in
                  */nix/store/*-ros-env) ;;
                  *) clean_app="''${clean_app:+$clean_app:}$entry" ;;
                esac
              done
              AMENT_PREFIX_PATH="$clean_app" \
                CMAKE_PREFIX_PATH="$(_filter_existing "''${CMAKE_PREFIX_PATH:-}")" \
                command colcon "$@"
            }
            _filter_existing() {
              local out="" entry IFS=":"
              for entry in $1; do
                [ -e "$entry" ] && out="''${out:+$out:}$entry"
              done
              printf '%s' "$out"
            }
            export -f colcon _filter_existing
          '';

          packages = [
            pkgs.colcon

            # Rust pkgs
            pkgs.cargo
            pkgs.rustc
            pkgs.rustfmt
            pkgs.clippy

            # ESP32 / micro-ROS firmware build (arduino_esp32 package)
            pkgs.platformio-core
            pkgs.esptool
            pkgs.python3

            pkgs.cargo-ament-build # provides `cargo ament-build`
            # colcon extensions for cargo
            pkgs.python3Packages.colcon-cargo
            pkgs.python3Packages.colcon-ros-cargo

            # ... other non-ROS packages
            (
              with pkgs.rosPackages.humble;
              buildEnv {
                paths = [
                  ament-cmake-core
                  ros-core
                  # rclrs links against these by default
                  rcl
                  rcl-action
                  rcl-lifecycle
                  rmw
                  rmw-implementation
                  rosidl-runtime-c
                  rosidl-typesupport-c

                  # Message packages rclrs hard-links against
                  action-msgs
                  builtin-interfaces
                  example-interfaces
                  rcl-interfaces
                  rosgraph-msgs
                  test-msgs
                  unique-identifier-msgs
                  std-msgs
                  geometry-msgs
                  sensor-msgs

                  # Pick at least one RMW implementation
                  rmw-cyclonedds-cpp
                  # or: rmw-fastrtps-cpp
                  # ... other ROS packages
                ];
              }
            )
          ];
        };
      }
    );

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
