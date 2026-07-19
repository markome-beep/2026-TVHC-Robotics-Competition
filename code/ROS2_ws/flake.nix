{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/develop";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
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
          name = "ROS2";

          packages = with pkgs; [
            colcon

            # Rust pkgs
            cargo
            rustc
            rustfmt
            clippy

            # ESP32 / micro-ROS firmware build (arduino_esp32 package)
            platformio-core
            esptool
            python3

            cargo-ament-build # provides `cargo ament-build`

            # colcon extensions for cargo
            python3Packages.colcon-cargo
            python3Packages.colcon-ros-cargo

            (
              with rosPackages.jazzy;
              buildEnv {
                paths = [
                  ament-cmake-core
                  ros-core
                  fastcdr
                  spdlog

                  # rclrs links against these by default
                  rcl
                  rcl-action
                  rcl-lifecycle
                  rcl-interfaces

                  rmw
                  rmw-implementation
                  rmw-fastrtps-cpp

                  rosidl-runtime-c
                  rosidl-typesupport-c

                  # Message packages rclrs hard-links against
                  example-interfaces
                  builtin-interfaces
                  test-msgs
                  action-msgs
                  rosgraph-msgs
                  unique-identifier-msgs
                  std-msgs
                  geometry-msgs
                  sensor-msgs
                  micro-ros-msgs

                  tf2-tools
                  tf2-ros
                  rviz2
                ];
              }
            )
          ];
        };
      }
    );
}
