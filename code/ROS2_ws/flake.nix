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
          name = "ROS2";

          postPatch = ''
            patchShebangs .
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
