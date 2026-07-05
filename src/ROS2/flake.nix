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

          packages = [
            pkgs.colcon
            pkgs.patchShebangs

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
                  tf2-tools
                  tf2-ros
                  rviz2
                  rmw-fastrtps-cpp

                  # Pick at least one RMW implementation
                  # rmw-cyclonedds-cpp
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
