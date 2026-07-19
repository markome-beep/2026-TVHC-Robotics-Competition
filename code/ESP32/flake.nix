{
  description = "ESP32 Development Environment";

  inputs = {
    nixpkgs-esp-dev.url = "github:mirrexagon/nixpkgs-esp-dev";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
  };

  outputs =
    {
      self,
      nixpkgs,
      nixpkgs-esp-dev,
      nix-ros-overlay,
    }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs {
        inherit system;
        overlays = [ nix-ros-overlay.overlays.default ];
      };

      esp = nixpkgs-esp-dev.packages.${system};
    in
    {
      devShells.${system}.default = pkgs.mkShell {
        buildInputs = [
          esp.esp-idf-full

          pkgs.colcon
          pkgs.python314
          pkgs.python314Packages.catkin-pkg
          pkgs.python314Packages.lark
          pkgs.python314Packages.empy
        ];

        shellHook = ''
          echo "ESP-IDF environment loaded."
          echo "You can now use idf.py"
        '';
      };
    };
}
