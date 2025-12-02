{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
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
        gazebo-overlay = final: prev: {
          boost = prev.boost186;
        };
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            nix-ros-overlay.overlays.default
            gazebo-overlay
          ];
          config.permittedInsecurePackages = [
            "freeimage-3.18.0-unstable-2024-04-18"
          ];
        };
      in
      {
        devShells.default = pkgs.mkShell {
          name = "Example project";
          QT_QPA_PLATFORM = "xcb";
          TURTLEBOT3_MODEL = "burger";

          shellHook = ''
            source ./install/local_setup.bash
          '';

          packages = [
            pkgs.colcon
            pkgs.python3Packages.sympy
            pkgs.python3Packages.numpy
            pkgs.python3Packages.matplotlib
            # ... other non-ROS packages
            (
              with pkgs.rosPackages.humble;
              buildEnv {
                paths = with pkgs.rosPackages.humble; [
                  ros-base
                  ros-core
                  desktop
                  gazebo-ros
                  gazebo-ros-pkgs
                  rviz2
                  geometry-msgs
                  turtlebot3-msgs
                  turtlebot3-teleop
                  nav-msgs
                  rclcpp
                  sensor-msgs
                  plotjuggler
                  # plotjuggler-ros
                  tf2
                  tf-transformations
                  ament-cmake
                  ament-cmake-core
                  python-cmake-module
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
