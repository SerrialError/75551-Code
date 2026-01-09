{
  description = "C++ dev shell";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = { self, nixpkgs, ... }:
    let
      pkgs = import nixpkgs {
        system = "x86_64-linux";
      };
    in {
      devShells.x86_64-linux.default = pkgs.mkShell {
        buildInputs = [
          pkgs.gcc
          pkgs.eigen
        ];

        # Make the Eigen include visible to g++
        shellHook = ''
          # prepend eigen include path if not already present
          if [ -n "$CPATH" ]; then
            export CPATH='${pkgs.eigen}/include/eigen3':$CPATH
          else
            export CPATH='${pkgs.eigen}/include/eigen3'
          fi

          if [ -n "$CPLUS_INCLUDE_PATH" ]; then
            export CPLUS_INCLUDE_PATH='${pkgs.eigen}/include/eigen3':$CPLUS_INCLUDE_PATH
          else
            export CPLUS_INCLUDE_PATH='${pkgs.eigen}/include/eigen3'
          fi
        '';
      };
    };
}
