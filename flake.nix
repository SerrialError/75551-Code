{
  description = "PROS flake with dev shell and pros-cli CLI";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = { self, nixpkgs, ... }:
  let
    pkgs = import nixpkgs { system = "x86_64-linux"; };

    pros-cli = pkgs.python3Packages.buildPythonApplication rec {
      pname = "pros-cli";
      version = "3.5.5";
      doCheck = false;

      nativeBuildInputs = with pkgs.python3Packages; [ pip setuptools wheel ];

      propagatedBuildInputs = with pkgs.python3Packages; [
        jsonpickle pyserial tabulate cobs click rich-click cachetools
        requests-futures semantic-version colorama pyzmq sentry-sdk pypng
      ];

      src = pkgs.fetchFromGitHub {
        owner = "purduesigbots";
        repo = pname;
        rev = version;
        sha256 = "sha256-Lw3NJaFmJFt0g3N+jgmGLG5AMeMB4Tqk3d4mPPWvC/c=";
      };

      postInstall = ''
        mkdir -p $python.sitePackages
        echo "${version}" > $python.sitePackages/version

        # create a stable pros-cli name that points to the installed pros binary
        if [ -x "$out/bin/pros" ] && [ ! -e "$out/bin/pros-cli" ]; then
          ln -s "$out/bin/pros" "$out/bin/pros-cli"
        fi
      '';

      pyproject = false;
    };

  in
  {
    devShells.x86_64-linux.default = pkgs.mkShell {
      buildInputs = [
        pros-cli
        pkgs.gcc
        pkgs.python3
      ];

      shellHook = ''
        # 1) Prefer the resolved output path of the derivation
        real=""
        if [ -n "${pros-cli.out}" ] && [ -x "$(readlink -f ${pros-cli.out})/bin/pros" ]; then
          real="$(readlink -f ${pros-cli.out})"
        else
          # 2) Fallback: find the first /nix/store/*pros-cli*/bin that contains an executable "pros"
          for d in /nix/store/*pros-cli*/bin; do
            [ -d "$d" ] || continue
            if [ -x "$d/pros" ]; then
              real="$(readlink -f "$(dirname "$d")")"
              break
            fi
          done
        fi

        if [ -n "$real" ] && [ -d "$real/bin" ]; then
          export PATH="$real/bin:$PATH"
        fi

        # optional compatibility alias
        if ! command -v pros-cli >/dev/null 2>&1 ; then
          alias pros-cli=pros 2>/dev/null || true
        fi

        echo "Welcome to the PROS dev shell!"
        if [ -n "$real" ]; then
          echo "Using pros-cli from: $real"
        else
          echo "Warning: pros binary not found in any pros-cli store path."
        fi
      '';
    };
  };
}
