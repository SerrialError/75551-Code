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
      version = "3.5.6";
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
	sha256 = "sha256-FuqXQk3hnOFipOZZWiLIk9q4P33N+I87NBBf2N+6OOA=";
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
    pros-bin = pkgs.stdenv.mkDerivation {
	    pname = "pros";
	    version = "4.2.1"; # change if needed

      	    src = pkgs.fetchFromGitHub {
        	owner = "purduesigbots";
        	repo = pname;
        	rev = version;
		sha256 = "sha256-FuqXQk3hnOFipOZZWiLIk9q4P33N+I87NBBf2N+6OOA=";
            };

	    nativeBuildInputs = [ pkgs.gnutar ];
	    unpackPhase = "true";
	    buildPhase = ''
		    mkdir -p $out
		    tmpdir=$PWD/tmppros
		    rm -rf "$tmpdir"
		    mkdir -p "$tmpdir"
		    tar -xzf ${src} -C "$tmpdir"
# adjust below to the path inside the tarball to the pros binary
# common layouts: bin/pros or pros/bin/pros
		    if [ -x "$tmpdir/bin/pros" ]; then
			    install -D "$tmpdir/bin/pros" $out/bin/pros
				    elif [ -x "$tmpdir/pros/bin/pros" ]; then
				    install -D "$tmpdir/pros/bin/pros" $out/bin/pros
		    else
			    echo "Could not find pros binary in release archive" >&2
				    ls -la "$tmpdir" >&2
				    exit 1
				    fi
				    '';
	    meta = with pkgs.lib; { description = "PROS CLI binary (packaged)"; };
    };

  in
  {
    devShells.x86_64-linux.default = pkgs.mkShell {
      buildInputs = [
      	pros-bin
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
