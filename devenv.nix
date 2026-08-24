{ pkgs, lib, inputs, ... }:

let
  # clangd for cross-compiled PROS code. The nixpkgs-wrapped clang-tools
  # clangd injects host x86_64 glibc/libstdc++ headers via CPATH, which
  # breaks arm-none-eabi parsing (e.g. missing gnu/stubs-32.h). Use the
  # unwrapped clangd and let --query-driver extract the real sysroot
  # includes and target from the cross-gcc in compile_commands.json. The
  # unwrapped binary cannot find clang's builtin headers (they live in the
  # separate lib output), hence the explicit --resource-dir.
  clangd-pros = pkgs.writeShellScriptBin "clangd" ''
    exec ${lib.getExe' pkgs.llvmPackages.clang-unwrapped "clangd"} \
      --query-driver='/run/current-system/sw/bin/arm-none-eabi-*' \
      --resource-dir='${lib.getLib pkgs.llvmPackages.clang-unwrapped}/lib/clang/${lib.versions.major pkgs.llvmPackages.clang-unwrapped.version}' \
      "$@"
  '';

  pros-cli = pkgs.python3Packages.buildPythonApplication rec {
    pname = "pros-cli";
    version = builtins.replaceStrings [ "\n" " " ] [ "" "" ]
      (builtins.readFile "${inputs.pros-cli-src}/version");
    doCheck = false;

    # pyproject = false (the old flake's setting) means "no build format" in
    # nixpkgs, producing an empty package with no bin/pros.
    format = "setuptools";

    nativeBuildInputs = with pkgs.python3Packages; [ setuptools wheel ];

    propagatedBuildInputs = with pkgs.python3Packages; [
      jsonpickle pyserial tabulate cobs click rich-click cachetools
      requests-futures semantic-version colorama pyzmq sentry-sdk pypng
    ];

    src = inputs.pros-cli-src;

    # pros' get_version() reads <site-packages>/version at runtime; the
    # pkg_resources fallback no longer exists on python 3.13.
    postInstall = ''
      echo "${version}" > $out/${pkgs.python3.sitePackages}/version
    '';
  };

in
{
  # Cache management needs nix trusted-user rights; without them devenv warns
  # on every activation. nixpkgs' cache still applies, and the custom
  # pros-cli/clangd builds wouldn't be in devenv's cache anyway.
  cachix.enable = false;

  packages = [
    pros-cli
    clangd-pros
    pkgs.gcc
    pkgs.python3
  ];

  enterShell = ''
    echo "PROS dev environment ready ($(pros --version 2>/dev/null || echo 'pros not found'))"
  '';
}
