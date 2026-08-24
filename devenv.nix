{ pkgs, inputs, ... }:

let
  system = pkgs.stdenv.hostPlatform.system;
  fenix = inputs.fenix.packages.${system};

  # Single source of truth: ./rust-toolchain.toml.
  # fenix reads the channel + components (rust-src) straight from that file,
  # so the toolchain is never duplicated/hardcoded here.
  rustToolchain = fenix.fromToolchainFile {
    file = ./rust-toolchain.toml;
    # Hash of the resolved toolchain (rustc/cargo/rust-src for nightly-2025-11-26).
    # Deliberate tradeoff: reproducible, but bound to this channel + the fenix
    # input rev. Bump the channel in rust-toolchain.toml => this hash goes stale
    # and the build fails with a mismatch until you paste the new "got:" value.
    sha256 = "sha256-bYq1DZv2iwLeZAQwA1nqJQgx7p1M/srnZyr1FYJ3+GU=";
  };

  # Source-built cargo-v5 from the upstream flake (buildRustPackage/crane),
  # fully pinned via devenv.lock — no `cargo install`, no ~/.cargo/bin PATH hacks.
  cargo-v5 = inputs.cargo-v5.packages.${system}.default;
in
{
  # `connor` isn't a trusted Nix user, so devenv can't auto-manage the cachix
  # binary cache — left enabled it aborts shell evaluation. Disable it here
  # (project-scoped) instead of editing global nix.conf / trusted-users.
  cachix.enable = false;

  packages = [
    rustToolchain
    cargo-v5

    # ELF -> .bin conversion / bare-metal linking used by `cargo v5 build`.
    pkgs.cargo-binutils
    pkgs.llvmPackages.bintools

    # USB serial deps for cargo-v5 / vex-v5-serial (the usual NixOS failure point).
    # libudev-zero is a standalone libudev with no systemd closure — purpose-built
    # for the serialport crate. pkg-config's setup hook auto-adds it to
    # PKG_CONFIG_PATH (no manual, clobber-prone override needed).
    pkgs.pkg-config
    pkgs.libudev-zero
  ];

  enterShell = ''
    echo "vexide dev shell"
    echo "  rustc:    $(rustc --version)"
    echo "  cargo:    $(cargo --version)"
    echo "  cargo-v5: $(cargo v5 --version || echo unavailable)"
    if [ -d "$(rustc --print sysroot)/lib/rustlib/src/rust/library" ]; then
      echo "  rust-src: present"
    else
      echo "  rust-src: MISSING"
    fi
  '';
}
