{
  rustPlatform,
  lib,
  pkg-config,
  openssl,
}:
rustPlatform.buildRustPackage rec {
  pname = "dora-cli";
  name = "latest";

  src = lib.cleanSource ../.;

  cargoLock = {
    lockFile = ../Cargo.lock;
    # Allow dependencies to be fetched from git and avoid having to set the outputHashes manually
    allowBuiltinFetchGit = true;
  };

  useFetchCargoVendor = true;
  # cargoHash = "sha256-kwJGkxRvCht+aVN8X/elcESuNdYtU1pc0IgNkrlc9a0=";
  nativeBuildInputs = [pkg-config];
  buildInputs = [openssl];
  OPENSSL_NO_VENDOR = 1;
  buildPhase = ''
    cargo build --release -p dora-cli
  '';

  installPhase = ''
    mkdir -p $out/bin
    cp target/release/dora $out/bin
  '';

  # doCheck = false;

  meta = {
    description = "Making robotic applications fast and simple!";
    homepage = "https://dora-rs.ai/";
    changelog = "https://github.com/dora-rs/dora/blob/main/Changelog.md";
    license = lib.licenses.asl20;
  };
}
