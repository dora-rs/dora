{
  rustPlatform,
  fetchFromGitHub,
  lib,
  pkg-config,
  openssl,
}:
rustPlatform.buildRustPackage rec {
  pname = "dora-cli";

  version = "0.3.12";

  src = fetchFromGitHub {
    owner = "dora-rs";
    repo = "dora";
    rev = "v${version}";
    hash = "sha256-ebcJq1FCxdBHM0f+PjoNP5YPaynsCe6+BEKHlL3l+Y4=";
  };

  useFetchCargoVendor = true;
  cargoHash = "sha256-6D+ZTMYto+iPazvJEFWfgAcBtBzXHUGBtx8TS9/eTQ8=";
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
