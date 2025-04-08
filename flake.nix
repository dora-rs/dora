{
  description = "Dora CLI package for Nix";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = {
    self,
    nixpkgs,
    flake-utils,
  }:
    flake-utils.lib.eachDefaultSystem (system: let
      pkgs = import nixpkgs {inherit system;};
    in {
      packages = {
        dora-cli = pkgs.rustPlatform.buildRustPackage rec {
          pname = "dora-cli";
          version = "0.3.6";

          src = pkgs.fetchFromGitHub {
            owner = "dora-rs";
            repo = "dora";
            rev = "v${version}";
            hash = "sha256-YwEqwA7Eqz7ZJYFfKoPTWkmgsudKpoATcFE6OOwxpbU=";
          };

          cargoHash = "sha256-vdKA4qfoWEgTOaR1Deve87GiIiKq3uRpuGo66+IflTg=";
          nativeBuildInputs = [pkgs.pkg-config];
          buildInputs = [pkgs.openssl];
          OPENSSL_NO_VENDOR = 1;
          buildPhase = ''
            cargo build --release -p dora-cli
          '';


          installPhase = ''
            mkdir -p $out/bin
            cp target/release/dora $out/bin
          '';

          doCheck = false;

          meta = {
            description = "Making robotic applications fast and simple!";
            homepage = "https://dora-rs.ai/";
            changelog = "https://github.com/dora-rs/dora/blob/main/Changelog.md";
            license = pkgs.lib.licenses.asl20;
          };
        };
      };
    });
}
