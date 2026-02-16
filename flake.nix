{
  description = "Adora CLI package for Nix";

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
        adora-cli = pkgs.rustPlatform.buildRustPackage rec {
          pname = "adora-cli";
          version = "0.3.6";

          src = pkgs.fetchFromGitHub {
            owner = "adora-rs";
            repo = "adora";
            rev = "v${version}";
            hash = "sha256-YwEqwA7Eqz7ZJYFfKoPTWkmgsudKpoATcFE6OOwxpbU=";
          };

          cargoHash = "sha256-AkungKYGHMK/tUfzh0d88zRRkQfshus7xOHzdtYAT/I=";
          nativeBuildInputs = [pkgs.pkg-config];
          buildInputs = [pkgs.openssl];
          OPENSSL_NO_VENDOR = 1;
          buildPhase = ''
            cargo build --release -p adora-cli
          '';


          installPhase = ''
            mkdir -p $out/bin
            cp target/release/adora $out/bin
          '';

          doCheck = false;

          meta = {
            description = "Making robotic applications fast and simple!";
            homepage = "https://adora-rs.ai/";
            changelog = "https://github.com/adora-rs/adora/blob/main/Changelog.md";
            license = pkgs.lib.licenses.asl20;
          };
        };
      };
    });
}
