{
  description = "Dora CLI package for Nix";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    systems.url = "github:nix-systems/default-linux";
  };

  outputs = {
    self,
    nixpkgs,
    systems,
    ...
  }: let
    inherit (nixpkgs) lib;
    eachSystem = f:
      lib.genAttrs (import systems)
      (system: f nixpkgs.legacyPackages.${system});
  in {
    formatter = eachSystem (pkgs: pkgs.alejandra);

    devShells = eachSystem (pkgs: {
      default = pkgs.mkShell {
        name = "dora-cli";
        venvDir = "./.venv";
        buildInputs = [
          self.packages.${pkgs.system}.dora-cli
          # A Python interpreter including the 'venv' module is required to bootstrap
          # the environment.
          pkgs.python3Packages.python

          # This executes some shell code to initialize a venv in $venvDir before
          # dropping into the shell
          pkgs.python3Packages.venvShellHook

          pkgs.python3Packages.pyarrow
        ];

        # Run this command, only after creating the virtual environment
        postVenvCreation = ''
          unset SOURCE_DATE_EPOCH
          # pip install -r requirements.txt
        '';
      };
    });

    packages = eachSystem (pkgs: {
      default = self.packages.${pkgs.system}.dora-cli;
      dora-cli = pkgs.callPackage ./nix/package.nix {};
    });
  };
}
