{
  description = "A reproducible environment for building certiflight";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "nixpkgs/02336c5c5f719cd6bd4cfc5a091a1ccee6f06b1d";
    mach-nix.url = github:DavHau/mach-nix;
  };

  outputs = inputs:
    inputs.flake-utils.lib.eachDefaultSystem (system:
      let pkgs = (import (inputs.nixpkgs) { config = {allowUnfree = true;}; system =
              "x86_64-linux";
                  });
                  
          extensions = (with pkgs.vscode-extensions; [
            ms-python.python
            ms-python.vscode-pylance
            ms-toolsai.jupyter
            jnoortheen.nix-ide
          ]);

          mach-nix-utils = import inputs.mach-nix {
            inherit pkgs;
            python = "python39Full";
          };

          vscodium-with-extensions = pkgs.vscode-with-extensions.override {
            vscode = pkgs.vscodium.fhs;
            vscodeExtensions = extensions;
          };
          
          python-with-deps = mach-nix-utils.mkPython {
            requirements=''
              numpy
              tensorflow
              pyserial
              crccheck
              mypy
              tqdm
            '';
          };
      in {
        devShell = pkgs.mkShell {
          buildInputs=[
            vscodium-with-extensions
            pkgs.python39Packages.pip
            pkgs.python39Packages.virtualenv
            python-with-deps
            pkgs.betaflight-configurator
          ];
        };
      }
    );
}
