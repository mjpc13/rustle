{
  description = "A development shell for my project";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = { self, nixpkgs }: 
    let
      system = "x86_64-linux"; # Change this to your system if different (e.g., "aarch64-darwin")

      pkgs = import nixpkgs {
        inherit system;
        config.allowUnfree = true;
      };

      # Create a Python environment with tkinter
      pythonWithTkinter = pkgs.python311.withPackages (ps: [
        ps.tkinter
      ]);

    in {
      devShells.${system}.default = pkgs.mkShell {
        buildInputs = with pkgs; [
          rust-analyzer # LSP Server
          rustfmt       # Formatter
          clippy        # Linter
          clang
          surrealdb     # Database
          pkg-config
          fontconfig
          gcc

          python311Full
        ];

        shellHook = ''
          export LIBCLANG_PATH="${pkgs.libclang.lib}/lib"
          export LD_LIBRARY_PATH="${pkgs.gcc.cc.lib}/lib:${pkgs.stdenv.cc.cc.lib}/lib:$LD_LIBRARY_PATH"
          python -m venv .venv
          source .venv/bin/activate
          pip install evo
        '';
      };
    };
}