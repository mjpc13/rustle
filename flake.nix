{
  description = "A development shell for my project";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = { self, nixpkgs }: 
    let
      system = "x86_64-linux"; # Change this to your system if different (e.g., "aarch64-darwin")
      prj_root = builtins.toString self.outPath;

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
          rustc
          cargo
          rust-analyzer # LSP Server
          rustfmt       # Formatter
          clippy        # Linter
          clang
          surrealdb     # Database
          pkg-config
          fontconfig
          gcc
          docker
          surrealist

          python311Full


          # Custom cleanup script
          (writeShellScriptBin "rustle_clean" ''
            #!/bin/sh
            echo "This will permanently remove:"
            echo "   - All database files in: $RUSTLE_ROOT/test/db/*"
            echo "   - All test results in: $RUSTLE_ROOT/test/results/*"
            echo "   - All Docker containers currently running on your system"
            echo ""
            printf "Are you sure you want to continue? (y/N) "
            read answer

            case "$answer" in
                [yY]|[yY][eE][sS])
                    echo "Starting cleanup..."
                    # Remove test directories
                    echo "Removing database and results..."
                    rm -rf "$RUSTLE_ROOT/test/db/*" "$RUSTLE_ROOT/test/results/*"
                    
                    # Remove Docker containers
                    echo "Stopping Docker containers..."
                    sudo docker rm -f $(sudo docker ps -aq) 2>/dev/null || true
                    
                    echo "Cleanup completed!"
                    ;;
                *)
                    echo "Cleanup aborted"
                    exit 0
                    ;;
            esac
          '')

          # Custom script to launch a database server
          (writeShellScriptBin "rustle_db" ''
            #!/bin/sh
            surreal start --log debug --user root --pass root "rocksdb://$RUSTLE_ROOT/test/db/"
          '')

        ];

        shellHook = ''
          # Set environment variable with absolute project path
          export RUSTLE_ROOT="$(pwd)"

          export LIBCLANG_PATH="${pkgs.libclang.lib}/lib"
          export LD_LIBRARY_PATH="${pkgs.gcc.cc.lib}/lib:${pkgs.stdenv.cc.cc.lib}/lib:$LD_LIBRARY_PATH"
          python -m venv .venv
          source .venv/bin/activate
          pip install evo
        '';
      };
    };
}
