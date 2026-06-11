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
          # rustup
          rustc
          cargo
          rust-analyzer # LSP Server
          rustfmt       # Formatter
          clippy        # Linter
          samply
          clang
          surrealdb     # Database
          pkg-config
          fontconfig
          gcc
          docker
          surrealist
          zlib

          python311Full


          # Custom cleanup script
          (writeShellScriptBin "rustle_clean" ''
            #!/bin/sh
            echo "This will permanently remove:"
            echo "   - All database files in: ~/.local/share/rustle/db/*"
            echo "   - All data in: $~/.local/share/rustle/data"
            echo "   - All Docker containers currently running on your system"
            echo ""
            printf "Are you sure you want to continue? (y/N) "
            read answer

            case "$answer" in
                [yY]|[yY][eE][sS])
                    echo "Starting cleanup..."
                    # Remove test directories
                    echo "Removing RUSTLE Data"
                    rm -rf ~/.local/share/rustle/*
                    
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
            surreal start --log debug --user root --pass root "rocksdb:~/.local/share/rustle/db/"
          '')

        ];

        shellHook = ''
          # Set environment variable with absolute project path
          export RUSTLE_ROOT="~/.local/share/rustle/"

          export LIBCLANG_PATH="${pkgs.libclang.lib}/lib"
          export LD_LIBRARY_PATH="${pkgs.zlib}/lib:${pkgs.gcc.cc.lib}/lib:${pkgs.stdenv.cc.cc.lib}/lib:$LD_LIBRARY_PATH"
          python -m venv .venv
          source .venv/bin/activate
          pip install evo rosbags
        '';
      };
    };
}
