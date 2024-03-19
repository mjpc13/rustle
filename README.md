# RustLE -  Reliable Unitary Simple Tests for Localization Experiments - WIP

## Getting Starting to develop

The only non-cargo dependency is the [evo evaluation tool](https://github.com/MichaelGrupp/evo), that can be installed with pip:

```
pip install evo
```

During initial tests I am using the the [park_dataset.bag](https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq) (3.22GB)

## Running code

To build:
```
cargo build
```

To run with all log levels:
```
RUST_LOG=rustle cargo run
```

At the end of each try you must manually remove the docker container created (latter in the development I will make this automatic, 
for now its usefull to be able to access the container afterwards):
```
docker rm -f rustle-task
```

## Project Structure

```bash
├── Cargo.toml
├── docker/ --> Folder for the dockers of different algorithms (like lio-sam or robot-localization)
│   ├── lio-sam/
│   │   ├── Dockerfile --> Dockerfile to build the algorithm
│   │   ├── entrypoint.sh --> entrypoint of the docker file (maybe not needed anymore)
│   │   └── rustle.launch --> Default launch for the algorithm (loads our custom params)
│   ├── mars/
│   │   └── Dockerfile
│   └── robot-localization/
│       ├── Dockerfile
│       ├── entrypoint.sh
│       └── rustle.launch
├── src/ -> Folder where the main code is stored
│   ├── db.rs -> implements methods for storing and fetching data from SurrealDB
│   ├── errors.rs -> file where our custom errors are declared
│   ├── lib.rs -> library API file
│   ├── main.rs -> execution file
│   ├── metrics.rs -> responsible to extract the metrics from the algorithm
│   └── task.rs -> Responsible to create and run the algorithms and statistics
└── test/ -> folder with some configurations to test in development
    └── config/
        └── params.yaml
    └── dataset/
        └── park_dataset.bag
```
