use clap::{Args, Parser, Subcommand};

/// CLI parser for the `rustle` application.
#[derive(Debug, Parser)]
#[clap(author, version, about)]
pub struct RustleArgs {
    /// Top-level command to execute
    #[clap(subcommand)]
    pub command: CommandType,
}

/// Enum defining available top-level commands.
#[derive(Debug, Subcommand)]
pub enum CommandType {
    /// Manage datasets (add, update, delete, list)
    Dataset(DatasetCommand),

    /// Manage SLAM algorithms (add, delete, list)
    Algo(AlgoCommand),

    /// Manage and run test setups
    Test(TestCommand),

    /// Manage application configuration
    Config(ConfigCommand),
}

// ================== DATASET COMMANDS ==================

/// Command wrapper for dataset-related actions.
#[derive(Debug, Args)]
pub struct DatasetCommand {
    #[clap(subcommand)]
    pub command: DatasetSubCommand,
}

/// Available subcommands under `dataset`.
#[derive(Debug, Subcommand)]
pub enum DatasetSubCommand {
    /// Add a new dataset entry
    Add(AddDataset),

    /// Update an existing dataset entry
    Update(UpdateDataset),

    /// Delete a dataset entry
    Delete(DeleteDataset),

    /// List all dataset entries
    List,
}

/// Arguments for adding a dataset.
#[derive(Debug, Args)]
pub struct AddDataset {
    /// YAML file to load dataset from
    #[clap(short, long)]
    pub file: Option<String>,

    /// Dataset name (if not using a file)
    pub name: Option<String>,

    /// Topic name for groundtruth data
    pub ground_truth_topic: Option<String>,

    /// File path where the dataset is stored
    pub dataset_path: Option<String>,
}

/// Arguments for updating a dataset.
#[derive(Debug, Args)]
pub struct UpdateDataset {
    /// Name of the dataset to update
    pub name: String,

    /// New groundtruth topic (optional)
    #[clap(long)]
    pub ground_truth_topic: Option<String>,

    /// New dataset path (optional)
    #[clap(long)]
    pub dataset_path: Option<String>,
}

/// Arguments for deleting a dataset.
#[derive(Debug, Args)]
pub struct DeleteDataset {
    /// Name of the dataset to delete
    pub name: String,
}

// ================== ALGO COMMANDS ==================

/// Command wrapper for algorithm-related actions.
#[derive(Debug, Args)]
pub struct AlgoCommand {
    #[clap(subcommand)]
    pub command: AlgoSubCommand,
}

/// Available subcommands under `algo`.
#[derive(Debug, Subcommand)]
pub enum AlgoSubCommand {
    /// Add a new SLAM algorithm
    Add(AddAlgorithm),

    /// Delete an existing algorithm
    Delete(DeleteAlgorithm),

    /// List all available algorithms
    List,
}

/// Arguments for adding a SLAM algorithm.
#[derive(Debug, Args)]
pub struct AddAlgorithm {
    /// YAML file to load algorithm configuration from
    #[clap(short, long)]
    pub file: Option<String>,

    /// Algorithm name (if not using a file)
    pub name: Option<String>,

    /// Version string
    pub version: Option<String>,

    /// Container or image name
    pub image_name: Option<String>,

    /// Custom parameters in string format
    pub parameters: Option<String>,

    /// List of odometry topics
    #[clap(short, long)]
    pub odom_topics: Vec<String>,
}

/// Arguments for deleting a SLAM algorithm.
#[derive(Debug, Args)]
pub struct DeleteAlgorithm {
    /// Name of the algorithm to delete
    pub name: String,
}

// ================== TEST COMMANDS ==================

/// Command wrapper for test-related actions.
#[derive(Debug, Args)]
pub struct TestCommand {
    #[clap(subcommand)]
    pub command: TestSubCommand,
}

/// Available subcommands under `test`.
#[derive(Debug, Subcommand)]
pub enum TestSubCommand {
    /// Add a new test setup
    Add(AddTest),

    /// Cleans the results for an existing test setup
    Clean(CleanTest),

    /// Delete an existing test setup
    Delete(DeleteTest),

    /// List all test setups
    List,

    /// Run a test or all tests
    Run(RunTest),

    /// Plot the results of a test
    Plot(PlotTest),

    /// Show the results of a test
    Show(ShowTest),
}

/// Arguments for adding a test.
#[derive(Debug, Args)]
pub struct AddTest {
    /// YAML file to load test from
    #[clap(short, long)]
    pub file: Option<String>,

    /// Test name (if not using a file)
    pub name: Option<String>,

    /// Number of parallel workers
    pub workers: Option<usize>,

    /// Number of iterations per test
    pub iterations: Option<usize>,

    /// List of algorithms to test
    #[clap(short, long)]
    pub algo_list: Vec<String>,

    /// Dataset name to test on
    pub dataset_name: Option<String>,

    /// Type of test (e.g., benchmark, accuracy)
    #[clap(long)]
    pub test_type: Option<String>,
}

/// Arguments for deleting a test.
#[derive(Debug, Args)]
pub struct DeleteTest {
    /// Name of the test to delete
    pub name: String,
}

#[derive(Debug, Args)]
pub struct CleanTest{
    /// Name of the test to clean
    pub name: String,

    /// Clean all tests
    #[clap(short, long)]
    pub all: bool,
}



/// Arguments for running a test.
#[derive(Debug, Args)]
pub struct RunTest {
    /// Run all tests
    #[clap(short, long)]
    pub all: bool,

    /// Specific test name to run (optional if `--all` is used)
    pub name: Option<String>,
}

/// Arguments for plotting test results.
#[derive(Debug, Args)]
pub struct PlotTest {
    /// Plot results for all tests
    #[clap(short, long)]
    pub all: bool,

    /// Specific test name to plot (optional if `--all` is used)
    pub name: Option<String>,

    /// Output directory for plots
    #[clap(short, long)]
    pub output_dir: Option<String>,

    /// File format for plot output (e.g., svg, png)
    #[clap(long, default_value = "svg")]
    pub format: String,

    /// Overwrite existing files
    #[clap(long, default_value = "false")]
    pub overwrite: bool,
}

/// Arguments for displaying test results.
#[derive(Debug, Args)]
pub struct ShowTest {
    /// Name of the test to show
    pub name: String,

    /// Print detailed metrics
    #[clap(long, default_value = "false")]
    pub detailed: bool,

    /// Sorting field for metrics (e.g., accuracy, time)
    #[clap(long, default_value = "random")]
    pub sort_by: Option<String>,

    /// Output format (e.g., csv, json)
    #[clap(long, default_value = "csv")]
    pub format: String,

    /// Overwrite existing output
    #[clap(long, default_value = "false")]
    pub overwrite: bool,

    /// Output directory
    #[clap(long)]
    pub output_dir: Option<String>,
}

// ================== CONFIG COMMANDS ==================

/// Command wrapper for configuration management.
#[derive(Debug, Args)]
pub struct ConfigCommand {
    #[clap(subcommand)]
    pub command: ConfigSubCommand,
}

/// Available subcommands under `config`.
#[derive(Debug, Subcommand)]
pub enum ConfigSubCommand {
    /// Display current configuration settings
    Show,

    /// Set a configuration value
    Set(SetConfig),
}

/// Arguments for setting a configuration key-value.
#[derive(Debug, Args)]
pub struct SetConfig {
    /// Configuration section to modify (e.g., "database")
    pub section: String,

    /// Key within the section to update
    pub key: String,

    /// New value to assign to the key
    pub value: String,
}
