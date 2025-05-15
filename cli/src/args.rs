use clap::{Args, Parser, Subcommand};

#[derive(Debug, Parser)]
#[clap(author, version, about)]
pub struct RustleArgs {
    #[clap(subcommand)]
    pub command: CommandType,
}

#[derive(Debug, Subcommand)]
pub enum CommandType {
    /// Manage datasets
    Dataset(DatasetCommand),

    /// Manage SLAM algorithms
    Algo(AlgoCommand),

    /// Manage test setups
    Test(TestCommand),

    /// Manage configuration
    Config(ConfigCommand),
}

// ==== DATASET COMMANDS ====

#[derive(Debug, Args)]
pub struct DatasetCommand {
    #[clap(subcommand)]
    pub command: DatasetSubCommand,
}

#[derive(Debug, Subcommand)]
pub enum DatasetSubCommand {
    /// Add a new dataset
    Add(AddDataset),

    /// Update a dataset
    Update(UpdateDataset),

    /// Delete a dataset
    Delete(DeleteDataset),

    /// List all datasets
    List,
}

#[derive(Debug, Args)]
pub struct AddDataset {
    /// Add from a YAML file
    #[clap(short, long)]
    pub file: Option<String>,

    /// The name of the dataset entry
    pub name: Option<String>,

    /// Groundtruth topic
    pub ground_truth_topic: Option<String>,

    /// Path for the dataset
    pub dataset_path: Option<String>,
}

#[derive(Debug, Args)]
pub struct UpdateDataset {
    /// The name of the dataset to update
    pub name: String,

    /// Optional new groundtruth topic
    #[clap(long)]
    pub ground_truth_topic: Option<String>,

    /// Optional new dataset path
    #[clap(long)]
    pub dataset_path: Option<String>,
}

#[derive(Debug, Args)]
pub struct DeleteDataset {
    /// The name of the dataset entry
    pub name: String,
}

// ==== ALGO COMMANDS ====

#[derive(Debug, Args)]
pub struct AlgoCommand {
    #[clap(subcommand)]
    pub command: AlgoSubCommand,
}

#[derive(Debug, Subcommand)]
pub enum AlgoSubCommand {
    /// Add a new SLAM algorithm
    Add(AddAlgorithm),

    /// Delete an algorithm
    Delete(DeleteAlgorithm),

    /// List available algorithms
    List,
}

#[derive(Debug, Args)]
pub struct AddAlgorithm {
    /// Add from a YAML file
    #[clap(short, long)]
    pub file: Option<String>,

    pub name: Option<String>,

    pub version: Option<String>,

    pub image_name: Option<String>,

    pub parameters: Option<String>,

    #[clap(short, long)]
    pub odom_topics: Vec<String>,
}

#[derive(Debug, Args)]
pub struct DeleteAlgorithm {
    pub name: String,
}

// ==== TEST COMMANDS ====

#[derive(Debug, Args)]
pub struct TestCommand {
    #[clap(subcommand)]
    pub command: TestSubCommand,
}

#[derive(Debug, Subcommand)]
pub enum TestSubCommand {
    /// Add a new test definition
    Add(AddTest),

    /// Delete a test definition
    Delete(DeleteTest),

    /// List all test definitions
    List,

    /// Run a test
    Run(RunTest),

    /// Plots the result for a given Test
    Plot(PlotTest),

    /// Show the results for a given Test
    Show(ShowTest),
}

#[derive(Debug, Args)]
pub struct AddTest {
    /// Add from a YAML file
    #[clap(short, long)]
    pub file: Option<String>,

    pub name: Option<String>,

    pub workers: Option<usize>,

    pub iterations: Option<usize>,

    #[clap(short, long)]
    pub algo_list: Vec<String>,

    pub dataset_name: Option<String>,

    #[clap(long)]
    pub test_type: Option<String>,
}

#[derive(Debug, Args)]
pub struct DeleteTest {
    pub name: String,
}

#[derive(Debug, Args)]
pub struct RunTest {

    #[clap(short, long)]
    pub all: bool,
    pub name: Option<String>,
}

//#[derive(Debug, Args)]
//pub struct PlotTest {
//
//    #[clap(short, long)]
//    pub all: bool,
//    pub name: Option<String>,
//}


#[derive(Debug, Args)]
pub struct PlotTest {
    #[clap(short, long)]
    pub all: bool,

    pub name: Option<String>,

    #[clap(long)]
    pub output_dir: Option<String>,

    #[clap(long, default_value = "svg")]
    pub format: String,

    #[clap(long, default_value = "false")]
    pub overwrite: bool,
}


#[derive(Debug, Args)]
pub struct ShowTest {

    /// Name of the test
    pub name: String,

    /// Print detailed metrics
    #[clap(long, default_value = "false")]
    pub detailed: bool,

    /// Sort the metrics by
    #[clap(long, default_value = "random")]
    pub sort_by: Option<String>,

    #[clap(long, default_value = "csv")] //csv, json, etc...
    pub format: String,

    #[clap(long, default_value = "false")]
    pub overwrite: bool,

    #[clap(long)]
    pub output_dir: Option<String>,
}





// ==== CONFIG COMMANDS ====

#[derive(Debug, Args)]
pub struct ConfigCommand {
    #[clap(subcommand)]
    pub command: ConfigSubCommand,
}

#[derive(Debug, Subcommand)]
pub enum ConfigSubCommand {
    /// Show current configurations
    Show,

    /// Set a configuration key
    Set(SetConfig),
}

#[derive(Debug, Args)]
pub struct SetConfig {
    /// The section to update, e.g., "database"
    pub section: String,

    /// The key to set, e.g., "path"
    pub key: String,

    /// The new value
    pub value: String,
}