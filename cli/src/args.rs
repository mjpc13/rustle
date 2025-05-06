use clap::{
    Args,
    Parser,
    Subcommand
};

#[derive(Debug, Parser)]
#[clap(author, version, about)]
pub struct RustleArgs{
    #[clap(subcommand)]
    pub command: CommandType,
}

#[derive(Debug, Subcommand)]
pub enum CommandType{

    ///Manage datasets
    Dataset(DatasetCommand),

    // Manage Algorithms
    //Algo(AlgoCommand),

    //Manage Test Setups
    //Test(TestCommand),

    // Manage Configuration of Rustle
    //Config(ConfigCommand)
}

#[derive(Debug, Args)]
pub struct DatasetCommand{
    #[clap(subcommand)]
    pub command: DatasetSubCommand,
}

#[derive(Debug, Subcommand)]
pub enum DatasetSubCommand{
    ///Add a new dataset
    Add(AddDataset),

    ///Update a dataset
    //Update(UpdateDataset),

    /// Delete a dataset
    Delete(DeleteDataset),

    /// List all dataset
    List,
}

#[derive(Debug, Args)]
pub struct AddDataset{
    /// The name of the dataset entry
    pub name: String,

    /// Groundtruth topic
    pub ground_truth_topic: String,

    /// Path for the dataset
    pub dataset_path: String,
}

#[derive(Debug, Args)]
pub struct DeleteDataset{
    /// The name of the dataset entry
    pub name: String,
}
