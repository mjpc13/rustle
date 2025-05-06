use rustle_core::{services::TestDefinitionService, models::test_definitions::TestDefinitionsConfig};
use crate::args::{TestCommand, TestSubCommand, AddTest};
use std::{fs::File, error::Error};
use serde_yaml;

pub async fn handle_test(
    cmd: TestCommand,
    service: &TestDefinitionService,
) -> Result<(), Box<dyn Error>> {
    match cmd.command {
        TestSubCommand::Add(add) => {
                        if let Some(file_path) = add.file {
                            // Parse entire tests.yaml file
                            let f = File::open(&file_path)?;
                            let config: TestDefinitionsConfig = serde_yaml::from_reader(f)?;
        
                            let defs = service.create_from_yaml(&file_path).await?;
                            println!("✅ Added {} test definitions from '{}'", defs.len(), file_path);
                        } else {
                            println!("YAML file required for adding test definitions (use --file)");
                        }
            }
        TestSubCommand::Delete(delete_test) => todo!(),
        TestSubCommand::List => todo!(),
        TestSubCommand::Run(run_test) => todo!(),
    }

    Ok(())
}