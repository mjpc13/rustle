use comfy_table::{presets::UTF8_FULL, ContentArrangement, Table};
use log::info;
use rustle_core::{
    models::{
        test_definitions::TestDefinitionsConfig,
        TestExecution, TestExecutionStatus,
    },
    services::{TestDefinitionService, TestExecutionService},
};
use crate::args::{TestCommand, TestSubCommand};
use std::{error::Error, fs::File};
use serde_yaml::from_reader;

pub async fn handle_test(
    cmd: TestCommand,
    service: &TestDefinitionService,
    test_exec_service: &TestExecutionService,
) -> Result<(), Box<dyn Error>> {
    match cmd.command {
        TestSubCommand::Add(add) => {
                        if let Some(file_path) = add.file {
                            let config: TestDefinitionsConfig = load_yaml_config(&file_path)?;
                            let defs = service.create_from_yaml(&file_path).await?;
                            println!("Added {} test definitions from '{}'", defs.len(), file_path);
                        } else {
                            println!("YAML file required for adding test definitions (use --file)");
                        }
            }
        TestSubCommand::List => {
                let tests = service.get_all().await;

                if tests.is_empty() {
                    println!("No test definitions found.");
                    return Ok(());
                }

                let mut table = Table::new();
                table.load_preset(UTF8_FULL);
                table.set_content_arrangement(ContentArrangement::Dynamic);
                table.set_header(vec!["Name", "Type", "Iterations", "Datasets", "Algorithms"]);

                for test in tests {
                    table.add_row(vec![
                        test.name,
                        format!("{:?}", test.test_type),
                        test.iterations.to_string(),
                        format!("{:?}", test.dataset_name),
                        format!("{:?}", test.algo_list),
                    ]);
                }

                println!("{table}");
            }
        TestSubCommand::Delete(del) => {
                service.delete_test_by_name(&del.name).await;
                println!("Deleted test definition '{}'", del.name);
            }
        TestSubCommand::Run(run) => {
                // If --all flag is present, run all test definitions
                if run.all {
                    let tests = service.get_all().await;
                    if tests.is_empty() {
                        println!("No test definitions available to run.");
                        return Ok(());
                    }

                    // Loop through all tests and start execution
                    for test in tests {
                        info!("Running test: {}", test.name);

                        // Create initial execution object
                        let mut execution = TestExecution {
                            id: None,
                            status: TestExecutionStatus::Scheduled,
                            num_iterations: test.iterations,
                            start_time: None,
                            end_time: None,
                            results: None,
                        };

                        let _ = test_exec_service.start_execution(execution, &test).await;
                    }

                    println!("Started execution for all tests.");
                } else {
                    // If --all isn't present, execute a specific test (by name)
                    if let Some(name) = run.name {
                        let test = service.get_by_name(&name).await?.unwrap();
                        info!("Running test: {}", test.name);

                        let mut execution = TestExecution {
                            id: None,
                            status: TestExecutionStatus::Scheduled,
                            num_iterations: test.iterations,
                            start_time: None,
                            end_time: None,
                            results: None,
                        };

                        let _ = test_exec_service.start_execution(execution, &test).await;
                        println!("Started execution for test: {}", test.name);
                    }
                }
            }
        TestSubCommand::Plot(plot_test) => {

            if plot_test.all {
                let tests = service.get_all().await;
                if tests.is_empty() {
                    println!("No test definitions available to plot.");
                    return Ok(());
                }

                // Loop through all tests and start execution
                for test in tests { 
                    // CALL THE PLOT THING FOR EACH TEST DEF. BE CAREFULL THEY MIGHT NOT HAVE DATA YET!
                }

            } else {
                // If --all isn't present, execute a specific test (by name)
                if let Some(name) = plot_test.name {
                    // CALL THE PLOT THING FOR A SINGLE TEST DEF. BE CAREFULL THEY MIGHT NOT HAVE DATA YET!
                }
            }

        },
    }

    Ok(())
}

fn load_yaml_config<T: serde::de::DeserializeOwned>(path: &str) -> Result<T, Box<dyn Error>> {
    let file = File::open(path)?;
    Ok(from_reader(file)?)
}

