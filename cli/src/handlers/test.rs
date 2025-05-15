use comfy_table::{presets::{ASCII_MARKDOWN, }, ContentArrangement, Table};
use log::{error, info, warn};
use rustle_core::{ models::{
        algorithm_run::AlgorithmRun, metric::MetricType::{
            Cpu, Frequency, Memory, PoseError
        }, test_definitions::TestDefinitionsConfig, TestExecution, TestExecutionStatus
    }, services::{TestDefinitionService, TestExecutionService}, utils::config::Config
};


use crate::args::{ShowTest, TestCommand, TestSubCommand};
use std::{error::Error, fs::{create_dir_all, File}, path::Path};
use serde_yaml::from_reader;

pub async fn handle_test(
    cmd: TestCommand,
    service: &TestDefinitionService,
    test_exec_service: &TestExecutionService,
) -> Result<(), Box<dyn Error>> {
    match cmd.command {
        TestSubCommand::Add(add) => {
                            if let Some(file_path) = add.file {
                                let _config: TestDefinitionsConfig = load_yaml_config(&file_path)?;
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
                    table.load_preset(ASCII_MARKDOWN);
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
                            let execution = TestExecution {
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

                            let execution = TestExecution {
                                id: None,
                                status: TestExecutionStatus::Scheduled,
                                num_iterations: test.iterations,
                                start_time: None,
                                end_time: None,
                                results: None,
                            };

                            let _ = test_exec_service.start_execution(execution, &test).await;
                            //println!("Started execution for test: {}", test.name);
                        }
                    }
                }
        TestSubCommand::Plot(plot_test) => {
            
                //Ensures the output path exists
                let output_path = match plot_test.output_dir {
                    Some(p) => p,
                    None => {
                        let config = Config::load()?; // Load from config file
                        config.data.path.clone()
                    }
                };

                if !Path::new(&output_path).exists() {
                    create_dir_all(&output_path)?; // Ensure output directory exists
                }
        
                let allowed_formats = ["png", "svg", "pdf"];
                if !allowed_formats.contains(&plot_test.format.as_str()) {
                    error!("Invalid format '{}'. Allowed formats: png, svg, pdf", plot_test.format);
                    return Ok(());
                }

                if plot_test.all {
                    let tests = service.get_all().await;
                    if tests.is_empty() {
                        warn!("No test definitions available to plot.");
                        return Ok(());
                    }

                    // Loop through all tests and start execution
                    for test in tests { 
                        // CALL THE PLOT THING FOR EACH TEST DEF. BE CAREFULL THEY MIGHT NOT HAVE DATA YET!
                                        
                        //Get the test executions derived from test definition
                        //let exec = service.get_executions(test).await;

                        //plot for every tests, but some tests may not have the necessary data, 
                        // this will throw an error for sure. DEAL WITH IT
                        if let Err(e) = test_exec_service
                            .plot_execution(&test, &output_path, plot_test.overwrite, &plot_test.format)
                            .await {
                                warn!("Failed to plot test '{}': {}", test.name, e);
                        }

                    }

                } else {
                    // If --all isn't present, execute a specific test (by name)
                    if let Some(name) = plot_test.name {
                        //todo!("Not implemented yet. Should be the same logic as to compute for all.");

                        let test = match service.get_by_name(&name).await? {
                            Some(t) => t,
                            None => {
                                error!("Test definition '{}' not found", name);
                                return Ok(());
                            }
                        };

                        if let Err(e) = test_exec_service
                            .plot_execution(&test, &output_path, plot_test.overwrite, &plot_test.format)
                            .await {
                                warn!("Failed to plot test '{}': {}", test.name, e);
                        }

                        // CALL THE PLOT THING FOR A SINGLE TEST DEF. BE CAREFULL THEY MIGHT NOT HAVE DATA YET!
                    }
                }

            },
            
            TestSubCommand::Show(show_test) => {
                let _ = handle_show_cmd(show_test, service, test_exec_service).await;
            },

    }

    Ok(())
}

fn load_yaml_config<T: serde::de::DeserializeOwned>(path: &str) -> Result<T, Box<dyn Error>> {
    let file = File::open(path)?;
    Ok(from_reader(file)?)
}

async fn handle_show_cmd(show_test: ShowTest, service: &TestDefinitionService, test_exec_service: &TestExecutionService) -> Result<(), Box<dyn Error>>{

    //Check if an allowed format was passed
    let allowed_formats = ["cli", "csv", "table", "json"];
    if !allowed_formats.contains(&show_test.format.as_str()) {
        error!("Invalid format '{}'. Allowed formats: png, svg, pdf", show_test.format);
        //return Ok(()); Return an error and deal with it in the TestSubCommand::Show arm;
    }

    //Get the Test Definition By Name
    let test = match service.get_by_name(&show_test.name).await? {
        Some(t) => t,
        None => {
            error!("Test definition '{}' not found", show_test.name);
            todo!();
            //return Ok(()); Return an error and deal with it in the TestSubCommand::Show arm;
        }
    };

    // Get the corresponding test execution; 
    let exec: TestExecution = service.get_executions(test).await?;

    let exec_id = match exec.id{
        Some(id) => id,
        None => return Ok(())
    };

    let algo_runs = test_exec_service.get_algo_runs(&exec_id).await?;

    if show_test.detailed{
        show_detail(&algo_runs, &test_exec_service).await;
    } else {
        show_simple(&algo_runs);
    }

    //Logic to write in a file! Depends on the output location and on the format!

    todo!("Still not finished");
}

async fn show_detail(algo_runs: &Vec<AlgorithmRun>, test_exec_service: &TestExecutionService){

    println!("Test: my_slam_test\n==================\n");
    
    for ar in algo_runs{

        //Get list of iterations and metrics!
        let iterations = test_exec_service.get_iterations_by_algo_run(ar.clone()).await.unwrap();
        let mut table = Table::new();
        table.load_preset(ASCII_MARKDOWN);
        table.set_content_arrangement(ContentArrangement::Dynamic);
        table.set_header(vec!["RUN", "APE" , "RPE" , "CPU (%)" , "Mem (MB)" , "Freq  (Hz)"]);

        //Print the first part!

        let mut ape: String = String::from("NaN");
        let mut rpe = String::from("NaN");
        let mut cpu = String::from("NaN");
        let mut mem = String::from("NaN");
        let mut freq = String::from("NaN");

        let mut algo_string = String::new();
        algo_string = format!("Algorithm: {}\n--------------------\nSummary (Combined):", ar.algo.name);

        println!("{algo_string}");

        for metric in ar.metrics.clone(){

            match metric.metric_type {
                Cpu(cpu_metrics) => {
                    cpu = format!("- CPU Load: Mean={:.3}%, Max={:.3}%, Std={:.3}\n", cpu_metrics.load.mean, cpu_metrics.load.max, cpu_metrics.load.std);
                },
                Memory(memory_metrics) => {
                    mem = format!("- Memory Usage: Max={:.3}Mb, Trend={:.3}Mb/s\n", memory_metrics.usage.max, memory_metrics.usage_trend_mb_sec);
                },
                PoseError(pose_error_metrics) => {
                    ape = format!("- APE: RMSE={:.3}, Mean={:.3}, Max={:.3}, Std={:.3}\n", pose_error_metrics.ape.rmse.ok_or(0.0).unwrap(), pose_error_metrics.ape.mean, pose_error_metrics.ape.max, pose_error_metrics.ape.std);
                    rpe = format!("- RPE: RMSE={:.3}, Mean={:.3}, Max={:.3}, Std={:.3}\n", pose_error_metrics.rpe.rmse.ok_or(0.0).unwrap(), pose_error_metrics.rpe.mean, pose_error_metrics.rpe.max, pose_error_metrics.rpe.std);
                },
                Frequency(statistical_metrics) => {
                    freq =  format!("- Frequency (Hz): Mean: {:.3}, Min: {:.3}, Max: {:.3}, Std: {:.3}\n", statistical_metrics.mean, statistical_metrics.min, statistical_metrics.max, statistical_metrics.std);
                },
            }
        }
        println!("{ape}{rpe}{freq}{cpu}{mem}\n\n Iterations:");

        for it in iterations{
            //Table for the iterations. I need to get all metrics for the iteration
            let mut ape: String = String::from("NaN");
            let mut rpe = String::from("NaN");
            let mut cpu = String::from("NaN");
            let mut mem = String::from("NaN");
            let mut freq = String::from("NaN");

            let metrics = test_exec_service.get_metrics_by_iteration(it.clone()).await.unwrap();

            for metric in metrics{
    
                match metric.metric_type {
                    Cpu(cpu_metrics) => {
                        cpu = format!("{:.3}", cpu_metrics.load.mean);
                    },
                    Memory(memory_metrics) => {
                        mem = format!("{:.3}", memory_metrics.usage.max);
                    },
                    PoseError(pose_error_metrics) => {
                        ape = format!("{:.3}", pose_error_metrics.ape.rmse.ok_or(0.0).unwrap());
                        rpe = format!("{:.3}", pose_error_metrics.rpe.rmse.ok_or(0.0).unwrap());
                    },
                    Frequency(statistical_metrics) => {
                        freq =  format!("{:.3}", statistical_metrics.mean);
                    },
                }
            }
    
            table.add_row(vec![
                format!("{}", it.iteration_num),
                ape,
                rpe,
                cpu,
                mem,
                freq
            ]);
    
        }
        println!("{table}\n");

    }

}

fn show_simple(algo_runs: &Vec<AlgorithmRun>){

    let mut table = Table::new();
    table.load_preset(ASCII_MARKDOWN);
    table.set_content_arrangement(ContentArrangement::Dynamic);
    table.set_header(vec!["Name", "APE" , "RPE" , "CPU (%)" , "Mem (MB)" , "Freq  (Hz)"]);
    

    for ar in algo_runs{

        //Get list of iterations and metrics!!!!

        let mut ape: String = String::from("NaN");
        let mut rpe = String::from("NaN");
        let mut cpu = String::from("NaN");
        let mut mem = String::from("NaN");
        let mut freq = String::from("NaN");


        for metric in ar.metrics.clone(){

            match metric.metric_type {
                Cpu(cpu_metrics) => {
                    cpu = format!("{:.3}", cpu_metrics.load.mean);
                },
                Memory(memory_metrics) => {
                    mem = format!("{:.3}", memory_metrics.usage.max);
                },
                PoseError(pose_error_metrics) => {
                    ape = format!("{:.3}", pose_error_metrics.ape.rmse.ok_or(0.0).unwrap());
                    rpe = format!("{:.3}", pose_error_metrics.rpe.rmse.ok_or(0.0).unwrap());
                },
                Frequency(statistical_metrics) => {
                    freq =  format!("{:.3}", statistical_metrics.mean);
                },
            }
        }
        

        table.add_row(vec![
            ar.algo.name.clone(),
            ape,
            rpe,
            cpu,
            mem,
            freq
        ]);
    }

    println!("{table}");

}