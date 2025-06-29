use comfy_table::{presets::{ASCII_MARKDOWN, }, ContentArrangement, Table};
use log::{error, info, warn};
use rustle_core::{ models::{
        TestType::{Simple, Speed, Cut, Drop},
        algorithm_run::AlgorithmRun, metric::MetricType::{
            Cpu, Frequency, Memory, PoseError, TemporalEfficiency
        }, test_definitions::TestDefinitionsConfig, ProgressMessage, TestExecution, TestExecutionStatus
    }, services::{TestDefinitionService, TestExecutionService}, utils::config::Config
};
use tokio::sync::mpsc;



use crate::args::{CleanTest, ShowTest, TestCommand, TestSubCommand};
use std::{error::Error, fs::{create_dir_all, File}, path::Path};
use serde_yaml::from_reader;

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use indicatif::{MultiProgress, ProgressBar, ProgressStyle};


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
                        
                        let (msg_tx, mut msg_rx) = mpsc::channel::<ProgressMessage>(100);


                        let multi = Arc::new(MultiProgress::new());
                        let bars = Arc::new(Mutex::new(HashMap::new()));

                        tokio::spawn({
                            let multi = multi.clone();
                            let bars = bars.clone();
                            async move {
                                while let Some(msg) = msg_rx.recv().await {
                                    let mut bars = bars.lock().unwrap();
                                
                                    let bar = bars.entry(msg.iteration_num).or_insert_with(|| {
                                        let pb = multi.add(ProgressBar::new((msg.total_duration * 1000.0) as u64));
                                        pb.set_style(
                                            ProgressStyle::with_template(
                                                "{elapsed_precise} | {prefix} |> {bar:40.cyan/blue} {percent}% | {pos:.2}/{len:.2} sec"
                                            )
                                            .unwrap()
                                            .progress_chars("█▇▅▃▁  "),
                                        );
                                        pb.set_prefix(format!(
                                            "\x1b[93m{} | Iter {}\x1b[0m",
                                            msg.algo, msg.iteration_num
                                        ));
                                        pb
                                    });
                                
                                    bar.set_position((msg.duration * 1000.0) as u64);
                                }
                            
                                // Finish remaining bars
                                for bar in bars.lock().unwrap().values() {
                                    bar.finish();
                                }
                            }
                        });


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
                                    metrics: HashMap::new(),
                                };

                                let _ = test_exec_service.start_execution(execution, &test, Some(msg_tx.clone())).await;
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
                                    metrics: HashMap::new(),
                                };

                                let _ = test_exec_service.start_execution(execution, &test, Some(msg_tx)).await;
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
        TestSubCommand::Clean(clean_test) => {
                    let _ = handle_clean_cmd(clean_test, service).await;
                },
    }

    Ok(())
}

fn load_yaml_config<T: serde::de::DeserializeOwned>(path: &str) -> Result<T, Box<dyn Error>> {
    let file = File::open(path)?;
    Ok(from_reader(file)?)
}



async fn handle_clean_cmd(clean_test: CleanTest, service: &TestDefinitionService)  -> Result<(), Box<dyn Error>>{

    // If --all flag is present, clean all test definitions
    if clean_test.all {
        let tests = service.get_all().await;
        if tests.is_empty() {
            println!("No test definitions available to clean.");
            return Ok(());
        }

        // Loop through all tests and start cleaning
        for test in tests {
            service.clean_by_name(test).await?;
        }
        info!("All tests clean!");

    } else {

        if let Some(name) = clean_test.name {

            // If --all isn't present, execute a specific test (by name)
            let test = service.get_by_name(&name).await?.unwrap();
            service.clean_by_name(test).await?;
            info!("Test {} clean!", &name);
        }

    }


    Ok(())
}


async fn handle_show_cmd(show_test: ShowTest, service: &TestDefinitionService, test_exec_service: &TestExecutionService) -> Result<(), Box<dyn Error>>{

    //Check if an allowed format was passed
    let allowed_formats = ["csv", "table", "json"];
    if !allowed_formats.contains(&show_test.format.as_str()) {
        error!("Invalid format '{}'. Allowed formats: csv, table (default), json", show_test.format);
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
    let exec: TestExecution = service.get_executions(test.clone()).await?;

    let exec_id = match &exec.id{
        Some(id) => id,
        None => return Ok(())
    };

    let algo_runs = test_exec_service.get_algo_runs(&exec_id).await?;

    if show_test.detailed{
        match test.test_type {
            Simple => show_detail(&algo_runs, &test_exec_service, &show_test.name).await,
            Speed(_) => {
                println!("--- Overall Results for Speed Metric ---");
                show_speed(&exec);
                println!("--- Specific Results for Each Algorithm Run and Speed ---");
                show_detail(&algo_runs, &test_exec_service, &show_test.name).await;
            },
            Drop(_) => todo!(),
            Cut(_) => todo!(),
        }
        
    } else {
        
        match test.test_type {
            Speed(_) => show_speed(&exec),
            _ => show_simple(&algo_runs),
        }
    }

    //Logic to write in a file! Depends on the output location and on the format!
    Ok(())
}

fn show_simple(algo_runs: &Vec<AlgorithmRun>){

    let mut table = Table::new();
    table.load_preset(ASCII_MARKDOWN);
    table.set_content_arrangement(ContentArrangement::Dynamic);
    table.set_header(vec!["Name", "Bag Speed", "APE" , "RPE" , "CPU (%)" , "Mem (MB)" , "Freq  (Hz)"]);
    

    for ar in algo_runs{

        //Get list of iterations and metrics!!!!

        let mut ape: String = String::from("NaN");
        let bag_speed = ar.bag_speed;
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
                TemporalEfficiency(_) => (),
            }
        }
        

        table.add_row(vec![
            ar.algo.name.clone(),
            bag_speed.to_string(),
            ape,
            rpe,
            cpu,
            mem,
            freq
        ]);
    }

    println!("{table}");
}


fn show_speed(exec: &TestExecution){

    let mut table = Table::new();
    table.load_preset(ASCII_MARKDOWN);
    table.set_content_arrangement(ContentArrangement::Dynamic);
    table.set_header(vec!["Name", "FPT (ms)", "TES",  "ATAS" , "RTAS"]);

    let metrics_hash = &exec.metrics;

    let mut tes: String = String::from("NaN");
    let mut fpt = String::from("NaN");
    let mut atas = String::from("NaN");
    let mut rtas = String::from("NaN");

    metrics_hash.iter().for_each(|(name, metric)|{

        match &metric.metric_type {
            TemporalEfficiency(m) => {

                tes = format!("{:.3}", m.tes);

                let fpt = match m.fpt{
                    -1.0 => format!("{}*", m.fpt),
                    _ => format!("{:.3}", m.fpt * 1000.0)
                };

                atas = format!("{:.3}", m.atas);
                rtas = format!("{:.3}", m.rtas);

                table.add_row(vec![
                    name,
                    &fpt,
                    &tes,
                    &atas,
                    &rtas,
                ]);
            },
            _ => (),
        }

    });

    println!("{table}");
    println!("* - Estimation frequency did not drop by 10% for the given set.");
}

async fn show_detail(algo_runs: &Vec<AlgorithmRun>, test_exec_service: &TestExecutionService, test_name: &String){

    println!("Test: {test_name}\n==================\n");
    
    for ar in algo_runs{

        //Get list of iterations and metrics!
        let iterations = test_exec_service.get_iterations_by_algo_run(ar.clone()).await.unwrap();
        let mut table = Table::new();
        table.load_preset(ASCII_MARKDOWN);
        table.set_content_arrangement(ContentArrangement::Dynamic);
        table.set_header(vec!["RUN", "RMSE APE", "Max APE", "Std APE", "RMSE RPE", "Max RPE","Std RPE", "CPU (%)" , "Mem (MB)" , "Freq  (Hz)"]);

        //Print the first part!

        let mut ape: String = String::from("- APE: RMSE= NaN, Mean= NaN, Max= NaN, Std= NaN\n");
        let mut rpe = String::from("- RPE: RMSE= NaN, Mean= NaN, Max= NaN, Std= NaN\n");
        let mut cpu = String::from("- CPU Load: Mean= NaN, Max= NaN, Std= NaN\n");
        let mut mem = String::from("- Memory Usage: Max=NaN, Trend=NaN\n");
        let mut freq = String::from("- Frequency (Hz): Mean: NaN, Min: NaN, Max: NaN, Std: NaN\n");

         let algo_string = format!("Algorithm: {}\n--------------------\nSummary (Combined):\n- Bag Speed: {}", ar.algo.name, ar.bag_speed);

        println!("{algo_string}");

        for metric in ar.metrics.clone(){

            match metric.metric_type {
                Cpu(cpu_metrics) => {
                                cpu = format!("- CPU Load: Mean= {:.3}%, Max= {:.3}%, Std= {:.3}\n", cpu_metrics.load.mean, cpu_metrics.load.max, cpu_metrics.load.std);
                            },
                Memory(memory_metrics) => {
                                mem = format!("- Memory Usage: Max= {:.3}Mb, Trend= {:.3}Mb/s\n", memory_metrics.usage.max, memory_metrics.usage_trend_mb_sec);
                            },
                PoseError(pose_error_metrics) => {
                                ape = format!("- APE: RMSE= {:.3}, Mean= {:.3}, Max= {:.3}, Std= {:.3}\n", pose_error_metrics.ape.rmse.ok_or(0.0).unwrap(), pose_error_metrics.ape.mean, pose_error_metrics.ape.max, pose_error_metrics.ape.std);
                                rpe = format!("- RPE: RMSE= {:.3}, Mean= {:.3}, Max= {:.3}, Std= {:.3}\n", pose_error_metrics.rpe.rmse.ok_or(0.0).unwrap(), pose_error_metrics.rpe.mean, pose_error_metrics.rpe.max, pose_error_metrics.rpe.std);
                            },
                Frequency(statistical_metrics) => {
                                freq =  format!("- Frequency (Hz): Mean: {:.3}, Min: {:.3}, Max: {:.3}, Std: {:.3}\n", statistical_metrics.mean, statistical_metrics.min, statistical_metrics.max, statistical_metrics.std);
                            },
                TemporalEfficiency(_) => (),
            }
        }
        println!("{ape}{rpe}{freq}{cpu}{mem}\n\n Iterations:");

        for it in iterations{
            //Table for the iterations. I need to get all metrics for the iteration
            let mut rmse_ape: String = String::from("NaN");
            let mut max_ape: String = String::from("NaN");
            let mut std_ape: String = String::from("NaN");
            let mut rmse_rpe: String = String::from("NaN");
            let mut max_rpe: String = String::from("NaN");
            let mut std_rpe: String = String::from("NaN");
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
                                        rmse_ape = format!("{:.3}", pose_error_metrics.ape.rmse.ok_or(0.0).unwrap());
                                        max_ape = format!("{:.3}", pose_error_metrics.ape.max);
                                        std_ape = format!("{:.3}", pose_error_metrics.ape.std);
                                        rmse_rpe = format!("{:.3}", pose_error_metrics.rpe.rmse.ok_or(0.0).unwrap());
                                        max_rpe = format!("{:.3}", pose_error_metrics.rpe.max);
                                        std_rpe = format!("{:.3}", pose_error_metrics.rpe.std);
                                    },
                    Frequency(statistical_metrics) => {
                                        freq =  format!("{:.3}", statistical_metrics.mean);
                                    },
                    TemporalEfficiency(_) => (),
                }
            }
            table.add_row(vec![
                format!("{}", it.iteration_num),
                rmse_ape,
                max_ape,
                std_ape,
                rmse_rpe,
                max_rpe,
                std_rpe,
                cpu,
                mem,
                freq
            ]);
    
        }
        println!("{table}\n");

    }

}