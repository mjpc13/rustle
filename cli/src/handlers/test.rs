use comfy_table::{presets::{ASCII_MARKDOWN, }, ContentArrangement, Table};
use log::{error, info, warn};
use rustle_core::{ models::{
        algorithm_run::AlgorithmRun, metric::MetricType::{
            Cpu, Frequency, Memory, PoseError, Robustness, TemporalEfficiency
        }, metrics::RobustnessMetric, test_definitions::TestDefinitionsConfig, ProgressMessage, TestExecution, TestExecutionStatus, TestType::{Cut, Drop, Simple, Speed}
    }, services::TestExecutionService, utils::config::Config
};
use tokio::sync::mpsc;



use crate::args::{AddTest, CleanTest, RunTest, ShowTest, TestCommand, TestSubCommand};
use std::{error::Error, fs::{create_dir_all, File}, path::Path};
use serde_yaml::from_reader;

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use indicatif::{MultiProgress, ProgressBar, ProgressStyle};


pub async fn handle_test(
    cmd: TestCommand,
    service: &TestExecutionService,
) -> Result<(), Box<dyn Error>> {
    match cmd.command {
        TestSubCommand::Add(add) => {
                                handle_add_cmd(add, service).await?;
                    }
        TestSubCommand::List => {
                        let tests = service.get_all().await.unwrap();

                        if tests.is_empty() {
                            println!("No test definitions found.");
                            return Ok(());
                        }

                        let mut table = Table::new();
                        table.load_preset(ASCII_MARKDOWN);
                        table.set_content_arrangement(ContentArrangement::Dynamic);
                        table.set_header(vec!["Name", "Type", "Iterations", "Datasets", "Algorithms", "Status"]);

                        for test in tests {
                            table.add_row(vec![
                                test.def.name,
                                format!("{:?}", test.def.test_type),
                                test.def.iterations.to_string(),
                                format!("{:?}", test.def.dataset_name),
                                format!("{:?}", test.def.algo_list),
                                format!("{:?}", test.status),
                            ]);
                        }

                        println!("{table}");
                    }
        TestSubCommand::Delete(del) => {
                        service.delete_test_by_name(&del.name).await;
                        println!("Deleted test definition '{}'", del.name);
                    }
        TestSubCommand::Run(run) => {
                        let _ = handle_run_cmd(run, service).await.unwrap();
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
                        let tests = service.get_all().await?;
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
                            if let Err(e) = service
                                .plot_execution(&test, &output_path, plot_test.overwrite, &plot_test.format)
                                .await {
                                    warn!("Failed to plot test '{}': {}", test.def.name, e);
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

                            if let Err(e) = service
                                .plot_execution(&test, &output_path, plot_test.overwrite, &plot_test.format)
                                .await {
                                    warn!("Failed to plot test '{}': {}", test.def.name, e);
                            }

                        }
                    }

                },
        TestSubCommand::Show(show_test) => {
                    let _ = handle_show_cmd(show_test, service).await;
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


async fn handle_clean_cmd(clean_test: CleanTest, service: &TestExecutionService)  -> Result<(), Box<dyn Error>>{

    // If --all flag is present, clean all test definitions
    if clean_test.all {
        let tests = service.get_all().await?;
        if tests.is_empty() {
            println!("No test definitions available to clean.");
            return Ok(());
        }

        // Loop through all tests and start cleaning
        for test in tests {
            service.clean_exec(test).await?;
        }
        info!("All tests clean!");

    } else {

        if let Some(name) = clean_test.name {

            // If --all isn't present, execute a specific test (by name)
            let test = service.get_by_name(&name).await?.unwrap();
            service.clean_exec(test).await?;
            info!("Test {} clean!", &name);
        }

    }


    Ok(())
}

async fn handle_add_cmd(add_test: AddTest, service: &TestExecutionService) -> Result<(), Box<dyn Error>>{

    if let Some(file_path) = add_test.file {
        let config: TestDefinitionsConfig = load_yaml_config(&file_path)?;

        let mut exec_list = TestExecution::create_from_yaml(&file_path).await?;

        //save the test executions in the database!

        for mut exec in exec_list{
            service.save_test_execution(&mut exec).await?;
        }

    } else {
        println!("YAML file required for adding test definitions (use --file)");
    }



    //Logic to write in a file! Depends on the output location and on the format!
    Ok(())
}

async fn handle_run_cmd(run_test: RunTest, service: &TestExecutionService) -> Result<(), Box<dyn Error>>{
         
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
    if run_test.all {
        let tests = service.get_all().await?;
        if tests.is_empty() {
            println!("No test definitions available to run.");
            return Ok(());
        }
        // Loop through all tests and start execution
        for test in tests {
            let _ = service.start_execution(test, Some(msg_tx.clone())).await;
        }
        println!("Started execution for all tests.");
    } else {
        // If --all isn't present, execute a specific test (by name)
        if let Some(name) = run_test.name {
            let test = service.get_by_name(&name).await?.unwrap();
            info!("Running test: {}", test.def.name);

            let _ = service.start_execution(test, Some(msg_tx)).await;
            //println!("Started execution for test: {}", test.name);
        }
    };

    Ok(())

}

async fn handle_show_cmd(show_test: ShowTest, service: &TestExecutionService) -> Result<(), Box<dyn Error>>{

    //Check if an allowed format was passed
    let allowed_formats = ["csv", "table", "json"];
    if !allowed_formats.contains(&show_test.format.as_str()) {
        error!("Invalid format '{}'. Allowed formats: csv, table (default), json", show_test.format);
        //return Ok(()); Return an error and deal with it in the TestSubCommand::Show arm;
    }

    //Get the Test Definition By Name
    let exec = match service.get_by_name(&show_test.name).await? {
        Some(t) => t,
        None => {
            error!("Test definition '{}' not found", show_test.name);
            todo!();
            //return Ok(()); Return an error and deal with it in the TestSubCommand::Show arm;
        }
    };

    let exec_id = match &exec.id{
        Some(id) => id,
        None => return Ok(())
    };

    let algo_runs = service.get_algo_runs(&exec_id).await?;

    if show_test.detailed{
        match exec.def.test_type {
            Simple => show_detail(&algo_runs, &service, &show_test.name).await,
            Speed(_) => {
                println!("--- Overall Results for Speed Metric ---");
                show_speed(&exec);
                println!("--- Specific Results for Each Algorithm Run and Speed ---");
                show_detail(&algo_runs, &service, &show_test.name).await;
            },
            _ => {
                println!("--- Overall Results for Robustness Metric ---\n");
                show_drop_crop(&exec);
                println!("\n--- Results for each Sensor Type and Period ---\n");
                show_drop_crop_detail(&exec);
                println!("\n--- Specific Results for Each Algorithm Run ---\n");
                show_detail(&algo_runs, &service, &show_test.name).await;
            }
        }
        
    } else {
        
        match exec.def.test_type {
            Simple => show_simple(&algo_runs),
            Speed(_) => show_speed(&exec),
            Drop(_) => show_drop_crop(&exec),
            Cut(_) => show_drop_crop(&exec),
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
                _ => (),
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

    metrics_hash.iter().for_each(|(algo, metric)|{

        match &metric[0].metric_type {
            TemporalEfficiency(m) => {

                tes = format!("{:.3}", m.tes);

                let fpt = match m.fpt{
                    -1.0 => format!("{}*", m.fpt),
                    _ => format!("{:.3}", m.fpt * 1000.0)
                };

                atas = format!("{:.3}", m.atas);
                rtas = format!("{:.3}", m.rtas);

                table.add_row(vec![
                    &algo,
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

fn show_drop_crop(exec: &TestExecution){

    let mut table = Table::new();
    table.load_preset(ASCII_MARKDOWN);
    table.set_content_arrangement(ContentArrangement::Dynamic);
    table.set_header(vec!["Name", "ADP", "RDP", "ART (ms)", "RRT (ms)"]);

    let metrics_hash = &exec.metrics;

    let mut adp: String = String::from("NaN");
    let mut rdp = String::from("NaN");
    let mut art = String::from("NaN");
    let mut rrt = String::from("NaN");

    metrics_hash.iter().for_each(|(algo, metric)|{

        match &metric[0].metric_type {
            Robustness(r) => {

                adp = format!("{:.3}", r.adp.metric.mean);

                rdp = format!("{:.3}", r.rdp.metric.mean);

                art = match r.art.rt {
                    Some(st) => format!("{:.3}", st.mean),
                    None => "-".to_owned(),
                };
                rrt = match r.rrt.rt {
                    Some(st) => format!("{:.3}", st.mean),
                    None => "-".to_owned(),
                };

                table.add_row(vec![
                    &algo,
                    &adp,
                    &rdp,
                    &art,
                    &rrt,
                ]);
            },
            _ => (),
        }

    });

    println!("{table}");

}

fn show_drop_crop_detail(exec: &TestExecution){

    let mut table = Table::new();
    table.load_preset(ASCII_MARKDOWN);
    table.set_content_arrangement(ContentArrangement::Dynamic);
    table.set_header(vec![
        "Name",
        "Sensor",
        "Period Index",
        "ART (s)",
        "RRT (s)",
    ]);


    let metrics_hash = &exec.metrics;

    for (algo, metric) in metrics_hash {
        match &metric[0].metric_type {
            Robustness(r) => {
                // ART per sensor-period

                let _ = r.art.hash_list_period.iter()
                    .zip(r.rrt.hash_list_period.iter())
                    .for_each(|((sa, va),(sr, vr))|{

                        for (idx, (va, vr)) in va.iter().zip(vr.iter()).enumerate(){
                            
                            let art = match va {
                                Some(v) => format!("{:.3}", v),
                                None => "-".to_owned(),
                            };
                            let rrt = match vr {
                                Some(v) => format!("{:.3}", v),
                                None =>  "-".to_owned(),
                            };

                            table.add_row(vec![
                                &algo,
                                sa,
                                &idx.to_string(),
                                &art,
                                &rrt, // RRT printed below
                            ]);
                        }

                    });

            },
            _ => (),
        }
    }

    println!("{table}");
}

async fn show_detail(algo_runs: &Vec<AlgorithmRun>, service: &TestExecutionService, test_name: &String){

    println!("Test: {test_name}\n==================\n");
    
    for ar in algo_runs{

        //Get list of iterations and metrics!
        let iterations = service.get_iterations_by_algo_run(ar.clone()).await.unwrap();
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
                Robustness(_) => (),
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

            let metrics = service.get_metrics_by_iteration(it.clone()).await.unwrap();

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
                    Robustness(robustness_metric) => todo!(),
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
