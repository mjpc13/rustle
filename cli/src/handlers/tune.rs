use crate::{args::{TuneCommand, TuneSubCommand}, handlers::{test, tune}};

use comfy_table::{presets::UTF8_FULL, ContentArrangement, Table};
use log::{error, info};
use rustle_core::{services::{tuning::TuningService, AlgorithmService, DatasetService, params::ParamsService}, models::Dataset, models::tuning_config::TuningConfig};
use rustle_core::models::tuning::grid_search::GridSearchConfig;
use chrono::Utc;
use serde_yaml::from_reader;
use std::{error::Error, fs::File, iter, vec};

use serde_json::{Value, Number, Map};
use std::collections::HashMap;
use std::io::BufReader;

use rustle_core::services::error::*;

use comfy_table::presets::ASCII_MARKDOWN;
use log::warn;

use rustle_core::models::tuning::TuneType;
use rustle_core::models::tuning::SimulatedAnnealingConfig;
use rustle_core::models::tuning::simulated_annealing::TemperatureFunction;
use rustle_core::models::Algorithm;

use std::io::Write;


pub async fn handle_tune(tune_cmd: TuneCommand, tuning_service: &mut TuningService, algo_service: &AlgorithmService, dataset_service: &DatasetService, params_service: &ParamsService) -> Result<(), Box<dyn Error>> {
    match tune_cmd.command {
        TuneSubCommand::Add(cfg) => {
            let mut cfg = load_tune_config(cfg.file.unwrap().as_str(), algo_service, dataset_service, params_service).await;

            match cfg {
                Ok(Some(mut result_cfg)) => {
                    let algo_params_id = algo_service.get_by_id(result_cfg.algo_id.clone()).await?.unwrap();
                    let mut initial_config: HashMap<String, Value> = params_service.get_by_id(algo_params_id.current_params).await?.unwrap().params;

                    tuning_service.save_to_db(&mut result_cfg).await?;

                    info!("Created tuning config");
                }
                Ok(None) => info!("Failed to create tuning config"),
                Err(e) => info!("Failed to create tuning config: {}", e),
            }
        }

        TuneSubCommand::List => {
            let tuning_configs = tuning_service.get_all().await?;

            if tuning_configs.is_empty() {
                println!("No tuning configurations found.");
                return Ok(());
            }

            let mut table = Table::new();
            table.load_preset(UTF8_FULL);
            table.set_content_arrangement(ContentArrangement::Dynamic);
            table.set_header(vec![
                "Name",
                "Algorithm name",
                "Dataset name",
                "Tuning algorithm",
            ]);

            for tune_cfg in &tuning_configs {
                let algo_name: String = algo_service.get_algo_name_by_id(tune_cfg.algo_id.clone()).await.unwrap().unwrap();
                let dataset_name: String = dataset_service.get_dataset_name_by_id(&tune_cfg.dataset_id).await.unwrap().unwrap();

                let mut tuning_algo = String::new();

                match tune_cfg.tuning_type {
                    Some(TuneType::GridSearch(_)) => {
                        tuning_algo = String::from("grid search");
                    }
                    Some(TuneType::RandomSearch(_)) => {
                        tuning_algo = String::from("random search");
                    }
                    Some(TuneType::SimulatedAnnealing(_)) => {
                        tuning_algo = String::from("simulated annealing");
                    }
                    None => {}
                }

                table.add_row(vec![
                    tune_cfg.name.clone().unwrap(),
                    algo_name,
                    dataset_name,
                    tuning_algo
                ]);
            }

            println!("{table}");
        }

        TuneSubCommand::Run(tuning_method) => {
            /*
            let file = File::open("examples/test_file.yaml")?;
            let reader = BufReader::new(file);
            let test_map: Map<String, Value> = serde_yaml::from_reader(reader)?;
            println!("{:?}", test_map);

            let yaml = serde_yaml::to_string(&test_map)?;
            //let yaml_pretty = yaml.replace("\"[", "[").replace("]\"", "]").replace("\'[", "[").replace("]\'", "]");

            let mut file = File::create("examples/test_file_output.yaml")?;
            file.write_all(yaml.as_bytes());        
            */
            //let mut config = SLAMConfig {params: HashMap::new(), id: None, created_at: Utc::now() };
            //config.params = serde_yaml::from_reader(reader)?;

            match tuning_service.get_by_name(tuning_method.tuning_instance_name.clone()).await {
                Ok(Some(mut cfg)) => {
                    let _ = tuning_service.run_tuning_algo(&mut cfg).await.unwrap();
                }
                Ok(None) => {
                    info!("Some error ocurred");
                }
                Err(e) => {
                    info!("Failed to retrieve the tuning configuration from the database.");
                }
            }
        }

        TuneSubCommand::Show(tune_test_name) => {
            if let Some(cfg) = tuning_service.get_by_name(tune_test_name.name).await? {
                let mut table = Table::new();
                table.load_preset(ASCII_MARKDOWN);
                table.set_content_arrangement(ContentArrangement::Dynamic);
                table.set_header(vec!["Iter", "APE(RMSE)", "RPE(RMSE)", "Memory usage(Mb)"]);

                for iter_metrics in cfg.results {
                    let mut ape = String::from("NaN");
                    let mut rpe = String::from("NaN");
                    let mut mem = String::from("NaN");

                    let ape_number = tuning_service.get_pose_error(&iter_metrics.0.clone()).unwrap().ape.rmse;
                    let rpe_number = tuning_service.get_pose_error(&iter_metrics.0.clone()).unwrap().rpe.rmse;
                    let memory_usage = tuning_service.get_memory_metrics(iter_metrics.0.clone()).unwrap().usage.mean;

                    ape = format!("{:.5}", ape_number.unwrap());
                    rpe = format!("{:.5}", rpe_number.unwrap());
                    mem = format!("{:.5}", memory_usage);

                    table.add_row(vec![
                        iter_metrics.1.to_string(),
                        ape,
                        rpe,
                        mem,
                    ]);
                }
                println!("{table}");
            }
            else {
                warn!("Some error ocurred");
            }
        }
    }

    Ok(())
}

async fn load_tune_config(file_name: &str, algo_service: &AlgorithmService, dataset_service: &DatasetService, params_service: &ParamsService) -> Result<Option<TuningConfig>, Box<dyn Error>> {
    let file = File::open(file_name)?;
    let reader = BufReader::new(file);

    let cfg: HashMap<String, Value> = serde_yaml::from_reader(reader)?;

    let mut new_tuning_config = TuningConfig::new();

    if let Some(tuning_name) = cfg.get("name") {
        let trimmed_name = tuning_name.as_str().unwrap().trim_matches('"').to_string();
        new_tuning_config.name = Some(trimmed_name.to_string());
    }
    else {
        return Err(Box::new(TuningError::NoName()));
    }

    let new_algo_name = cfg.get("algo_name").ok_or(TuningError::NoAlgoName())?
                       .as_str().ok_or(TuningError::WrongTypeField(String::from("algo_name"), String::from("String")))?
                       .trim_matches('"').to_string();
    new_tuning_config.algo_id = algo_service.get_algo_id_by_name(new_algo_name).await?;


    let dataset_settings_map = cfg.get("dataset_settings").ok_or(TuningError::NoDatasetSettingsField())?
                                              .as_object().ok_or(TuningError::NoDatasetSettingsObject())?;

    let new_dataset_name = dataset_settings_map.get("name").ok_or(TuningError::NoDatasetName())?
                       .as_str().ok_or(TuningError::WrongTypeField(String::from("dataset_name"), String::from("String")))?
                       .trim_matches('"').to_string();
    new_tuning_config.dataset_id = dataset_service.get_dataset_id_by_name(new_dataset_name).await?;

    new_tuning_config.dataset_settings.0 = Some(
        dataset_settings_map.get("start").ok_or(TuningError::NoDatasetStart())?
                            .as_f64().ok_or(TuningError::WrongTypeField(String::from("start(dataset_settings)"), String::from("f64")))? as f32
    );
    if new_tuning_config.dataset_settings.0.unwrap() < (0 as f32) {
        return Err(Box::new(TuningError::NumericTypeNegativeValue(String::from("start(dataset_settings)"))));
    }

    new_tuning_config.dataset_settings.1 = Some(
        dataset_settings_map.get("duration").ok_or(TuningError::NoDatasetDuration())?
                            .as_f64().ok_or(TuningError::WrongTypeField(String::from("duration(dataset_settings)"), String::from("f64")))? as f32
    );
    if new_tuning_config.dataset_settings.0.unwrap() < (0 as f32) {
        return Err(Box::new(TuningError::NumericTypeNegativeValue(String::from("duration(dataset_settings)"))));
    }


    let early_stopping_map = cfg.get("early_stopping").ok_or(TuningError::NoEarlyStoppingField())?
                                            .as_object().ok_or(TuningError::NoEarlyStoppingObject())?;
    
    let mut new_early_stopping_params: (u64, f32) = (3, 0.01);
    new_early_stopping_params.0 = early_stopping_map.get("tolerance").ok_or(TuningError::EarlyStoppingNoToleranceField())?
                                                    .as_u64().ok_or(TuningError::WrongTypeField(String::from("tolerance"), String::from("u64")))?;

    new_early_stopping_params.1 = early_stopping_map.get("delta").ok_or(TuningError::EarlyStoppingNoDeltaField())?
                                                    .as_f64().ok_or(TuningError::WrongTypeField(String::from("delta"), String::from("f64")))? as f32;
    if new_early_stopping_params.1 < 0.0 || new_early_stopping_params.1 > 1.0 {
        return Err(Box::new(TuningError::EarlyStoppingDeltaOutOfRange()));
    }
    new_tuning_config.early_stopping_params = Some(new_early_stopping_params);


    let mut new_metrics_weights: (f32, f32) = (0.0, 0.0);
    let metric_weights_map = cfg.get("metric_weights").ok_or(TuningError::NoMetricWeightsField())?
                                            .as_object().ok_or(TuningError::NoMetricWeightsObject())?;

    new_metrics_weights.0 = match metric_weights_map.get("ape") {
        None => 0 as f32,
        Some(value) => {
            value.as_f64().ok_or(TuningError::WrongTypeField(String::from("ape"), String::from("f64")))? as f32
        }
    };
    new_metrics_weights.1 = match metric_weights_map.get("rpe") {
        None => 0 as f32,
        Some(value) => {
            value.as_f64().ok_or(TuningError::WrongTypeField(String::from("rpe"), String::from("f64")))? as f32
        }
    };
    new_tuning_config.metrics_weights = Some(normalize_weights(&new_metrics_weights));

    new_tuning_config.time_limit = match cfg.get("time_limit") {
        Some(time_value) => {
            if let Some(time_number) = time_value.as_f64() {
                if time_number > (0 as f64) {
                    Some(time_number)
                }
                else {
                    return Err(Box::new(TuningError::InvalidTimeLimit()));
                }
            }
            else {
                return Err(Box::new(TuningError::WrongTypeField(String::from("time_limit"), String::from("f64"))));
            }
        }
        None => None,
    };

    if let Some(tune_algo) = cfg.get("tuning_algo") {
        match tune_algo.as_str() {
            Some("grid_search") => {
                load_grid_search_config(&cfg, &mut new_tuning_config, algo_service, params_service).await?;
            }
            Some("random_search") => {
                load_random_search_config(&cfg, &mut new_tuning_config, algo_service, params_service).await?
            }
            Some("simulated_annealing") => {
                load_simulated_annealing_config(&cfg, &mut new_tuning_config, algo_service, params_service).await?;
            }
            Some(anything_else) => {
                return Err(Box::new(TuningError::InvalidTuningAlgo(String::from(anything_else))));
            }
            None => {
                return Err(Box::new(TuningError::NoTuningAlgo()));
            }
        }
    }
    
    Ok(Some(new_tuning_config))
}

async fn load_grid_search_config(input_file_params: &HashMap<String, Value>, tuning_config: &mut TuningConfig, algo_service: &AlgorithmService, params_service: &ParamsService) -> Result<(), Box<dyn Error>> {
    tuning_config.tuning_algo = Some(String::from("grid_search"));

    // testing some stuff...
    if let Some(params_object) = input_file_params.get("parameters") {
        if let Some(params_map) = params_object.as_object() {
            tuning_config.parameters_to_tune = params_map.clone().into_iter().collect();
        }
        else {
            return Err(Box::new(TuningError::NoParametersObject()));
        }
    }
    else {
        return Err(Box::new(TuningError::NoParametersField()));
    }

    let algo_params_config = algo_service.get_by_id(tuning_config.algo_id.clone()).await?.unwrap();
    let mut algo_params: HashMap<String, Value> = params_service.get_by_id(algo_params_config.current_params).await?.unwrap().params;
    tuning_config.validate_parameter_space(&mut algo_params)?;

    let mut new_grid_search_config = GridSearchConfig::new();
    new_grid_search_config.tunable_params = tuning_config.parameters_to_tune.clone();
    new_grid_search_config.build_array_sizes();
    //println!("{:?}", new_grid_search_config.tunable_params_array_sizes);


    //new_grid_search_config.build_configurations(tuning_config.parameters_to_tune.clone(), algo_params);
    tuning_config.tuning_type = Some(TuneType::GridSearch(new_grid_search_config));
    
    //let initial_grid_point = tuning_config.generate_initial_grid_point().unwrap();
    //let mut current_grid_point = initial_grid_point.clone();

    /*
    println!("{:?}", initial_grid_point);

    for i in 0..100100 {
        current_grid_point = tuning_config.generate_next_grid_point(current_grid_point).unwrap();
        println!("{:?}", current_grid_point);
    }
    */

    Ok(())
}

async fn load_random_search_config(input_file_params: &HashMap<String, Value>, tuning_config: &mut TuningConfig, algo_service: &AlgorithmService, params_service: &ParamsService) -> Result<(), Box<dyn Error>> {
    tuning_config.tuning_algo = Some(String::from("random_search"));
    tuning_config.tuning_type = Some(TuneType::RandomSearch(GridSearchConfig::new()));

    // testing some stuff...
    if let Some(params_object) = input_file_params.get("parameters") {
        if let Some(params_map) = params_object.as_object() {
            tuning_config.parameters_to_tune = params_map.clone().into_iter().collect();
        }
        else {
            return Err(Box::new(TuningError::NoParametersObject()));
        }
    }
    else {
        return Err(Box::new(TuningError::NoParametersField()));
    }

    let algo_params_config = algo_service.get_by_id(tuning_config.algo_id.clone()).await?.unwrap();
    let mut algo_params: HashMap<String, Value> = params_service.get_by_id(algo_params_config.current_params).await?.unwrap().params;
    tuning_config.validate_parameter_space(&mut algo_params)?;

    let mut new_grid_search_config = GridSearchConfig::new();
    new_grid_search_config.tunable_params = tuning_config.parameters_to_tune.clone();
    new_grid_search_config.build_array_sizes();

    //println!("{:?}", new_grid_search_config.tunable_params_array_sizes);
    //new_grid_search_config.build_configurations(tuning_config.parameters_to_tune.clone(), algo_params);
    tuning_config.tuning_type = Some(TuneType::RandomSearch(new_grid_search_config));

    Ok(())
}

async fn load_simulated_annealing_config(input_file_params: &HashMap<String, Value>, tuning_config: &mut TuningConfig, algo_service: &AlgorithmService, params_service: &ParamsService) -> Result<(), Box<dyn Error>> {
    tuning_config.tuning_algo = Some(String::from("simulated_annealing"));

    let mut sa_config = SimulatedAnnealingConfig::new(None)?;

    let tuning_settings_map = input_file_params.get("tuning_settings").ok_or(TuningError::NoTuningSettingsField())?
                                                           .as_object().ok_or(TuningError::NoTuningSettingsObject())?;

    let temperature_settings_map = tuning_settings_map.get("temperature").ok_or(SimulatedAnnealingError::NoTemperatureSettingsField())?
                                                         .as_object().ok_or(SimulatedAnnealingError::NoTemperatureSettingsObject())?;
    sa_config.initial_temp = temperature_settings_map.get("initial_value").ok_or(SimulatedAnnealingError::NoInitialTemperatureValueField())?
                                                     .as_f64().ok_or(SimulatedAnnealingError::InvalidInitialTemperature())?;
    if sa_config.initial_temp.clone() < 0.0 {
        return Err(Box::new(SimulatedAnnealingError::InvalidInitialTemperature()));
    }
    sa_config.current_temp = sa_config.initial_temp.clone();
    
    let temp_func = temperature_settings_map.get("function").ok_or(SimulatedAnnealingError::NoTemperatureFunctionField())?
                                               .as_str().ok_or(TuningError::WrongTypeField(String::from("function"), String::from("String")))?;
    match temp_func {
        "fast" => { sa_config.temp_func = TemperatureFunction::TemperatureFast }
        "boltzman" => { sa_config.temp_func = TemperatureFunction::Boltzman }
        "exponential" => { sa_config.temp_func = TemperatureFunction::Exponential }
        _ => {
            return Err(Box::new(SimulatedAnnealingError::InvalidTemperatureFunction()));
        }
    };

    let algo: Algorithm = algo_service.get_by_id(tuning_config.algo_id.clone()).await?.unwrap();
    let gt_parameters: HashMap<String, Value> = params_service.get_by_id(algo.current_params).await?.unwrap().params;
    let mut new_parameter_bounds: HashMap<String, (Value, Value, Value)> = HashMap::new();

    let parameter_bounds_map = tuning_settings_map.get("parameter_bounds").ok_or(SimulatedAnnealingError::NoParameterBoundsField())?
                                                     .as_object().ok_or(SimulatedAnnealingError::NoParameterBoundsObject())?;

    for (key, value) in parameter_bounds_map.clone() {
        if !gt_parameters.contains_key(&key) {
            return Err(Box::new(SimulatedAnnealingError::BoundKeyNotFound(String::from(algo.name.clone()), String::from(key.clone()))));
        }
        else if !value.is_array() {
            return Err(Box::new(SimulatedAnnealingError::BoundNotArray(key.clone())));
        }
        else if value.clone().as_array().unwrap().len() != 3 {
            return Err(Box::new(SimulatedAnnealingError::BoundArrayWrongLength(key.clone())));
        }
        else if !gt_parameters.get(&key.clone()).unwrap().is_number() {
            return Err(Box::new(SimulatedAnnealingError::BoundParameterForbiddenType(key.clone())));
        }

        let value_arr = value.as_array().unwrap();

        if get_value_type(gt_parameters.get(&key.clone()).unwrap()) != get_value_type(&value_arr[0]) {
            return Err(Box::new(SimulatedAnnealingError::BoundParameterDifferentTypes(key.clone(), get_numeric_type_string(gt_parameters.get(&key.clone()).unwrap()))));
        }
        else if get_value_type(gt_parameters.get(&key.clone()).unwrap()) != get_value_type(&value_arr[1]) {
            return Err(Box::new(SimulatedAnnealingError::BoundParameterDifferentTypes(key.clone(), get_numeric_type_string(gt_parameters.get(&key.clone()).unwrap()))));
        }
        else if get_value_type(gt_parameters.get(&key.clone()).unwrap()) != get_value_type(&value_arr[2]) {
            return Err(Box::new(SimulatedAnnealingError::BoundParameterDifferentTypes(key.clone(), get_numeric_type_string(gt_parameters.get(&key.clone()).unwrap()))));
        }

        if !are_bounds_valid(&value_arr[1], &value_arr[2]) {
            return Err(Box::new(SimulatedAnnealingError::BoundParameterInvalidBounds(key.clone())));
        }

        new_parameter_bounds.insert(key.clone(), (value_arr[0].clone(), value_arr[1].clone(), value_arr[2].clone()));
        // (get_value_type(gt_parameters.get(&key.clone()).unwrap()) != get_value_type(&value)) 
    }

    let new_max_iterations: i64 = tuning_settings_map.get("max_iterations").ok_or(SimulatedAnnealingError::NoMaxIterationsField())?
                                                     .as_i64().ok_or(SimulatedAnnealingError::MaxIterationsNotValidInteger())?;
    if new_max_iterations <= 0 {
        return Err(Box::new(SimulatedAnnealingError::MaxIterationsNotValidInteger()));
    }
    sa_config.max_iterations = Some(new_max_iterations as usize);

    //println!("{:?}", gt_parameters);
    //println!("{:?}", new_parameter_bounds);

    sa_config.parameter_bounds = Some(new_parameter_bounds);
    tuning_config.tuning_type = Some(TuneType::SimulatedAnnealing(sa_config));
    Ok(())
}

fn normalize_weights(metric_weights: &(f32, f32)) -> (f32, f32) {
    let sum = metric_weights.0 + metric_weights.1;

    (metric_weights.0 / sum, metric_weights.1 / sum)
}

fn get_value_type(value: &Value) -> &str {
    if value.is_u64() {
        return "u64";
    }
    else if value.is_i64() {
        return "i64";
    }
    else if value.is_f64() {
        return "f64";
    }
    else if value.is_string() {
        return "String";
    }
    else if value.is_boolean() {
        return "bool";
    }
    else if value.is_array() {
        return "array";
    }
    "something else"
}

fn get_numeric_type_string(value: &Value) -> String {
    if value.is_i64() {
        String::from("i64")
    }
    else if value.is_u64() {
        String::from("u64")
    }
    else {
        String::from("f64")
    }
}

fn are_bounds_valid(lower_bound: &Value, upper_bound: &Value) -> bool {
    if lower_bound.is_u64() {
        lower_bound.as_u64().unwrap() < upper_bound.as_u64().unwrap()
    }
    else if lower_bound.is_i64() {
        lower_bound.as_i64().unwrap() < upper_bound.as_i64().unwrap()
    }
    else {
        lower_bound.as_f64().unwrap() < upper_bound.as_f64().unwrap()
    }
}

fn format_string(arr: Vec<Value>) -> String {
    let mut final_string = String::from("[");
    for i in 0..arr.len() {
        final_string.push_str(&arr[i].to_string());
        if i < arr.len() - 1 {
            final_string.push_str(", ");
        }
    }
    final_string.push_str("]");
    final_string
}