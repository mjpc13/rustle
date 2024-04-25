use crate::{errors::{EvoError, RosError}, evo_wrapper::EvoArg, ros_msgs::{Odometry, RosMsg}, task::{Config, TaskOutput}};
use plotters::{backend::BitMapBackend, chart::ChartBuilder, drawing::IntoDrawingArea, element::PathElement, series::LineSeries, style::{Color, IntoFont, RGBColor, BLACK, RED, WHITE}};
use serde::{Deserialize, Serialize};
use bollard::container::{MemoryStats, CPUStats};
use chrono::{DateTime, Utc};

use surrealdb::{
    Surreal,
    engine::any
};
use temp_dir::TempDir;
use core::time;
use std::{collections::HashMap, env, fs::OpenOptions, io::Write, path::PathBuf, str::FromStr};
use itertools::Itertools;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ContainerStats {
    pub uid: Option<u32>,
    pub memory_stats: MemoryStats,
    pub cpu_stats: CPUStats,
    pub precpu_stats: CPUStats,
    pub num_procs: u32,
    pub created_at: Option<DateTime<Utc>>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Metric{
    max: f32,
    median: f32,
    min: f32,
    rmse: f32,
    sse: f32,
    std: f32
}

impl Metric{
    pub fn compute<R: EvoArg>(data: &TaskOutput, args: R, output_path: Option<&str>) -> Result<Vec<Metric>, EvoError>{
        
        //Write TUM files to temporary directory
        let mut path = match output_path{
            Some(p) => {
                PathBuf::from(p)
            },
            None => {
                env::temp_dir()
            }
        };
        let mut path_gt = path.clone();
        
        Self::write_file(&data.groundtruth, "groundtruth", &mut path_gt);

        let metric_results: Vec<_> = data.odoms
            .iter()
            .map( |(s, v)| {
                let mut path_clone = path.clone();

                Self::write_file(v, &s, &mut path_clone);
                let evo_str = args.compute(path_gt.to_str().unwrap(), path_clone.to_str().unwrap()).unwrap();
                Metric::from_str(&evo_str).unwrap()

            })
            .collect();

        Ok(metric_results)
        
    }

    fn write_file<'a>(data: &Vec<Odometry>, name: &str, path: &mut PathBuf) -> Result<(), std::io::Error>{

        path.push(name.replace("/", "_"));

        let mut file = OpenOptions::new()
            .write(true)
            .append(false)
            .create(true)
            .truncate(true)
            .open(path)?;
        
        let _: Vec<_> = data.iter()
            .map(|o|{
                if let Err(e) = writeln!(file, "{}", o ){
                    eprintln!("Couldn't write to file: {}", e);
                }
            })
            .collect();
        Ok(())
    }
}

impl FromStr for Metric {
    type Err = EvoError;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let mut hash: HashMap<&str, f32> = HashMap::new();

        let data_vec: Vec<_> = s.split("\n")
            .filter(|&s| s.contains("\t"))
            .map(|s| s.split("\t"))
            .flatten()
            .map(|s| s.trim())
            .batching( |it| {
                match it.next() {
                    None => None,
                    Some(x) => match it.next() {
                        None => None,
                        Some(y) => Some((x, y)),
                    }
                }
            }
            )
            .map(|(k, v)| {
                let n = v.parse::<f32>().unwrap();
                    //.unwrap_or(
                    //return Err::<(),RosError>(
                    //        RosError::ParseError { 
                    //            name: "asd".into(), 
                    //            value: "dsa".into() 
                    //        }
                    //    )
                    //);
                    hash.insert(k, n);
            }
        )
        .collect();

        return Ok(Metric{
            max: hash["max"],
            median: hash["median"],
            min: hash["min"],
            rmse: hash["rmse"],
            sse: hash["sse"],
            std: hash["std"]
        })
    }
}

#[derive(Clone)]
pub struct ContainerMetrics{
    stats: Vec<ContainerStats>
}

pub struct ContainerMemory<'a>{
    resolution: (u32, u32),
    bg_color: RGBColor,
    line_color: RGBColor,
    border_color: RGBColor,
    legend: &'a str,
}

pub enum ContainerPlot{
    MemoryUsage,
    MemoryUsagePerSec,
    Load,
    LoadPercentage
}

pub struct ContainerPlotArg<'a>{
    resolution: (u32, u32),
    bg_color: RGBColor,
    line_color: RGBColor,
    border_color: RGBColor,
    caption: &'a str,
    caption_style: (&'a str, u32),
    legend: &'a str,
}


impl ContainerPlot {
    pub fn plot(&self, data: &Vec<ContainerStats>, output_file: &str){

        match self {
            Self::MemoryUsage => {
                self.plot_memory(data, output_file)
            },
            Self::MemoryUsagePerSec => {
                self.plot_memory_per_sec(data, output_file)
            },
            Self::Load => {
                self.plot_load(data, output_file)
            },
            Self::LoadPercentage => {
               self.plot_load_percentage(data, output_file)
            }
        }
    }

    fn plot_memory(&self, data: &Vec<ContainerStats>, output_file: &str){

        let root = BitMapBackend::new(output_file, (1920, 1080)).into_drawing_area();
        root.fill(&WHITE).unwrap();

        let x_max = (data.last().unwrap().created_at.unwrap() - data[0].created_at.unwrap()).num_seconds();
        let y_min = data[0].memory_stats.usage.unwrap() as f64 * 1e-6;
        let y_max = data.last().unwrap().memory_stats.usage.unwrap() as f64 * 1e-6;


        let mut chart = ChartBuilder::on(&root)
            .caption("Memory Usage (MiB)", ("sans-serif", 50).into_font())
            .margin(5)
            .x_label_area_size(50)
            .y_label_area_size(50)
            .build_cartesian_2d(0i64..x_max, y_min..y_max).unwrap();


    

        chart.configure_mesh()
            .y_desc("Memory Usage (MiB)")
            .x_desc("Time (s)")
            .axis_desc_style(("sans-serif", 21))
            .draw()
            .unwrap();
        
        chart
            .draw_series(LineSeries::new(
                    data.clone().into_iter().map(|s| 
                        (
                            //(s.created_at.unwrap() - data[0].created_at.unwrap()).num_seconds(),
                            (s.created_at.unwrap() - data[0].created_at.unwrap()).num_seconds() as i64, 
                            s.memory_stats.usage.unwrap() as f64 * 1e-6
                        )
                    ),
                &RED,
            )).unwrap()
            .label("LIO-SAM")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));
        
        chart
            .configure_series_labels()
            .background_style(&WHITE.mix(0.8))
            .border_style(&BLACK)
            .draw().unwrap();
        
        root.present().unwrap();
    }

    fn plot_memory_per_sec(&self, data: &Vec<ContainerStats>, output_file: &str){
        let root = BitMapBackend::new(output_file, (1920, 1080)).into_drawing_area();
        root.fill(&WHITE).unwrap();

        let mem_sec: Vec<f64> = data
            .iter()
            .tuple_windows()
            .map(|(m1, m2)| {
                let memory_diff = (m2.memory_stats.usage.unwrap() as f64 - m1.memory_stats.usage.unwrap() as f64);
                let time_diff = m2.created_at.unwrap() - m1.created_at.unwrap();

                let usage_per_sec = memory_diff * 1e-6 / (time_diff.num_seconds() as f64);

                usage_per_sec

            })
            .collect();

        let x_max = (data.last().unwrap().created_at.unwrap() - data[0].created_at.unwrap()).num_seconds();
        let y_min = *mem_sec.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap_or(&f64::from(0)); 
        let y_max = *mem_sec.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap_or(&f64::from(100));


        let mut chart = ChartBuilder::on(&root)
            .caption("Memory Usage Per Second (MiB/s)", ("sans-serif", 50).into_font())
            .margin(5)
            .x_label_area_size(50)
            .y_label_area_size(50)
            .build_cartesian_2d(0i64..x_max, y_min..y_max).unwrap();

        chart.configure_mesh()
            .y_desc("Memory Usage Per Second (MiB/s)")
            .x_desc("Time (s)")
            .axis_desc_style(("sans-serif", 21))
            .draw()
            .unwrap();
        
        chart
            .draw_series(LineSeries::new(
                    data.clone()
                    .into_iter()
                    .step_by(2)
                    .zip(mem_sec.into_iter())
                    .map(|(s, m)| 
                        (
                            (s.created_at.unwrap() - data[0].created_at.unwrap()).num_seconds() as i64, 
                            m
                        )
                    ),
                &RED,
            )).unwrap()
            .label("LIO-SAM")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));
        
        chart
            .configure_series_labels()
            .background_style(&WHITE.mix(0.8))
            .border_style(&BLACK)
            .draw().unwrap();
        
        root.present().unwrap();
    }

    fn plot_load_percentage(&self, data: &Vec<ContainerStats>, output_file: &str){
        let root = BitMapBackend::new(output_file, (1920, 1080)).into_drawing_area();
        root.fill(&WHITE).unwrap();

        let load_perc: Vec<f64> = data
            .iter()
            .skip(1)
            .map(|cs| {

                let load_used = (cs.cpu_stats.cpu_usage.total_usage - cs.precpu_stats.cpu_usage.total_usage) as f64;
                
                let sys_now = cs.cpu_stats.system_cpu_usage.unwrap() as f64;
                let sys_prev = cs.precpu_stats.system_cpu_usage.unwrap() as f64;


                let available = (cs.cpu_stats.system_cpu_usage.unwrap() - cs.precpu_stats.system_cpu_usage.unwrap()) as f64;
                
                
                let load_perc = load_used / available * 100.0 * cs.cpu_stats.online_cpus.unwrap() as f64; 

                load_perc

            })
            .collect();



        let x_max = (data.last().unwrap().created_at.unwrap() - data[0].created_at.unwrap()).num_seconds();


        let mut chart = ChartBuilder::on(&root)
            .caption("Computer CPU", ("sans-serif", 50).into_font())
            .margin(5)
            .x_label_area_size(50)
            .y_label_area_size(50)
            .build_cartesian_2d(0i64..x_max, 0f64..100f64).unwrap();



        chart.configure_mesh()
            .y_desc("CPU Load (%)")
            .x_desc("Time (s)")
            .axis_desc_style(("sans-serif", 21))
            .draw()
            .unwrap();
        
        chart
            .draw_series(LineSeries::new(
                    data.clone()
                    .into_iter()
                    .skip(1)
                    .zip_eq(load_perc.into_iter())
                    .map(|(s, l)| 
                        (
                            (s.created_at.unwrap() - data[0].created_at.unwrap()).num_seconds() as i64, 
                            l
                        )
                    ),
                &RED,
            )).unwrap()
            .label("LIO-SAM")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));
        


        chart
            .configure_series_labels()
            .background_style(&WHITE.mix(0.8))
            .border_style(&BLACK)
            .draw().unwrap();
        
        root.present().unwrap();
    }



    fn plot_load(&self, data: &Vec<ContainerStats>, output_path: &str){
        todo!()
    }
}