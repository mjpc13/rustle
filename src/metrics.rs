use crate::{errors::{EvoError, RosError}, evo_wrapper::EvoArg, ros_msgs::{Odometry, RosMsg}, task::{Config, TaskOutput}};
//use anyhow::Ok;
use plotters::{backend::{BitMapBackend, SVGBackend}, chart::{ChartBuilder, SeriesLabelPosition}, coord::ranged1d::{IntoSegmentedCoord, SegmentValue}, data::{fitting_range, Quartiles}, drawing::IntoDrawingArea, element::{Boxplot, PathElement, Rectangle}, series::LineSeries, style::{self, Color, IntoFont, Palette, Palette99, RGBColor, BLACK, RED, WHITE}};
use serde::{Deserialize, Serialize};
use bollard::container::{MemoryStats, CPUStats};
use chrono::{DateTime, Utc};

use surrealdb::{
    Surreal,
    engine::any
};
use temp_dir::TempDir;
use core::time;
use std::{collections::{hash_map::Entry, HashMap}, default, env, fs::OpenOptions, io::Write, path::PathBuf, str::FromStr};
use itertools::Itertools;
use std::fmt;


#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ContainerStats {
    pub uid: Option<u32>,
    pub memory_stats: MemoryStats,
    pub cpu_stats: CPUStats,
    pub precpu_stats: CPUStats,
    pub num_procs: u32,
    pub created_at: Option<DateTime<Utc>>,
}

#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub struct Metric{
    max: f32,
    median: f32,
    min: f32,
    rmse: f32,
    sse: f32,
    std: f32
}

impl Metric{
    pub fn compute<R: EvoArg>(data: &TaskOutput, args: &R, output_path: Option<&str>) -> Vec<Result<Metric, EvoError>>{
        
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
                let evo_str = args.compute(
                    path_gt.to_str().unwrap(), 
                    path_clone.to_str().unwrap())?;
                
                Metric::from_str(&evo_str)

            })
            .collect();

        metric_results
        
    }


    pub fn compute_batch<R: EvoArg>(data: &HashMap<&str, Vec<TaskOutput>>, args: R) -> HashMap<String, Vec<Result<Metric, EvoError>>>{
        
        let mut metric_hash: HashMap<String, Vec<Result<Metric, EvoError>>> = HashMap::new();

        for (k, v) in data{
            
            //let mut vec_mec_hash: HashMap<String, Vec<Metric>> = HashMap::new();

            for to in v {
                
                let vec_metric = Metric::compute(to, &args, None);

                if vec_metric.len() == 1{
                    match metric_hash.entry(k.to_string()) {
                        Entry::Vacant(e) => { e.insert(vec_metric); },
                        Entry::Occupied(mut e) => { e.get_mut().push(vec_metric.into_iter().nth(0).unwrap()); }
                    }

                }else{

                    for (m,n) in vec_metric.into_iter().zip(to.odoms.keys().into_iter()){
                        let n1 = n.split("/").last().unwrap();
                        let name = format!("{k} - {n1}");

                        match metric_hash.entry(name) {
                            Entry::Vacant(e) => { e.insert(vec![m]); },
                            Entry::Occupied(mut e) => { e.get_mut().push(m); }
                        }
                    }
                }
            }
        }

        return metric_hash;
    }

    //USE From keyword maybe
    fn average_metric(data: &Vec<Result<Metric, EvoError>>) -> Result<Metric, EvoError> {
        
        let metrics: Vec<&Metric> = data
            .into_iter()
            .filter(|m| m.is_ok())
            .map(|m| m.as_ref().unwrap())
            .collect();

        let size = metrics.len(); //use the filter option


        if size == 0 {
            todo!()
        } else{
            let max: f32 = metrics.iter()
                .map(|m| {
                    m.max
                })
                .sum();
            let max = max / size as f32;

            let median: f32 = metrics.iter()
                .map(|m| {
                    m.median
                })
                .sum();
            let median: f32 = median / size as f32;
            
            let min: f32 = metrics.iter()
            .map(|m| {
                m.min
            })
            .sum();
            let min = min / size as f32;

            let rmse: f32 = metrics.iter()
                .map(|m| {
                    m.rmse
                })
                .sum();
            let rmse = rmse / size as f32;

            let sse: f32 = metrics.iter()
                .map(|m| {
                    m.sse
                })
                .sum();
            let sse = sse / size as f32;

            let std: f32 = metrics.iter()
                .map(|m| {
                    m.std
                })
                .sum();
            let std = std / size as f32;
            
            Ok(Metric { max, median, min, rmse, sse, std})
        }

    }

    fn max_metric(data: Vec<Metric>) -> Metric {

        //TODO: Find a better way to do this!!!
        let max = data.iter()
            .map(|m| {
                m.max
            })
            .fold(std::f32::MIN, |a,b| a.max(b));
        
        let median = data.iter()
            .map(|m| {
                m.median
            })
            .fold(std::f32::MIN, |a,b| a.max(b));
        
        let min = data.iter()
            .map(|m| {
                m.min
            })
            .fold(std::f32::MIN, |a,b| a.max(b));
        
        let rmse = data.iter()
            .map(|m| {
                m.rmse
            })
            .fold(std::f32::MIN, |a,b| a.max(b));

        let sse = data.iter()
            .map(|m| {
                m.sse
            })
            .fold(std::f32::MIN, |a,b| a.max(b));

        let std = data.iter()
            .map(|m| {
                m.std
            })
            .fold(std::f32::MIN, |a,b| a.max(b));

        Metric { max, median, min, rmse, sse, std}


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

    fn to_md(&self, name: &str) -> String {
        let s = format!("| {} | {} | {} | {} | {} | {} | {} |\n", name, self.max, self.median, self.min, self.rmse, self.sse, self.std);
        return s
    }


    pub fn print_batch(data: &HashMap<String, Vec<Result<Metric, EvoError>>>) -> String {
        
        //Computes the average for every run!
        let mut metric_hash: HashMap<String, Result<Metric, EvoError>> = HashMap::new();
        for (name, v) in data{
            let average_metric = Metric::average_metric(v);
            metric_hash.insert(name.to_string(), average_metric);
        }

        let mut evo_md = String::from("| Name | Max | Median | Min | RMSE | SSE | Std |\n|--------|-------|--------|-------|-------|-------|-------|\n");

        for (k, v) in metric_hash{

            evo_md = match v{
                Ok(ve) => evo_md + &ve.to_md(&k),
                Err(e) => evo_md + &format!("| {}ks | - | - | - | - | - | - |\n", &k)
            }
        }

        return evo_md;
    }

    pub fn print(data: &HashMap<String, Result<Metric, EvoError>>) -> String {

        let mut evo_md = String::from("| Name | Max | Median | Min | RMSE | SSE | Std |\n|--------|-------|--------|-------|-------|-------|-------|\n");

        for (k, v) in data{
            evo_md = match v{
                Ok(ve) => evo_md + &ve.to_md(&k),
                Err(e) => evo_md + &format!("| {}ks | - | - | - | - | - | - |\n", &k)
            }
        }

        return evo_md;
    }



    fn get_rmse(data: &Vec<Metric>) -> Vec<f32> {
        let res = data.iter().map(|m| (m.rmse)).collect();
        return res;
    }

    pub fn box_plot<'a>(data: &HashMap<String, Vec<Result<Metric, EvoError>>>, output_file: &'a str){


        let mut data_filtered: HashMap<String, Vec<Metric>> = HashMap::new();

        for (k, v) in data {
            let metric_vec: Vec<Metric> = v
                .into_iter()
                .filter(|m| m.is_ok())
                .map(|m| *m.as_ref().unwrap())
                .collect();

            if metric_vec.len() == v.len(){
                data_filtered.insert(k.to_string(), metric_vec);
            } else {
                //Some runs have failed
                data_filtered.insert(format!("{:}*", k), metric_vec);
            }
        };

        let names: Vec<String> = data_filtered.keys().cloned().collect();


        let root = SVGBackend::new(output_file, (800, 600)).into_drawing_area();
        root.fill(&WHITE).unwrap();

        let quartiles: Vec<Quartiles> = data_filtered
            .iter()
            .map(|(s,v)| {
                Quartiles::new(&Metric::get_rmse(v))
            })
            .collect();

        let y_min = quartiles.iter()
            .map(|q| q.values()[0])
            .fold(std::f32::MAX, |a,b| a.min(b));

        let y_max = quartiles.iter()
            .map(|q| q.values()[0])
            .fold(std::f32::MIN, |a,b| a.max(b));
        
        let y_min = y_min - 0.1*y_min;
        let y_max = y_max + 0.1*y_max;

        let mut chart = ChartBuilder::on(&root)
            .x_label_area_size(50)
            .y_label_area_size(50)
            .caption("Boxplot of RMSE", ("sans-serif", 24).into_font())
            .build_cartesian_2d(
                names[..].into_segmented(),
                y_min..y_max,
            ).unwrap();

        chart.configure_mesh()
            .y_desc("APE RMSE")
            .axis_desc_style(("sans-serif", 18))
            .draw()
            .unwrap();

        let boxplot: Vec<Boxplot<_,_>> = quartiles
            .iter()
            .zip_eq(names.iter())
            .map(|(q, n)| {
                Boxplot::new_vertical(SegmentValue::CenterOf(n), q)
            })
            .collect();

        chart.draw_series(boxplot).unwrap();
        
        root.present().unwrap();
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
    pub fn plot(&self, data: &Vec<TaskOutput>, output_file: &str){

        match self {
            Self::MemoryUsage => {
                self.plot_memory(data, output_file)
            },
            Self::MemoryUsagePerSec => {
                self.plot_memory_per_sec(data, output_file)
            },
            Self::Load => {
                //self.plot_load(data, output_file)
            },
            Self::LoadPercentage => {
               self.plot_load_percentage(data, output_file)
            }
        }
    }

    fn plot_memory(&self, data: &Vec<TaskOutput>, output_file: &str){

        let root = SVGBackend::new(output_file, (800, 600)).into_drawing_area();
        root.fill(&WHITE).unwrap();

        let x_max = (data[0].stats.last().unwrap().created_at.unwrap() - data[0].stats[0].created_at.unwrap()).num_seconds();

        //Get maximum value for Memory usage
        let y_max = data.iter()
            .map(|d| {
                d.stats.iter()
                .fold(std::f64::MIN, |a,b| a.max(b.memory_stats.usage.unwrap() as f64 * 1e-6))
            })
            .fold(std::f64::MIN, |a,b| a.max(b));

            
        let starting_time: Vec<DateTime<Utc>> = data.iter().map(|d| {    
            d.stats[0].created_at.unwrap()
        }).collect();

        let mut chart = ChartBuilder::on(&root)
            .caption("Memory Usage (MiB)", ("sans-serif", 24).into_font())
            .margin(5)
            .x_label_area_size(50)
            .y_label_area_size(50)
            .build_cartesian_2d(0i64..x_max, 0f64..y_max).unwrap();


    

        chart.configure_mesh()
            .y_desc("Memory Usage (MiB)")
            .x_desc("Time (s)")
            .axis_desc_style(("sans-serif", 18))
            .draw()
            .unwrap();
        
        let _: Vec<_> = data
            .iter()
            .enumerate()
            .map(|(idx, d)|{
                let color = Palette99::pick(idx).mix(0.9);

                
                chart
                .draw_series(LineSeries::new(
                        d.stats.iter().map(|s| 
                            (
                                (s.created_at.unwrap() - starting_time[idx]).num_seconds() as i64, 
                                s.memory_stats.usage.unwrap() as f64 * 1e-6
                            )
                        ),
                        color.stroke_width(3)
                )).unwrap()
                .label(&d.name)
                .legend(move |(x, y)| Rectangle::new([(x, y - 5), (x + 10, y + 5)], color.filled()));

                
            })
            .collect();
        
        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperRight)
            .background_style(&WHITE.mix(0.8))
            .border_style(&BLACK)
            .draw().unwrap();
        
        root.present().unwrap();
    }

    fn plot_memory_per_sec(&self, data: &Vec<TaskOutput>, output_file: &str){
        let root = SVGBackend::new(output_file, (800, 600)).into_drawing_area();
        root.fill(&WHITE).unwrap();

        println!("Hello there");
        
        let mem_sec: Vec<Vec<f64>> = data
            .iter()
            .map(|to|{
                to.stats
                    .iter()
                    .tuple_windows()
                    .map(|(m1, m2)| {
                        let memory_diff = (m2.memory_stats.usage.unwrap() as f64 - m1.memory_stats.usage.unwrap() as f64);
                        let time_diff = m2.created_at.unwrap() - m1.created_at.unwrap();
        
                        let usage_per_sec = memory_diff * 1e-6 / (time_diff.num_seconds() as f64);
        
                        usage_per_sec
        
                    })
                    .collect()
            })
            .collect();


        //Get the maximum memory per second
        let y_max = mem_sec.iter()
            .map(|d| {
                d.iter()
                .filter(|&&a| a < 100.0)
                .fold(std::f64::MIN, |a,b| a.max(*b))
            })
            .fold(std::f64::MIN, |a,b| a.max(b));

        //Get the maximum memory per second
        let y_min = mem_sec.iter()
        .map(|d| {
            d.iter()
            .fold(std::f64::MAX, |a,b| a.min(*b))
        })
        .fold(std::f64::MAX, |a,b| a.min(b));

        println!("Hello ");

        let x_max = (data[0].stats.last().unwrap().created_at.unwrap() - data[0].stats[0].created_at.unwrap()).num_seconds();
        //Get starting time of each Task
        
        let starting_time: Vec<DateTime<Utc>> = data.iter().map(|d| {    
            d.stats[0].created_at.unwrap()
        }).collect();
        println!("man ");

        let mut chart = ChartBuilder::on(&root)
            .caption("Memory Usage Per Second (MiB/s)", ("sans-serif", 24).into_font())
            .margin(5)
            .x_label_area_size(50)
            .y_label_area_size(50)
            .build_cartesian_2d(0i64..x_max, y_min..y_max).unwrap();
        
        println!("i have no ");

        chart.configure_mesh()
            .y_desc("MiB/s")
            .x_desc("Time (s)")
            .axis_desc_style(("sans-serif", 18))
            .draw()
            .unwrap();
        
            println!("i have no ");


            let _: Vec<_> = data
            .iter()
            .zip_eq(mem_sec.into_iter())
            .enumerate()
            .map(|(idx, (d, m))|{
                println!("idea ");

                let color = Palette99::pick(idx).mix(0.9);
                chart
                .draw_series(
                    LineSeries::new(
                        d.stats
                            .iter()
                            .skip(1)
                            .zip_eq(m.into_iter())
                            .map(|(s, l)| 
                                (
                                    (s.created_at.unwrap() - starting_time[idx]).num_seconds() as i64, 
                                    l
                                )
                            ), 
                        color.stroke_width(3)
                )).unwrap()
                .label(&d.name)
                .legend(move |(x, y)| Rectangle::new([(x, y - 5), (x + 10, y + 5)], color.filled()));
            println!("fu ");

            })
            .collect();


            println!("me ");

        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperRight)
            .background_style(&WHITE.mix(0.8))
            .border_style(&BLACK)
            .draw().unwrap();
        
        root.present().unwrap();
    }

    fn plot_load_percentage(&self, data: &Vec<TaskOutput>, output_file: &str){
        let root = SVGBackend::new(output_file, (800, 600)).into_drawing_area();
        root.fill(&WHITE).unwrap();


        let load_perc: Vec<Vec<f64>> = data
            .iter()
            .map(|to|{
                to.stats
                    .iter()
                    .skip(1)
                    .map(|cs|{
                        let load_used = (cs.cpu_stats.cpu_usage.total_usage - cs.precpu_stats.cpu_usage.total_usage) as f64;
                
                        let sys_now = cs.cpu_stats.system_cpu_usage.unwrap() as f64;
                        let sys_prev = cs.precpu_stats.system_cpu_usage.unwrap() as f64;
        
        
                        let available = (cs.cpu_stats.system_cpu_usage.unwrap() - cs.precpu_stats.system_cpu_usage.unwrap()) as f64;
                        
                        
                        let load_perc = load_used / available * 100.0 * cs.cpu_stats.online_cpus.unwrap() as f64; 
        
                        load_perc
                    })
                    .collect()
            })
            .collect();

        //Get the maximum percentage (if it lower than 100%, use 100% as the maximum value)
        let y_max = load_perc.iter()
            .map(|d| {
                d.iter()
                .fold(std::f64::MIN, |a,b| a.max(*b))
            })
            .fold(std::f64::MIN, |a,b| a.max(b));

        let y_max: f64 = match y_max >= 100.0{
            true => y_max,
            false => 100.0
        };

        //Get the max time
        let x_max = (data[0].stats.last().unwrap().created_at.unwrap() - data[0].stats[0].created_at.unwrap()).num_seconds();
        
        //Get starting time of each Task
        let starting_time: Vec<DateTime<Utc>> = data.iter().map(|d| {    
            d.stats[0].created_at.unwrap()
        }).collect();


        let mut chart = ChartBuilder::on(&root)
            .caption("Computer CPU", ("sans-serif", 24).into_font())
            .margin(5)
            .x_label_area_size(50)
            .y_label_area_size(50)
            .build_cartesian_2d(0i64..x_max, 0f64..y_max)
            .unwrap();

        chart.configure_series_labels()
             .position(SeriesLabelPosition::UpperRight);

        chart
            .configure_mesh()
            .y_desc("CPU Load (%)")
            .x_desc("Time (s)")
            .axis_desc_style(("sans-serif", 18))
            .draw()
            .unwrap();
        

        let _: Vec<_> = data
            .iter()
            .zip_eq(load_perc.into_iter())
            .enumerate()
            .map(|(idx, (d, l))|{
                let color = Palette99::pick(idx).mix(0.9);
                chart
                .draw_series(
                    LineSeries::new(
                        d.stats
                            .iter()
                            .skip(1)
                            .zip_eq(l.into_iter())
                            .map(|(s, l)| 
                                (
                                    (s.created_at.unwrap() - starting_time[idx]).num_seconds() as i64, 
                                    l
                                )
                            ), 
                        color.stroke_width(3)
                )).unwrap()
                .label(&d.name)
                .legend(move |(x, y)| Rectangle::new([(x, y - 5), (x + 10, y + 5)], color.filled()));
            })
            .collect();
        


        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperRight)
            .background_style(&WHITE.mix(0.8))
            .border_style(&BLACK)
            .draw().unwrap();
        
        root.present().unwrap();
    }


    fn plot_load(&self, data: &Vec<ContainerStats>, output_path: &str){
        todo!()
    }
}