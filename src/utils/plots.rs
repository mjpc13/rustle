//Util to create plots

use std::collections::HashMap;

use chrono::{DateTime, Utc};
use plotters::{chart::ChartBuilder, data::Quartiles, prelude::{IntoDrawingArea, SVGBackend}, style::{IntoFont, WHITE}};

use crate::models::metrics::{ContainerStats, PoseErrorMetrics};


//Box Plot of the APE of the multiple methods. This shows
fn box_plot_pose_metrics(data: Vec<PoseErrorMetrics>, file: String){

    todo!();

}


//pub fn box_plot<'a>(data: &HashMap<String, Vec<PoseErrorMetrics>>, file_path: &'a str){
//
//
//    let root = SVGBackend::new(file_path, (800, 600)).into_drawing_area();
//    root.fill(&WHITE).unwrap();
//
//
//
//
//    let quartiles: Vec<Quartiles> = data_filtered
//        .iter()
//        .map(|(s,v)| {
//            Quartiles::new(&Metric::get_rmse(v))
//        })
//        .collect();
//
//    //Get the minimum and maximum Y to know the window size
//    let y_min = quartiles.iter()
//        .map(|q| q.values()[0])
//        .fold(std::f32::MAX, |a,b| a.min(b));
//
//    let y_max = quartiles.iter()
//        .map(|q| q.values()[0])
//        .fold(std::f32::MIN, |a,b| a.max(b));
//    
//    let y_min = y_min - 0.1*y_min;
//    let y_max = y_max + 0.1*y_max;
//
//    let mut chart = ChartBuilder::on(&root)
//        .x_label_area_size(50)
//        .y_label_area_size(80)
//        .caption("Boxplot of RMSE", ("sans-serif", 24).into_font())
//        .margin(8)
//        .build_cartesian_2d(
//            names[..].into_segmented(),
//            y_min..y_max,
//        ).unwrap();
//
//    chart.configure_mesh()
//        .y_desc("APE RMSE")
//        .axis_desc_style(("sans-serif", 20))
//        .label_style(("sans-serif", 14).into_font())
//        .draw()
//        .unwrap();
//
//    let boxplot: Vec<Boxplot<_,_>> = quartiles
//        .iter()
//        .zip_eq(names.iter())
//        .map(|(q, n)| {
//            Boxplot::new_vertical(SegmentValue::CenterOf(n), q)
//        })
//        .collect();
//
//    chart.draw_series(boxplot).unwrap();
//    
//    root.present().unwrap();
//}









//Box Plot of the RPE of the multiple methods


//Box Plot of the Frequencies of the multiple methods









//Function to draw the CPU load of a Vector of stats
//fn plot_load_percentage(data: &Vec<ContainerStats>, output_file: &str){
//    let root = SVGBackend::new(output_file, (800, 600)).into_drawing_area();
//    root.fill(&WHITE).unwrap();
//
//    let load_perc: Vec<Vec<f64>> = data
//        .iter()
//        .map(|to|{
//            to
//                .iter()
//                .skip(1)
//                .map(|cs|{
//                    let load_used = (cs.cpu_stats.cpu_usage.total_usage - cs.precpu_stats.cpu_usage.total_usage) as f64;
//            
//                    let sys_now = cs.cpu_stats.system_cpu_usage.unwrap() as f64;
//                    let sys_prev = cs.precpu_stats.system_cpu_usage.unwrap() as f64;
//    
//    
//                    let available = (cs.cpu_stats.system_cpu_usage.unwrap() - cs.precpu_stats.system_cpu_usage.unwrap()) as f64;
//                    
//                    
//                    let load_perc = load_used / available * 100.0 * cs.cpu_stats.online_cpus.unwrap() as f64; 
//    
//                    load_perc
//                })
//                .collect()
//        })
//        .collect();
//
//    //Get the maximum percentage (if it lower than 100%, use 100% as the maximum value)
//    let y_max = load_perc.iter()
//        .map(|d| {
//            d.iter()
//            .fold(std::f64::MIN, |a,b| a.max(*b))
//        })
//        .fold(std::f64::MIN, |a,b| a.max(b));
//
//    let y_max: f64 = match y_max >= 100.0{
//        true => y_max,
//        false => 100.0
//    };
//
//    //Get the max time
//    let x_max = (data.last().unwrap().created_at - data[0].created_at).num_seconds();
//    
//    //Get starting time of each Task
//    let starting_time: Vec<DateTime<Utc>> = data.iter().map(|d| {    
//        d[0].created_at.unwrap()
//    }).collect();
//
//
//    let mut chart = ChartBuilder::on(&root)
//        .caption("Computer CPU", ("sans-serif", 24).into_font())
//        .margin(8)
//        .x_label_area_size(50)
//        .y_label_area_size(80)
//        .build_cartesian_2d(0i64..x_max, 0f64..y_max)
//        .unwrap();
//
//    chart
//        .configure_mesh()
//        .y_desc("CPU Load (%)")
//        .x_desc("Time (s)")
//        .axis_desc_style(("sans-serif", 20))
//        .label_style(("sans-serif", 18).into_font())
//        .draw()
//        .unwrap();
//    
//
//    let _: Vec<_> = data
//        .iter()
//        .zip_eq(load_perc.into_iter())
//        .enumerate()
//        .map(|(idx, (d, l))|{
//            let color = Palette99::pick(idx).mix(0.9);
//            chart
//            .draw_series(
//                LineSeries::new(
//                    d.stats
//                        .iter()
//                        .skip(1)
//                        .zip_eq(l.into_iter())
//                        .map(|(s, l)| 
//                            (
//                                (s.created_at.unwrap() - starting_time[idx]).num_seconds() as i64, 
//                                l
//                            )
//                        ), 
//                    color.stroke_width(3)
//            )).unwrap()
//            .label(&d.name)
//            .legend(move |(x, y)| Rectangle::new([(x, y - 5), (x + 10, y + 5)], color.filled()));
//        })
//        .collect();
//    
//
//
//    chart
//        .configure_series_labels()
//        .position(SeriesLabelPosition::UpperRight)
//        .label_font(("sans-serif", 14))
//        .background_style(&WHITE.mix(0.8))
//        .border_style(&BLACK)
//        .draw().unwrap();
//    
//    root.present().unwrap();
//}