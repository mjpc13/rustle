//Util to create plots

use core::f64;
use std::collections::{BTreeMap, HashMap};

use chrono::{format::Item, DateTime, Utc};
use log::warn;
use crate::{models::{metrics::{pose_error::{APE, RPE}, ContainerStats, PoseErrorMetrics}, Algorithm, TestDefinition, TestType}, services::error::PlotError};

use rand::Rng;

use charming::{
    component::{Axis, Legend}, datatype::{Dataset, Transform}, element::{AreaStyle, AxisLabel, AxisType, ItemStyle, Label, LabelPosition, LineStyle, LineStyleType, MarkArea, MarkAreaData, MarkLine, MarkLineData, MarkLineVariant, Orient, SplitArea, SplitLine, Symbol, TextStyle}, series::{Boxplot, Graph, Line}, theme::Theme, Chart, ImageRenderer
};


#[derive(Debug)]
struct DataItem{
    time: i32,
    value: f64,
    l: f64,
    u: f64
}

//PLOTS FOR A SINGLE ITERATION!!!

pub fn cpu_load_line_chart(data: &Vec<ContainerStats>) -> Result<Chart, PlotError> {
    let data_ts: Vec<DateTime<Utc>> = data.iter().map(|cs| cs.created_at).collect();
    let start_ts = data_ts[0];

    let time_sec: Vec<f64> = data_ts
        .iter()
        .map(|ts| (*ts - start_ts).num_seconds() as f64)
        .collect();

    let cpu_load: Result<Vec<f64>, PlotError> = data.iter()
        .skip(2)
        .map(|cs| {

            let total_usage = cs.cpu_stats.cpu_usage.total_usage;
            let prev_usage = cs.precpu_stats.cpu_usage.total_usage;
            let system_cpu = cs.cpu_stats.system_cpu_usage.ok_or(PlotError::MissingData("CPU usage".to_owned()))?;
            let prev_system_cpu = cs.precpu_stats.system_cpu_usage.ok_or(PlotError::MissingData("Pre CPU usage".to_owned()))?;
            let online_cpus = cs.cpu_stats.online_cpus.ok_or(PlotError::MissingData("Online CPUs".to_owned()))? as f64;

            let used = (total_usage - prev_usage) as f64;
            let available = (system_cpu - prev_system_cpu) as f64;
            Ok(used / available * 100.0 * online_cpus)

        })
        .collect();

    let cpu_load = cpu_load?;

    let mean = if !cpu_load.is_empty() {
        cpu_load.iter().sum::<f64>() / cpu_load.len() as f64
    } else {
        0.0
    };

    let max_load = cpu_load.iter()
        .filter(|&&x| x.is_finite())
        .fold(f64::NAN, |a, &b| if a.is_nan() { b } else { a.max(b) });

    let max_y = if max_load >= 100.0 { max_load.floor() + 5.0 } else { 100.0 };

    let std_dev = if cpu_load.len() > 1 {
        let variance = cpu_load.iter()
            .map(|&x| (x - mean).powi(2))
            .sum::<f64>() / (cpu_load.len() - 1) as f64;
        variance.sqrt()
    } else {
        0.0
    };

    let xy_data: Vec<Vec<f64>> = time_sec.iter().zip(&cpu_load).map(|(x, y)| vec![*x, *y]).collect();

    let chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Time (s)")
                .name_text_style(TextStyle::new().font_size(16).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(14)),
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("CPU Load (%)")
                .max(max_y)
                .name_text_style(TextStyle::new().font_size(16).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(14)),
        )
        .series(
            Line::new()
                .name("CPU Load Over Time")
                .data(xy_data)
                .smooth(true)
                .line_style(LineStyle::new().width(3).color("rgba(52, 152, 219, 0.9)")) // a soft blue
                .item_style(ItemStyle::new().color("rgba(41, 128, 185, 0.6)")) // point color
                .mark_area(
                    MarkArea::new()
                        .item_style(ItemStyle::new().color("rgba(127, 158, 249, 0.15)")) // more subtle
                        .label(Label::new().show(false))
                        .data(vec![
                            (
                                MarkAreaData::new().name("1σ Range").y_axis((mean - std_dev).to_string()),
                                MarkAreaData::new().y_axis((mean + std_dev).to_string()),
                            ),
                        ]),
                )
                .mark_line(
                    MarkLine::new()
                        .line_style(
                            LineStyle::new()
                                .type_(LineStyleType::Dashed)
                                .width(2)
                                .color("rgba(231, 76, 60, 0.8)"),
                        )
                        .label(Label::new().show(true).formatter("Mean"))
                        .symbol(vec![Symbol::None, Symbol::None])
                        .data(vec![
                            MarkLineVariant::Simple(MarkLineData::new().name("Mean").y_axis(mean))
                        ]),
                )
        );

    //let filename = "cpu_load.svg";
    //let full_path = format!("{file_path}/{filename}");
    //let mut renderer = ImageRenderer::new(1000, 800)
    //    .theme(Theme::Infographic);
    //renderer.save(&chart, full_path);

    Ok(chart)

}

pub fn memory_usage_line_chart(data: &Vec<ContainerStats>, file_path: &str) {
    let data_ts: Vec<DateTime<Utc>> = data.iter().map(|cs| cs.created_at).collect();
    let start_ts = data_ts[0];

    let time_sec: Vec<f64> = data_ts
        .iter()
        .map(|ts| (*ts - start_ts).num_seconds() as f64)
        .collect();

    let memory_usage: Vec<f64> = data.iter()
        .map(|cs| cs.memory_stats.usage.unwrap_or(0) as f64 / 1_000_000_000.0)
        .collect();

    let mean = if !memory_usage.is_empty() {
        memory_usage.iter().sum::<f64>() / memory_usage.len() as f64
    } else {
        0.0
    };

    let xy_data: Vec<Vec<f64>> = time_sec
        .iter()
        .zip(&memory_usage)
        .map(|(x, y)| vec![*x, *y])
        .collect();

    let chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Time (s)")
                .name_text_style(TextStyle::new().font_size(16).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(14)),
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Memory Usage (GB)")
                .name_text_style(TextStyle::new().font_size(16).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(14)),
        )
        .series(
            Line::new()
                .name("Memory Usage Over Time")
                .data(xy_data)
                .smooth(true)
                .line_style(LineStyle::new().width(3).color("rgba(46, 204, 113, 0.85)"))
                .item_style(ItemStyle::new().color("rgba(39, 174, 96, 0.5)"))
                .mark_line(
                    MarkLine::new()
                        .line_style(
                            LineStyle::new()
                                .type_(LineStyleType::Dashed)
                                .width(2)
                                .color("rgba(22, 160, 133, 0.8)"),
                        )
                        .label(Label::new().show(false))
                        .symbol(vec![Symbol::None, Symbol::None])
                        .data(vec![
                            MarkLineVariant::Simple(
                                MarkLineData::new().y_axis(mean)
                            )
                        ]),
                )
        );

    let filename = "memory_usage.svg";
    let full_path = format!("{}/{}", file_path, filename);

    let mut renderer = ImageRenderer::new(1000, 800)
        .theme(Theme::Infographic);
    renderer.save(&chart, full_path);
}

pub fn ape_line_chart(data: &Vec<APE>, test_definition: &TestDefinition) -> Result<Chart, PlotError> {
    let time: Vec<f64> = data.iter().map(|ape| ape.time_from_start).collect();
    let ape_values: Vec<f64> = data.iter().map(|ape| ape.value).collect();

    let mean = if !ape_values.is_empty() {
        ape_values.iter().sum::<f64>() / ape_values.len() as f64
    } else {
        0.0
    };

    let area_data = get_area_from_def(test_definition);

    let xy_data: Vec<Vec<f64>> = time
        .iter()
        .zip(&ape_values)
        .map(|(x, y)| vec![*x, *y])
        .collect();

    let mut chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Time (s)")
                .name_gap(25)
                .name_text_style(TextStyle::new().font_size(14))
                .axis_label(AxisLabel::new().font_size(12))
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("APE (m)")
                .name_gap(30)
                .name_text_style(TextStyle::new().font_size(14))
                .axis_label(AxisLabel::new().font_size(12))
        )
        .series(
            Line::new()
                .name("APE Over Time")
                .smooth(true)
                .line_style(LineStyle::new()
                    .color("rgba(52, 152, 219, 0.85)")
                    .width(3)
                    .type_(LineStyleType::Solid))
                .item_style(ItemStyle::new()
                    .color("rgba(41, 128, 185, 1.0)")
                    .border_color("rgba(41, 128, 185, 1.0)")
                    .border_width(1))
                .symbol(Symbol::Circle)
                .symbol_size(6)
                .data(xy_data)
                .mark_line(
                    MarkLine::new()
                        .line_style(LineStyle::new()
                            .type_(LineStyleType::Dashed)
                            .color("rgba(52, 152, 219, 0.5)")
                            .width(2))
                        .label(Label::new()
                            .show(true)
                            .formatter("Mean")
                            .color("#555")
                            .font_size(12))
                        .symbol(vec![Symbol::None, Symbol::None])
                        .data(vec![
                            MarkLineVariant::Simple(
                                MarkLineData::new()
                                    .name("Mean")
                                    .y_axis(mean)
                            )
                        ])
                )
        );

    chart = add_areas_markers(chart, area_data);

    //let filename = "ape.svg";
    //let full_path = format!("{file_path}/{filename}");
    //let mut renderer = ImageRenderer::new(1000, 800).theme(Theme::Infographic);
    //renderer.save(&chart, full_path);

    Ok(chart)

}

pub fn rpe_line_chart(data: &Vec<RPE>, test_definition: &TestDefinition, file_path: &String) {
    let time: Vec<f64> = data.iter().map(|rpe| rpe.time_from_start).collect();
    let rpe_values: Vec<f64> = data.iter().map(|rpe| rpe.value).collect();

    let mean = if !rpe_values.is_empty() {
        rpe_values.iter().sum::<f64>() / rpe_values.len() as f64
    } else {
        0.0
    };

    let area_data = get_area_from_def(&test_definition);

    let xy_data: Vec<Vec<f64>> = time
        .iter()
        .zip(&rpe_values)
        .map(|(x, y)| vec![*x, *y])
        .collect();

    let mut chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Time (s)")
                .name_gap(25)
                .name_text_style(TextStyle::new().font_size(14))
                .axis_label(AxisLabel::new().font_size(12))
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("RPE (m)")
                .name_gap(30)
                .name_text_style(TextStyle::new().font_size(14))
                .axis_label(AxisLabel::new().font_size(12))
        )
        .series(
            Line::new()
                .name("RPE Over Time")
                .smooth(true)
                .line_style(LineStyle::new()
                    .color("rgba(155, 89, 182, 0.85)")
                    .width(3)
                    .type_(LineStyleType::Solid))
                .item_style(ItemStyle::new()
                    .color("rgba(142, 68, 173, 1.0)")
                    .border_color("rgba(142, 68, 173, 1.0)")
                    .border_width(1))
                .symbol(Symbol::Circle)
                .symbol_size(6)
                .data(xy_data)
                .mark_line(
                    MarkLine::new()
                        .line_style(LineStyle::new()
                            .type_(LineStyleType::Dashed)
                            .color("rgba(155, 89, 182, 0.5)")
                            .width(2))
                        .label(Label::new()
                            .show(true)
                            .formatter("Mean")
                            .color("#555")
                            .font_size(12))
                        .symbol(vec![Symbol::None, Symbol::None])
                        .data(vec![
                            MarkLineVariant::Simple(
                                MarkLineData::new()
                                    .name("Mean")
                                    .y_axis(mean)
                            )
                        ])
                )
        );

    chart = add_areas_markers(chart, area_data);

    let filename = "rpe.svg";
    let full_path = format!("{file_path}/{filename}");

    let mut renderer = ImageRenderer::new(1000, 800).theme(Theme::Infographic);
    renderer.save(&chart, full_path);
}



/// PLOTS FOR THE MULTIPLE ITERATIONS

pub fn algorithm_memory_usage_chart(iterations: Vec<Vec<ContainerStats>>) -> Result<Chart, PlotError> {
            
    // Process each iteration to get Memory load percentages
    let mut time_buckets: BTreeMap<i64, Vec<f64>> = BTreeMap::new(); // To put multiple memory usages in the approx the same time;

    for iteration in &iterations {
        let data_ts: Vec<DateTime<Utc>> = iteration.iter().map(|cs| cs.created_at).collect();
        let start_ts = data_ts[0];
        
        let time_sec: Vec<f64> = data_ts.iter()
            .map(|ts| (*ts - start_ts).num_seconds() as f64)
            .collect();
        
        let memory_usage: Vec<f64> = iteration.iter()
            .skip(2)
            .map(|cs| {

                let mu = cs.memory_stats.usage;

                let usage = match mu {
                    Some(u) => u,
                    None => 0
                };

                usage as f64 / 1_000_000.0 //Memory in MB

            })
            .collect();


        for (t, usage) in time_sec.into_iter().zip(memory_usage) {
            let bucket_key = (t * 1000.0) as i64; // ms precision for alignment
            time_buckets
                .entry(bucket_key)
                .or_insert_with(Vec::new)
                .push(usage);
        }
    }

    // Convert to Vec<DataItem> with statistics
    let data_items: Vec<DataItem> = time_buckets
        .into_iter()
        .map(|(key, loads)| {
            let time = ((key as f64) / 1000.0) as i32; // Convert back to seconds
            let mean = loads.iter().sum::<f64>() / loads.len() as f64;
            let variance = loads.iter()
                .map(|&x| (x - mean).powi(2))
                .sum::<f64>() / loads.len() as f64;
            let std_dev = variance.sqrt();

            DataItem {
                time,
                value: mean,
                l: (mean - std_dev).max(0.0), // Don't go below 0%
                u: (mean + std_dev), // Don't exceed 100%
            }
        })
        .collect();

    let base = -data_items
        .iter()
        .fold(f64::INFINITY, |min, val| f64::floor(f64::min(min, val.l)));
    
    let max_y = data_items
        .iter()
        .fold(-f64::INFINITY, |max, val| f64::floor(f64::max(max, val.u)));


    // Create chart
    let chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Category)
                .name("Time (s)")
                .data(data_items.iter().map(|d| d.time.to_string()).collect())
                .name_text_style(TextStyle::new().font_size(16).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(13)),
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Memory Usage (MB)")
                .max(max_y)
                .name_text_style(TextStyle::new().font_size(16).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(13)),
        )
        .series(
            Line::new()
                .name("L")
                .data(data_items.iter().map(|x| x.l + base).collect())
                .line_style(LineStyle::new().opacity(0))
                .stack("confidence-band")
                .symbol(Symbol::None)
        )
        .series(
            Line::new()
                .name("U")
                .data(data_items.iter().map(|x| x.u - x.l).collect())
                .line_style(LineStyle::new().opacity(0))
                .area_style(
                    AreaStyle::new()
                        .color("rgba(102, 187, 106, 0.2)") // Soft green area fill
                )
                .stack("confidence-band")
                .symbol(Symbol::None)
        )
        .series(
            Line::new()
                .name("Mean Memory Usage")
                .data(data_items.iter().map(|d| d.value + base).collect())
                .symbol(Symbol::None)
                .smooth(true)
                .line_style(
                    LineStyle::new()
                        .color("rgba(56, 142, 60, 0.9)")  // Vivid deep green
                        .width(3)
                )
        );

    //let filename = "aggregated_memory_usage.svg";
    //let full_path = format!("{}/{}", file_path, filename);
    //let mut renderer = ImageRenderer::new(1200, 800).theme(Theme::Infographic);
    //renderer.save(&chart, full_path);

    Ok(chart)

}


pub fn algorithm_cpu_load_chart(iterations: Vec<Vec<ContainerStats>>) -> Result<Chart, PlotError> {
    let mut time_buckets: BTreeMap<i64, Vec<f64>> = BTreeMap::new();

    for iteration in &iterations {
        let data_ts: Vec<DateTime<Utc>> = iteration.iter().map(|cs| cs.created_at).collect();
        let start_ts = data_ts[0];

        let time_sec: Vec<f64> = data_ts.iter()
            .map(|ts| (*ts - start_ts).num_seconds() as f64)
            .collect();

        let cpu_load: Result<Vec<f64>, PlotError> = iteration.iter()
            .skip(2)
            .map(|cs| {
    
                let total_usage = cs.cpu_stats.cpu_usage.total_usage;
                let prev_usage = cs.precpu_stats.cpu_usage.total_usage;
                let system_cpu = cs.cpu_stats.system_cpu_usage.ok_or(PlotError::MissingData("CPU usage".to_owned()))?;
                let prev_system_cpu = cs.precpu_stats.system_cpu_usage.ok_or(PlotError::MissingData("Pre CPU usage".to_owned()))?;
                let online_cpus = cs.cpu_stats.online_cpus.ok_or(PlotError::MissingData("Online CPUs".to_owned()))? as f64;
    
                let used = (total_usage - prev_usage) as f64;
                let available = (system_cpu - prev_system_cpu) as f64;
                Ok(used / available * 100.0 * online_cpus)
    
            })
            .collect();

        let cpu_load = cpu_load?;

        for (t, load) in time_sec.into_iter().zip(cpu_load) {
            let bucket_key = (t * 1000.0) as i64;
            time_buckets.entry(bucket_key).or_default().push(load);
        }
    }

    let data_items: Vec<DataItem> = time_buckets
        .into_iter()
        .map(|(key, loads)| {
            let time = ((key as f64) / 1000.0) as i32;
            let mean = loads.iter().sum::<f64>() / loads.len() as f64;
            let variance = loads.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / loads.len() as f64;
            let std_dev = variance.sqrt();
            DataItem {
                time,
                value: mean,
                l: (mean - std_dev).max(0.0),
                u: (mean + std_dev).min(100.0),
            }
        })
        .collect();

    let base = -data_items.iter().map(|d| d.l).fold(f64::INFINITY, f64::min).floor();
    let max_y = data_items.iter().map(|d| d.u).fold(100.0, f64::max).ceil();

    let time_labels: Vec<String> = data_items.iter().map(|d| d.time.to_string()).collect();
    let lower_band: Vec<f64> = data_items.iter().map(|d| d.l + base).collect();
    let std_band: Vec<f64> = data_items.iter().map(|d| (d.u - d.l)).collect();
    let mean_line: Vec<f64> = data_items.iter().map(|d| d.value + base).collect();

    let chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Category)
                .name("Time (s)")
                .data(time_labels)
                .name_text_style(TextStyle::new().font_size(16).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(13)),
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("CPU Load (%)")
                .max(max_y.floor())
                .name_text_style(TextStyle::new().font_size(16).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(13)),
        )
        // Lower bound (invisible, used for stack base)
        .series(
            Line::new()
                .name("Lower Bound")
                .data(lower_band)
                .line_style(LineStyle::new().opacity(0.0))
                .symbol(Symbol::None)
                .stack("std-band")
        )
        // Upper band (deviation area)
        .series(
            Line::new()
                .name("Deviation")
                .data(std_band)
                .line_style(LineStyle::new().opacity(0.0))
                .area_style(
                    AreaStyle::new()
                        .color("rgba(100, 181, 246, 0.3)") // light blue
                )
                .symbol(Symbol::None)
                .stack("std-band")
        )
        // Mean line
        .series(
            Line::new()
                .name("Mean CPU Load")
                .data(mean_line)
                .smooth(true)
                .symbol(Symbol::None) 
                .line_style(LineStyle::new()
                    .width(3)
                    .color("rgba(33, 150, 243, 0.9)") // blue
                )
        );

    //let filename = "aggregated_cpu_load.svg";
    //let full_path = format!("{}/{}", file_path, filename);
    //let mut renderer = ImageRenderer::new(1200, 800)
    //    .theme(Theme::Infographic);
    //renderer.save(&chart, full_path);

    Ok(chart)

}

pub fn algorithm_ape_line_chart(iterations: Vec<Vec<APE>>, test_definition: &TestDefinition) -> Result<Chart, PlotError> {

    // Process each iteration to get Memory load percentages
    let mut time_buckets: BTreeMap<i64, Vec<f64>> = BTreeMap::new(); // To put multiple memory usages in the approx the same time;

    for data in &iterations {

        let time_sec: Vec<f64> = data.iter()
            .map(|ape| {
                ape.time_from_start
            })
            .collect();

        let ape_values: Vec<f64> = data.iter()
            .map(|ape| {
                ape.value
            })
            .collect();


        for (t, usage) in time_sec.into_iter().zip(ape_values) {
            let bucket_key = (t * 1000.0) as i64; // ms precision for alignment
            time_buckets
                .entry(bucket_key)
                .or_insert_with(Vec::new)
                .push(usage);
        }
    }

    let area_data = get_area_from_def(test_definition);

    // Convert to Vec<DataItem> with statistics
    let data_items: Vec<DataItem> = time_buckets
        .into_iter()
        .map(|(key, loads)| {
            let time = ((key as f64) / 1000.0) as i32; // Convert back to seconds
            let mean = loads.iter().sum::<f64>() / loads.len() as f64;
            let variance = loads.iter()
                .map(|&x| (x - mean).powi(2))
                .sum::<f64>() / loads.len() as f64;
            let std_dev = variance.sqrt();

            DataItem {
                time,
                value: mean,
                l: (mean - std_dev).max(0.0), // Don't go below 0%
                u: (mean + std_dev), // Don't exceed 100%
            }
        })
        .collect();

    let base = -data_items
        .iter()
        .fold(f64::INFINITY, |min, val| f64::floor(f64::min(min, val.l)));
    
    let max_y = data_items
        .iter()
        .fold(-f64::INFINITY, |max, val| f64::floor(f64::max(max, val.u)));


    // Create chart
    let mut chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Category)
                .name("Time (s)")
                .data(data_items.iter().map(|d| d.time.to_string()).collect())
                .name_gap(25)
                .name_text_style(TextStyle::new().font_size(14))
                .axis_label(AxisLabel::new().font_size(12))
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("APE (m)")
                .name_gap(30)
                .name_text_style(TextStyle::new().font_size(14))
                .axis_label(AxisLabel::new().font_size(12))
        )
        .series(
            Line::new()
                .name("L")
                .data(data_items.iter().map(|x| x.l + base).collect())
                .line_style(LineStyle::new().opacity(0))
                .stack("confidence-band")
                .symbol(Symbol::None)
        )
        .series(
            Line::new()
                .name("U")
                .data(data_items.iter().map(|x| x.u - x.l).collect())
                .line_style(LineStyle::new().opacity(0))
                .area_style(AreaStyle::new().color("rgba(100, 149, 237, 0.3)"))
                .stack("confidence-band")
                .symbol(Symbol::None)
        )
        .series(
            Line::new()
                .name("APE")
                .data(data_items.iter().map(|d| d.value + base).collect())
                .line_style(LineStyle::new()
                    .color("rgba(52, 152, 219, 0.85)")
                    .width(3)
                    .type_(LineStyleType::Solid))
                .item_style(ItemStyle::new()
                    .color("rgba(41, 128, 185, 1.0)")
                    .border_color("rgba(41, 128, 185, 1.0)")
                    .border_width(1))        
        );

    chart = add_areas_markers(chart, area_data);


    //let filename = "aggregated_ape.svg";
    //let full_path = format!("{}/{}", file_path, filename);
    //let mut renderer = ImageRenderer::new(1200, 800).theme(Theme::Infographic);
    //renderer.save(&chart, full_path);

    Ok(chart)


}

pub fn algorithm_rpe_line_chart(iterations: Vec<Vec<RPE>>, test_definition: &TestDefinition) -> Result<Chart, PlotError> {

    // Process each iteration to get Memory load percentages
    let mut time_buckets: BTreeMap<i64, Vec<f64>> = BTreeMap::new(); // To put multiple memory usages in the approx the same time;

    for data in &iterations {

        let time_sec: Vec<f64> = data.iter()
            .map(|rpe| {
                rpe.time_from_start
            })
            .collect();

        let rpe_values: Vec<f64> = data.iter()
            .map(|rpe| {
                rpe.value
            })
            .collect();


        for (t, usage) in time_sec.into_iter().zip(rpe_values) {
            let bucket_key = (t * 1000.0) as i64; // ms precision for alignment
            time_buckets
                .entry(bucket_key)
                .or_insert_with(Vec::new)
                .push(usage);
        }
    }

    let area_data = get_area_from_def(test_definition);

    // Convert to Vec<DataItem> with statistics
    let data_items: Vec<DataItem> = time_buckets
        .into_iter()
        .map(|(key, loads)| {
            let time = ((key as f64) / 1000.0) as i32; // Convert back to seconds
            let mean = loads.iter().sum::<f64>() / loads.len() as f64;
            let variance = loads.iter()
                .map(|&x| (x - mean).powi(2))
                .sum::<f64>() / loads.len() as f64;
            let std_dev = variance.sqrt();

            DataItem {
                time,
                value: mean,
                l: (mean - std_dev).max(0.0), // Don't go below 0%
                u: (mean + std_dev), // Don't exceed 100%
            }
        })
        .collect();

    let base = -data_items
        .iter()
        .fold(f64::INFINITY, |min, val| f64::floor(f64::min(min, val.l)));
    
    let max_y = data_items
        .iter()
        .fold(-f64::INFINITY, |max, val| f64::floor(f64::max(max, val.u)));


    // Create chart
    let mut chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Category)
                .name("Time (s)")
                .data(data_items.iter().map(|d| d.time.to_string()).collect())
                .name_gap(25)
                .name_text_style(TextStyle::new().font_size(14))
                .axis_label(AxisLabel::new().font_size(12))
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("RPE (m)")
                .name_gap(30)
                .name_text_style(TextStyle::new().font_size(14))
                .axis_label(AxisLabel::new().font_size(12))
        )
        .series(
            Line::new()
                .name("L")
                .data(data_items.iter().map(|x| x.l + base).collect())
                .line_style(LineStyle::new().opacity(0))
                .stack("confidence-band")
                .symbol(Symbol::None)
        )
        .series(
            Line::new()
                .name("U")
                .data(data_items.iter().map(|x| x.u - x.l).collect())
                .line_style(LineStyle::new().opacity(0))
                .area_style(AreaStyle::new().color("rgba(203, 116, 237, 0.35)"))
                .stack("confidence-band")
                .symbol(Symbol::None)
        )
        .series(
            Line::new()
                .name("APE")
                .data(data_items.iter().map(|d| d.value + base).collect())
                .line_style(LineStyle::new()
                    .color("rgba(155, 89, 182, 0.85)")
                    .width(3)
                    .type_(LineStyleType::Solid))
                .item_style(ItemStyle::new()
                    .color("rgba(142, 68, 173, 1.0)")
                    .border_color("rgba(142, 68, 173, 1.0)")
                    .border_width(1))        
        );

    chart = add_areas_markers(chart, area_data);

    Ok(chart)

}









///PLOTS COMPARING THE DIFFERENT METHODS
pub fn test_cpu_load_line_chart(data: &HashMap<Algorithm, Vec<Vec<ContainerStats>>>, file_path: &String) {


    //For each algorithm//Stats in data I need to get a series!!!
    let mut lines_vec: Vec<[Line;3]> = Vec::new();

    for (algo, stats_vec) in data{

        let mut time_buckets: BTreeMap<i64, Vec<f64>> = BTreeMap::new();

        for iteration in stats_vec {
            let data_ts: Vec<DateTime<Utc>> = iteration.iter().map(|cs| cs.created_at).collect();
            let start_ts = data_ts[0];
    
            let time_sec: Vec<f64> = data_ts.iter()
                .map(|ts| (*ts - start_ts).num_seconds() as f64)
                .collect();
    
            let cpu_load: Vec<f64> = iteration.iter()
                .skip(2)
                .map(|cs| {
                    let load_used = (cs.cpu_stats.cpu_usage.total_usage - cs.precpu_stats.cpu_usage.total_usage) as f64;
                    let available = (cs.cpu_stats.system_cpu_usage.unwrap() - cs.precpu_stats.system_cpu_usage.unwrap()) as f64;
                    (load_used / available * 100.0 * cs.cpu_stats.online_cpus.unwrap() as f64).clamp(0.0, 100.0)
                })
                .collect();
    
            for (t, load) in time_sec.into_iter().zip(cpu_load) {
                let bucket_key = (t * 1000.0) as i64;
                time_buckets.entry(bucket_key).or_default().push(load);
            }
        }
    
        let data_items: Vec<DataItem> = time_buckets
            .into_iter()
            .map(|(key, loads)| {
                let time = ((key as f64) / 1000.0) as i32;
                let mean = loads.iter().sum::<f64>() / loads.len() as f64;
                let variance = loads.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / loads.len() as f64;
                let std_dev = variance.sqrt();
                DataItem {
                    time,
                    value: mean,
                    l: (mean - std_dev).max(0.0),
                    u: (mean + std_dev).min(100.0),
                }
            })
            .collect();
    
        let base = -data_items.iter().map(|d| d.l).fold(f64::INFINITY, f64::min).floor();
        let max_y = data_items.iter().map(|d| d.u).fold(100.0, f64::max).ceil();
    
        let time_labels: Vec<f64> = data_items.iter().map(|d| d.time as f64).collect();
        let lower_band: Vec<f64> = data_items.iter().map(|d| d.l + base).collect();
        let std_band: Vec<f64> = data_items.iter().map(|d| (d.u - d.l)).collect();
        let mean_line: Vec<f64> = data_items.iter().map(|d| d.value + base).collect();

        let xy_mean: Vec<Vec<f64>> = time_labels.iter().zip(&mean_line).map(|(x, y)| vec![*x, *y]).collect();
        let xy_up: Vec<Vec<f64>> = time_labels.iter().zip(&std_band).map(|(x, y)| vec![*x, *y]).collect();
        let xy_lower: Vec<Vec<f64>> = time_labels.iter().zip(&lower_band).map(|(x, y)| vec![*x, *y]).collect();

        let main_line = Line::new()
            .name(&algo.name)
            .data(xy_mean)
            .smooth(true)
            .show_symbol(false)
            //.symbol(Symbol::None) 
            .line_style(LineStyle::new()
                .width(3)
                .color(algo.get_rgba(0.9)) // blue
        );

        let lower_band = Line::new()
            .data(xy_lower)
            .line_style(LineStyle::new().opacity(0.0))
            .symbol(Symbol::None)
            .stack(format!("std-band-{}", algo.name));

        let upper_band = Line::new()
            .data(xy_up)
            .line_style(LineStyle::new().opacity(0.0))
            .area_style(
                AreaStyle::new()
                    .color(algo.get_rgba(0.2)) // light blue
            )
            .symbol(Symbol::None)
            .stack(format!("std-band-{}", algo.name));
        
        lines_vec.push([main_line, lower_band, upper_band]);

    }

    let mut chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Category)
                .name("Time (s)")
                .name_text_style(TextStyle::new().font_size(16).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(13)),
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("CPU Load (%)")
                .name_text_style(TextStyle::new().font_size(16).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(13)),
        ).legend(
            Legend::new()
                .show(true)
                .top("top")
                .left("left")
                .orient(Orient::Horizontal)
                .text_style(TextStyle::new().font_size(14))
        );

    chart = add_lines(chart, lines_vec);


    let filename = "test_cpu_load.svg";
    let full_path = format!("{file_path}/{filename}");

    let mut renderer = ImageRenderer::new(1000, 800)
        .theme(Theme::Infographic);
    renderer.save(&chart, full_path);
}




//Helper functions

fn get_area_from_def(test_def: &TestDefinition) -> Vec<MarkArea>{  

    match &test_def.test_type{
        TestType::Simple => vec![],
        TestType::Speed(speed_test_params) => vec![],
        TestType::Drop(drop_params) => {

            let mut mark_areas_vector = Vec::new();

            for drop in drop_params.drop_list.clone(){
                
                let mark_areas_data: Vec<(MarkAreaData, MarkAreaData)> = drop.active_periods.iter()
                    .map(|ap|{
                        (
                            MarkAreaData::new().name(drop.sensor.to_string()).x_axis((ap.start_sec).to_string()),
                            MarkAreaData::new().name(drop.sensor.to_string()).x_axis((ap.start_sec+ap.duration_sec).to_string()),
                        )
                    }).collect();

                    let mark_area = MarkArea::new()
                    .item_style(ItemStyle::new()
                        .color(drop.sensor.get_color())
                        .border_color(drop.sensor.get_color().replace("0.1", "0.3"))
                        .border_width(1))
                        .label(Label::new()
                            .show(true)
                            .position(LabelPosition::Top)
                            .color("rgba(80, 80, 80, 0.8)")
                            .font_size(12)
                            .formatter("{b}")
                        )                        
                    .data(mark_areas_data);
                mark_areas_vector.push(mark_area);

            };
            mark_areas_vector

        },
        TestType::Cut(cut_params) => {
            let mut mark_areas_vector = Vec::new();

            for cut in cut_params.cut_list.clone(){
                
                //let name = format!("{}");

                let mark_areas_data: Vec<(MarkAreaData, MarkAreaData)> = cut.active_periods.iter()
                    .map(|ap|{
                        (
                            MarkAreaData::new().name(cut.sensor.to_string()).x_axis((ap.start_sec).to_string()),
                            MarkAreaData::new().name(cut.sensor.to_string()).x_axis((ap.start_sec+ap.duration_sec).to_string()),
                        )
                    }).collect();

                    let mark_area = MarkArea::new()
                    .item_style(ItemStyle::new()
                        .color(cut.sensor.get_color())
                        .border_color(cut.sensor.get_color().replace("0.1", "0.3"))
                        .border_width(1))
                        .label(Label::new()
                            .show(true)
                            .position(LabelPosition::Top)
                            .color("rgba(80, 80, 80, 1)")
                            .font_size(12)
                            .formatter("{b}")
                        )                        
                    .data(mark_areas_data);
                mark_areas_vector.push(mark_area);

            };
            mark_areas_vector
        },
        }

}

fn add_areas_markers(chart: Chart, area_data: Vec<MarkArea>) -> Chart {

    area_data.into_iter().fold(chart, |acc_chart, area |{
        acc_chart.series(
            Line::new()
            .mark_area(area)
            .line_style(LineStyle::new().opacity(0)) // Hide the line
        )
    })

}

fn add_lines(chart: Chart, line_data: Vec<[Line;3]>) -> Chart{

    line_data.into_iter().fold(chart, |acc_chart, [m,l,u] |{
        acc_chart.series(l).series(u).series(m)
    })

}