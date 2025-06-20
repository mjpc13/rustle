//Util to create plots

use core::f32;
use std::collections::{BTreeMap, HashMap};

use chrono::{DateTime, Utc};
use crate::{models::{metrics::{pose_error::{APE, RPE}, ContainerStats}, AlgorithmRun, TestDefinition, TestType}, services::error::PlotError};

use charming::{
    component::{Axis, Legend}, element::{AreaStyle, AxisLabel, AxisType, ItemStyle, Label, LabelPosition, LineStyle, LineStyleType, MarkArea, MarkAreaData, MarkLine, MarkLineData, MarkLineVariant, Orient, Symbol, TextStyle}, series::Line, Chart
};

use super::config::Config;


#[derive(Debug)]
struct DataItem{
    time: f32,
    value: f32,
    l: f32,
    u: f32
}

//PLOTS FOR A SINGLE ITERATION!!!

pub fn cpu_load_line_chart(data: &Vec<ContainerStats>, config: &Config) -> Result<Chart, PlotError> {
    let data_ts: Vec<DateTime<Utc>> = data.iter().map(|cs| cs.created_at).collect();
    let start_ts = data_ts[0];

    let time_sec: Vec<f32> = data_ts
        .iter()
        .map(|ts| (*ts - start_ts).num_seconds() as f32)
        .collect();

    let cpu_load: Result<Vec<f32>, PlotError> = data.iter()
        .skip(2)
        .map(|cs| {

            let total_usage = cs.cpu_stats.cpu_usage.total_usage;
            let prev_usage = cs.precpu_stats.cpu_usage.total_usage;
            let system_cpu = cs.cpu_stats.system_cpu_usage.ok_or(PlotError::MissingData("CPU usage".to_owned()))?;
            let prev_system_cpu = cs.precpu_stats.system_cpu_usage.ok_or(PlotError::MissingData("Pre CPU usage".to_owned()))?;
            let online_cpus = cs.cpu_stats.online_cpus.ok_or(PlotError::MissingData("Online CPUs".to_owned()))? as f32;

            let used = (total_usage - prev_usage) as f32;
            let available = (system_cpu - prev_system_cpu) as f32;
            Ok(used / available * 100.0 * online_cpus)

        })
        .collect();

    let cpu_load = cpu_load?;

    let mean = if !cpu_load.is_empty() {
        cpu_load.iter().sum::<f32>() / cpu_load.len() as f32
    } else {
        0.0
    };

    let max_load = cpu_load.iter()
        .filter(|&&x| x.is_finite())
        .fold(f32::NAN, |a, &b| if a.is_nan() { b } else { a.max(b) });

    let max_y = if max_load >= 100.0 { (max_load + 0.1*max_load).floor() } else { 100.0 };

    let std_dev = if cpu_load.len() > 1 {
        let variance = cpu_load.iter()
            .map(|&x| (x - mean).powi(2))
            .sum::<f32>() / (cpu_load.len() - 1) as f32;
        variance.sqrt()
    } else {
        0.0
    };

    let xy_data: Vec<Vec<f32>> = time_sec.iter().zip(&cpu_load).map(|(x, y)| vec![*x, *y]).collect();

    let chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Time (s)")
                .name_text_style(TextStyle::new().font_size(config.plotting.x_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.x_axis_label_size)),
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("CPU Load (%)")
                .max(max_y)
                .name_text_style(TextStyle::new().font_size(config.plotting.y_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.y_axis_label_size)),
        )
        .series(
            Line::new()
                .name("CPU Load Over Time")
                .data(xy_data)
                .symbol::<Symbol>(config.plotting.marker_type.into())
                .smooth(config.plotting.smooth)
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

    Ok(chart)

}

pub fn memory_usage_line_chart(data: &Vec<ContainerStats>, config: &Config) -> Result<Chart, PlotError> {
    let data_ts: Vec<DateTime<Utc>> = data.iter().map(|cs| cs.created_at).collect();
    let start_ts = data_ts[0];

    let time_sec: Vec<f32> = data_ts
        .iter()
        .map(|ts| (*ts - start_ts).num_seconds() as f32)
        .collect();

    let memory_usage: Vec<f32> = data.iter()
        .map(|cs| cs.memory_stats.usage.unwrap_or(0) as f32 / 1_000_000.0)
        .collect();

    let mean = if !memory_usage.is_empty() {
        memory_usage.iter().sum::<f32>() / memory_usage.len() as f32
    } else {
        0.0
    };

    let xy_data: Vec<Vec<f32>> = time_sec
        .iter()
        .zip(&memory_usage)
        .map(|(x, y)| vec![*x, *y])
        .collect();

    let chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Time (s)")
                .name_text_style(TextStyle::new().font_size(config.plotting.x_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.x_axis_label_size)),
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Memory Usage (MB)")
                .name_text_style(TextStyle::new().font_size(config.plotting.y_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.y_axis_label_size)),
        )
        .series(
            Line::new()
                .name("Memory Usage Over Time")
                .data(xy_data)
                .symbol::<Symbol>(config.plotting.marker_type.into())
                .smooth(config.plotting.smooth)
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

    Ok(chart)
}

pub fn ape_line_chart(data: &Vec<APE>, test_definition: &TestDefinition, config: &Config) -> Result<Chart, PlotError> {
    let time: Vec<f32> = data.iter().map(|ape| ape.time_from_start).collect();
    let ape_values: Vec<f32> = data.iter().map(|ape| ape.value).collect();

    let mean = if !ape_values.is_empty() {
        ape_values.iter().sum::<f32>() / ape_values.len() as f32
    } else {
        0.0
    };

    let area_data = get_area_from_def(test_definition);

    let xy_data: Vec<Vec<f32>> = time
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
                .name_text_style(TextStyle::new().font_size(config.plotting.x_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.x_axis_label_size))
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("APE (m)")
                .name_gap(30)
                .name_text_style(TextStyle::new().font_size(config.plotting.y_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.y_axis_label_size))
        )
        .series(
            Line::new()
                .name("APE Over Time")
                .smooth(config.plotting.smooth)
                .line_style(LineStyle::new()
                    .color("rgba(52, 152, 219, 0.85)")
                    .width(3)
                    .type_(LineStyleType::Solid))
                .item_style(ItemStyle::new()
                    .color("rgba(41, 128, 185, 1.0)")
                    .border_color("rgba(41, 128, 185, 1.0)")
                    .border_width(1))
                .symbol::<Symbol>(config.plotting.marker_type.into())
                .symbol_size(config.plotting.marker_size)
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

    chart = add_areas_markers(chart, area_data, config);

    Ok(chart)

}

pub fn rpe_line_chart(data: &Vec<RPE>, test_definition: &TestDefinition, config: &Config) -> Result<Chart, PlotError> {
    let time: Vec<f32> = data.iter().map(|rpe| rpe.time_from_start).collect();
    let rpe_values: Vec<f32> = data.iter().map(|rpe| rpe.value).collect();

    let mean = if !rpe_values.is_empty() {
        rpe_values.iter().sum::<f32>() / rpe_values.len() as f32
    } else {
        0.0
    };

    let area_data = get_area_from_def(&test_definition);

    let xy_data: Vec<Vec<f32>> = time
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
                .name_text_style(TextStyle::new().font_size(config.plotting.x_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.x_axis_label_size))
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("RPE (m)")
                .name_gap(30)
                .name_text_style(TextStyle::new().font_size(config.plotting.y_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.y_axis_label_size))
        )
        .series(
            Line::new()
                .name("RPE Over Time")
                .smooth(config.plotting.smooth)
                .line_style(LineStyle::new()
                    .color("rgba(155, 89, 182, 0.85)")
                    .width(3)
                    .type_(LineStyleType::Solid))
                .item_style(ItemStyle::new()
                    .color("rgba(142, 68, 173, 1.0)")
                    .border_color("rgba(142, 68, 173, 1.0)")
                    .border_width(1))
                .symbol::<Symbol>(config.plotting.marker_type.into())
                .symbol_size(config.plotting.marker_size)
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

    chart = add_areas_markers(chart, area_data, config);

    Ok(chart)
}


/// PLOTS FOR THE MULTIPLE ITERATIONS
pub fn algorithm_memory_usage_chart(iterations: Vec<Vec<ContainerStats>>, config: &Config) -> Result<Chart, PlotError> {

    // Process each iteration to get Memory load percentages
    let mut time_buckets: BTreeMap<i64, Vec<f32>> = BTreeMap::new(); // To put multiple memory usages in the approx the same time;

    for iteration in &iterations {
        let data_ts: Vec<DateTime<Utc>> = iteration.iter().map(|cs| cs.created_at).collect();
        let start_ts = data_ts[0];
        
        let time_sec: Vec<f32> = data_ts.iter()
            .map(|ts| (*ts - start_ts).num_seconds() as f32)
            .collect();
        
        let memory_usage: Vec<f32> = iteration.iter()
            .skip(2)
            .map(|cs| {

                let mu = cs.memory_stats.usage;

                let usage = match mu {
                    Some(u) => u,
                    None => 0
                };

                usage as f32 / 1_000_000.0 //Memory in MB

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
            let time = (key as f32) / 1000.0; // Convert back to seconds
            let mean = loads.iter().sum::<f32>() / loads.len() as f32;

            let variance = loads.iter()
                .map(|&x| (x - mean).powi(2))
                .sum::<f32>() / loads.len() as f32;
            let std_dev = variance.sqrt();

            DataItem {
                time,
                value: mean,
                l: (mean - std_dev).max(0.0), // Don't go below 0%
                u: (mean + std_dev), // Don't exceed 100%
            }
        })
        .collect();
    
    let max_y = data_items
        .iter()
        .fold(-f32::INFINITY, |max, val| f32::floor(f32::max(max, val.u)));

    // Create confidence band and mean line points
    let xy_mean = data_items.iter().map(|d| vec![d.time, d.value]).collect::<Vec<_>>();
    let upper = data_items.iter().map(|d| vec![d.time, d.u]).collect::<Vec<_>>();
    let lower = data_items.iter().rev().map(|d| vec![d.time, d.l]).collect::<Vec<_>>();

    let confidence_band = upper
        .into_iter()
        .chain(lower)
        .collect::<Vec<_>>();

    // Create chart
    let chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Time (s)")
                .name_text_style(TextStyle::new().font_size(config.plotting.x_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.x_axis_label_size)),
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Memory Usage (MB)")
                .max(max_y+0.05*max_y)
                .name_text_style(TextStyle::new().font_size(config.plotting.y_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.y_axis_label_size)),
        )
        .series(
            Line::new()
                .name("Confidence Band")
                .data(confidence_band)
                .line_style(LineStyle::new().opacity(0))
                .area_style(AreaStyle::new().color("rgba(56, 142, 60, 0.2)"))
                .symbol(Symbol::None)
        )
        .series(
            Line::new()
                .name("Mean Memory Usage")
                .data(xy_mean)
                .symbol::<Symbol>(config.plotting.marker_type.into())
                .symbol_size(config.plotting.marker_size)
                .smooth(config.plotting.smooth)
                .line_style(
                    LineStyle::new()
                        .color("rgba(56, 142, 60, 0.9)")  // Vivid deep green
                        .width(3)
                )
        );

    Ok(chart)
}

pub fn algorithm_cpu_load_chart(iterations: Vec<Vec<ContainerStats>>, config: &Config) -> Result<Chart, PlotError> {
    let mut time_buckets: BTreeMap<i64, Vec<f32>> = BTreeMap::new();

    for iteration in &iterations {
        let data_ts: Vec<DateTime<Utc>> = iteration.iter().map(|cs| cs.created_at).collect();
        let start_ts = data_ts[0];

        let time_sec: Vec<f32> = data_ts.iter()
            .map(|ts| (*ts - start_ts).num_seconds() as f32)
            .collect();

        let cpu_load: Result<Vec<f32>, PlotError> = iteration.iter()
            .skip(2)
            .map(|cs| {
    
                let total_usage = cs.cpu_stats.cpu_usage.total_usage;
                let prev_usage = cs.precpu_stats.cpu_usage.total_usage;
                let system_cpu = cs.cpu_stats.system_cpu_usage.ok_or(PlotError::MissingData("CPU usage".to_owned()))?;
                let prev_system_cpu = cs.precpu_stats.system_cpu_usage.ok_or(PlotError::MissingData("Pre CPU usage".to_owned()))?;
                let online_cpus = cs.cpu_stats.online_cpus.ok_or(PlotError::MissingData("Online CPUs".to_owned()))? as f32;
    
                let used = (total_usage - prev_usage) as f32;
                let available = (system_cpu - prev_system_cpu) as f32;
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
            let time = (key as f32) / 1000.0;
            let mean = loads.iter().sum::<f32>() / loads.len() as f32;
            let variance = loads.iter().map(|&x| (x - mean).powi(2)).sum::<f32>() / loads.len() as f32;
            let std_dev = variance.sqrt();

            DataItem {
                time,
                value: mean,
                l: (mean - std_dev).max(0.0),
                u: (mean + std_dev),
            }
        })
        .collect();

    let max_y = data_items.iter().map(|d| d.u + 0.1 * d.u).fold(100.0, f32::max).ceil();

    // Create confidence band and mean line points
    let xy_mean = data_items.iter().map(|d| vec![d.time, d.value]).collect::<Vec<_>>();
    let upper = data_items.iter().map(|d| vec![d.time, d.u]).collect::<Vec<_>>();
    let lower = data_items.iter().rev().map(|d| vec![d.time, d.l]).collect::<Vec<_>>();

    let confidence_band = upper
        .into_iter()
        .chain(lower)
        .collect::<Vec<_>>();


    let chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Time (s)")
                .name_text_style(TextStyle::new().font_size(config.plotting.x_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.x_axis_label_size)),
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("CPU Load (%)")
                .max(max_y.floor())
                .name_text_style(TextStyle::new().font_size(config.plotting.y_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.y_axis_label_size)),
        )
        // Mean line
        .series(
            Line::new()
                .name("Mean CPU Load")
                .data(xy_mean)
                .symbol::<Symbol>(config.plotting.marker_type.into())
                .symbol_size(config.plotting.marker_size)
                .smooth(config.plotting.smooth)
                .line_style(LineStyle::new()
                    .width(3)
                    .color("rgba(33, 150, 243, 0.9)") // blue
                )
        )
        .series(
            Line::new()
                .name("Confidence Band")
                .data(confidence_band)
                .line_style(LineStyle::new().opacity(0))
                .area_style(AreaStyle::new().color("rgba(100, 181, 246, 0.3)"))
                .symbol(Symbol::None)
        );

    Ok(chart)

}

pub fn algorithm_ape_line_chart(iterations: Vec<Vec<APE>>, test_definition: &TestDefinition, config: &Config) -> Result<Chart, PlotError> {

    // Process each iteration to get Memory load percentages
    let mut time_buckets: BTreeMap<i64, Vec<f32>> = BTreeMap::new(); // To put multiple memory usages in the approx the same time;

    for data in &iterations {

        let time_sec: Vec<f32> = data.iter()
            .map(|ape| {
                ape.time_from_start
            })
            .collect();

        let ape_values: Vec<f32> = data.iter()
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
            
            let time = (key as f32) / 1000.0; // Convert back to seconds
            let mean = loads.iter().sum::<f32>() / loads.len() as f32;
            let variance = loads.iter()
                .map(|&x| (x - mean).powi(2))
                .sum::<f32>() / loads.len() as f32;
            let std_dev = variance.sqrt();

            DataItem {
                time,
                value: mean,
                l: (mean - std_dev).max(0.0), // Don't go below 0
                u: (mean + std_dev), // Don't exceed 100%
            }
        })
        .collect();
    
    // Create confidence band and mean line points
    let xy_mean = data_items.iter().map(|d| vec![d.time, d.value]).collect::<Vec<_>>();
    let upper = data_items.iter().map(|d| vec![d.time, d.u]).collect::<Vec<_>>();
    let lower = data_items.iter().rev().map(|d| vec![d.time, d.l]).collect::<Vec<_>>();

    let confidence_band = upper
        .into_iter()
        .chain(lower)
        .collect::<Vec<_>>();

    // Create chart
    let mut chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Time (s)")
                .name_gap(25)
                .name_text_style(TextStyle::new().font_size(config.plotting.x_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.x_axis_label_size))
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("APE (m)")
                .name_gap(30)
                .name_text_style(TextStyle::new().font_size(config.plotting.x_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.y_axis_label_size))
        )
        .series(
            Line::new()
                .name("APE")
                .data(xy_mean)
                .line_style(LineStyle::new()
                    .color("rgba(52, 152, 219, 0.85)")
                    .width(3)
                    .type_(LineStyleType::Solid))
                .item_style(ItemStyle::new()
                    .color("rgba(41, 128, 185, 1.0)")
                    .border_color("rgba(41, 128, 185, 1.0)")
                    .border_width(1))
                .symbol::<Symbol>(config.plotting.marker_type.into())
                .symbol_size(config.plotting.marker_size)
        )
        .series(
            Line::new()
                .name("Confidence Band")
                .data(confidence_band)
                .line_style(LineStyle::new().opacity(0))
                .area_style(AreaStyle::new().color("rgba(100, 149, 237, 0.3)"))
                .symbol(Symbol::None)
        );

    chart = add_areas_markers(chart, area_data, config);


    Ok(chart)
}

pub fn algorithm_rpe_line_chart(iterations: Vec<Vec<RPE>>, test_definition: &TestDefinition, config: &Config) -> Result<Chart, PlotError> {

    // Process each iteration to get Memory load percentages
    let mut time_buckets: BTreeMap<i64, Vec<f32>> = BTreeMap::new(); // To put multiple memory usages in the approx the same time;

    for data in &iterations {

        let time_sec: Vec<f32> = data.iter()
            .map(|rpe| {
                rpe.time_from_start
            })
            .collect();

        let rpe_values: Vec<f32> = data.iter()
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
            let time = (key as f32) / 1000.0; // Convert back to seconds
            let mean = loads.iter().sum::<f32>() / loads.len() as f32;
            let variance = loads.iter()
                .map(|&x| (x - mean).powi(2))
                .sum::<f32>() / loads.len() as f32;
            let std_dev = variance.sqrt();

            DataItem {
                time,
                value: mean,
                l: (mean - std_dev).max(0.0), // Don't go below 0%
                u: (mean + std_dev), // Don't exceed 100%
            }
        })
        .collect();
    
    // Create confidence band and mean line points
    let xy_mean = data_items.iter().map(|d| vec![d.time, d.value]).collect::<Vec<_>>();
    let upper = data_items.iter().map(|d| vec![d.time, d.u]).collect::<Vec<_>>();
    let lower = data_items.iter().rev().map(|d| vec![d.time, d.l]).collect::<Vec<_>>();

    let confidence_band = upper
        .into_iter()
        .chain(lower)
        .collect::<Vec<_>>();

    // Create chart
    let mut chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Time (s)")
                .name_gap(25)
                .name_text_style(TextStyle::new().font_size(config.plotting.x_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.x_axis_label_size))
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("RPE (m)")
                .name_gap(30)
                .name_text_style(TextStyle::new().font_size(config.plotting.y_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.y_axis_label_size))
        )
        .series(
            Line::new()
                .name("APE")
                .data(xy_mean)
                .line_style(LineStyle::new()
                    .color("rgba(155, 89, 182, 0.85)")
                    .width(3)
                    .type_(LineStyleType::Solid))
                .item_style(ItemStyle::new()
                    .color("rgba(142, 68, 173, 1.0)")
                    .border_color("rgba(142, 68, 173, 1.0)")
                    .border_width(1))
                .symbol::<Symbol>(config.plotting.marker_type.into())
                .symbol_size(config.plotting.marker_size)
        ).series(
            Line::new()
                .name("Confidence Band")
                .data(confidence_band)
                .line_style(LineStyle::new().opacity(0))
                .area_style(AreaStyle::new().color("rgba(203, 116, 237, 0.35)"))
                .symbol(Symbol::None)
        );

    chart = add_areas_markers(chart, area_data, config);

    Ok(chart)

}




//PLOTS COMPARING THE DIFFERENT METHODS
pub fn test_cpu_load_line_chart(data: &HashMap<AlgorithmRun, Vec<Vec<ContainerStats>>>, config: &Config)  -> Result<Chart, PlotError> {

    //For each algorithm//Stats in data I need to get a series!!!
    let mut lines_vec: Vec<[Line;2]> = Vec::new();

    let mut max_y_list: Vec<f32> = Vec::new();

    for (algo_run, stats_vec) in data{

        let mut time_buckets: BTreeMap<i64, Vec<f32>> = BTreeMap::new();

        for iteration in stats_vec {
            let data_ts: Vec<DateTime<Utc>> = iteration.iter().map(|cs| cs.created_at).collect();
            let start_ts = data_ts[0];
    
            let time_sec: Vec<f32> = data_ts.iter()
                .map(|ts| (*ts - start_ts).num_seconds() as f32)
                .collect();
    
            let cpu_load: Vec<f32> = iteration.iter()
                .skip(2)
                .map(|cs| {
                    let load_used = (cs.cpu_stats.cpu_usage.total_usage - cs.precpu_stats.cpu_usage.total_usage) as f32;
                    let available = (cs.cpu_stats.system_cpu_usage.unwrap() - cs.precpu_stats.system_cpu_usage.unwrap()) as f32;
                    (load_used / available * 100.0 * cs.cpu_stats.online_cpus.unwrap() as f32).clamp(0.0, f32::MAX)
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
                let time = (key as f32) / 1000.0;
                let mean = loads.iter().sum::<f32>() / loads.len() as f32;
                let variance = loads.iter().map(|&x| (x - mean).powi(2)).sum::<f32>() / loads.len() as f32;
                let std_dev = variance.sqrt();

                DataItem {
                    time,
                    value: mean,
                    l: (mean - std_dev).max(0.0),
                    u: (mean + std_dev),
                }
            })
            .collect();
    
        let max_y = data_items.iter().map(|d| d.value).fold(100.0, f32::max).ceil();


        max_y_list.push(max_y);
    
        // Create confidence band and mean line points
        let xy_mean = data_items.iter().map(|d| vec![d.time, d.value]).collect::<Vec<_>>();
        let upper = data_items.iter().map(|d| vec![d.time, d.u]).collect::<Vec<_>>();
        let lower = data_items.iter().rev().map(|d| vec![d.time, d.l]).collect::<Vec<_>>();

        let confidence_band = upper
            .into_iter()
            .chain(lower)
            .collect::<Vec<_>>();

        let main_line = Line::new()
            .name(format!("{}_{}x",&algo_run.algo.name, &algo_run.bag_speed))
            .data(xy_mean)
            .smooth(config.plotting.smooth)
            .symbol::<Symbol>(config.plotting.marker_type.into())
            .symbol_size(config.plotting.marker_size)
            .line_style(LineStyle::new()
                .width(3)
                .color(algo_run.get_distinct_rgba(0.9))
        );

        let band_line = Line::new()
                .name("")
                .data(confidence_band)
                .line_style(LineStyle::new().opacity(0))
                .area_style(AreaStyle::new().color(algo_run.get_distinct_rgba(0.2)))
                .symbol(Symbol::None);
        
        lines_vec.push([main_line, band_line]);

    }


    let max_y = max_y_list
        .iter()
        .copied()
        .fold(f32::NEG_INFINITY, f32::max)
        .max(100.0);

    let mut chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Time (s)")
                .name_text_style(TextStyle::new().font_size(config.plotting.x_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.x_axis_label_size)),
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("CPU Load (%)")
                .max((max_y + max_y*0.1).floor())
                .name_text_style(TextStyle::new().font_size(config.plotting.y_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.y_axis_label_size)),
        );

    if config.plotting.show_legend{
        chart = chart.legend(
            Legend::new()
                .show(true)
                .top("top")
                .left("left")
                .orient(Orient::Horizontal)
                .text_style(TextStyle::new().font_size(14))
        )
    }

    chart = add_lines(chart, lines_vec, config);


    Ok(chart)

}

pub fn test_memory_usage_line_chart(data: &HashMap<AlgorithmRun, Vec<Vec<ContainerStats>>>, config: &Config)  -> Result<Chart, PlotError> {


    //For each algorithm//Stats in data I need to get a series!!!
    let mut lines_vec: Vec<[Line;2]> = Vec::new();

    for (algo_run, stats_vec) in data{

        let mut time_buckets: BTreeMap<i64, Vec<f32>> = BTreeMap::new();

        for iteration in stats_vec {
            let data_ts: Vec<DateTime<Utc>> = iteration.iter().map(|cs| cs.created_at).collect();
            let start_ts = data_ts[0];
    
            let time_sec: Vec<f32> = data_ts.iter()
                .map(|ts| (*ts - start_ts).num_seconds() as f32)
                .collect();
    
            let memory: Vec<f32> = iteration.iter()
                .skip(2)
                .map(|cs| {
                    let mem = match cs.memory_stats.usage{
                        Some(p) => p  as f32 / 1_000_000.0,
                        None => -1.0
                    };
                    mem
                })
                .collect();
    
            for (t, mem) in time_sec.into_iter().zip(memory) {
                let bucket_key = (t * 1000.0) as i64;
                time_buckets.entry(bucket_key).or_default().push(mem);
            }
        }
    
        let data_items: Vec<DataItem> = time_buckets
            .into_iter()
            .map(|(key, mem)| {
                let time = (key as f32) / 1000.0;
                let mean = mem.iter().sum::<f32>() / mem.len() as f32;
                let variance = mem.iter().map(|&x| (x - mean).powi(2)).sum::<f32>() / mem.len() as f32;
                let std_dev = variance.sqrt();
                DataItem {
                    time,
                    value: mean,
                    l: (mean - std_dev).max(0.0),
                    u: (mean + std_dev),
                }
            })
            .collect();
        
        // Create confidence band and mean line points
        let xy_mean = data_items.iter().map(|d| vec![d.time, d.value]).collect::<Vec<_>>();
        let upper = data_items.iter().map(|d| vec![d.time, d.u]).collect::<Vec<_>>();
        let lower = data_items.iter().rev().map(|d| vec![d.time, d.l]).collect::<Vec<_>>();

        let confidence_band = upper
            .into_iter()
            .chain(lower)
            .collect::<Vec<_>>();

        let main_line = Line::new()
            .name(format!("{}_{}x",&algo_run.algo.name, &algo_run.bag_speed))
            .data(xy_mean)
            .smooth(config.plotting.smooth)
            .symbol::<Symbol>(config.plotting.marker_type.into())
            .symbol_size(config.plotting.marker_size)
            .line_style(LineStyle::new()
                .width(3)
                .color(algo_run.get_distinct_rgba(0.9))
        );

        let band_line = Line::new()
                .name("")
                .data(confidence_band)
                .line_style(LineStyle::new().opacity(0))
                .area_style(AreaStyle::new().color(algo_run.get_distinct_rgba(0.2)))
                .symbol(Symbol::None);
        
        lines_vec.push([main_line, band_line]);

    }

    let mut chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Time (s)")
                .name_text_style(TextStyle::new().font_size(config.plotting.x_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.x_axis_label_size)),
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Memory Usage (Mb)")
                .name_text_style(TextStyle::new().font_size(config.plotting.y_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.y_axis_label_size)),
        );

        if config.plotting.show_legend{
            chart = chart.legend(
                Legend::new()
                    .show(true)
                    .top("top")
                    .left("left")
                    .orient(Orient::Horizontal)
                    .text_style(TextStyle::new().font_size(14))
            )
        }
    

    chart = add_lines(chart, lines_vec, config);


    Ok(chart)
}

pub fn test_ape_line_chart(data: &HashMap<AlgorithmRun, Vec<Vec<APE>>>, config: &Config)  -> Result<Chart, PlotError> {

    //For each algorithm//Stats in data I need to get a series!!!
    let mut lines_vec: Vec<[Line;2]> = Vec::new();

    for (algo_run, iterations) in data{

        let mut time_buckets: BTreeMap<i64, Vec<f32>> = BTreeMap::new();

        for iteration in iterations {
            let time_sec: Vec<f32> = iteration.iter().map(|ape| ape.time_from_start).collect();
    
            let ape_values: Vec<f32> = iteration.iter()
                .skip(1)
                .map(|a| {
                    a.value
                })
                .collect();
    
            for (t, ape) in time_sec.into_iter().zip(ape_values) {
                let bucket_key = (t * 1000.0) as i64;
                time_buckets.entry(bucket_key).or_default().push(ape);
            }
        }
    
        let data_items: Vec<DataItem> = time_buckets
            .into_iter()
            .map(|(key, ape)| {
                let time = (key as f32) / 1000.0;
                let mean = ape.iter().sum::<f32>() / ape.len() as f32;
                let variance = ape.iter().map(|&x| (x - mean).powi(2)).sum::<f32>() / ape.len() as f32;
                let std_dev = variance.sqrt();
                DataItem {
                    time,
                    value: mean,
                    l: (mean - std_dev).max(0.0),
                    u: (mean + std_dev),
                }
            })
            .collect();
    
   
        // Create confidence band and mean line points
        let xy_mean = data_items.iter().map(|d| vec![d.time, d.value]).collect::<Vec<_>>();
        let upper = data_items.iter().map(|d| vec![d.time, d.u]).collect::<Vec<_>>();
        let lower = data_items.iter().rev().map(|d| vec![d.time, d.l]).collect::<Vec<_>>();

        let confidence_band = upper
            .into_iter()
            .chain(lower)
            .collect::<Vec<_>>();

        let main_line = Line::new()
            .name(format!("{}_{}x",&algo_run.algo.name, &algo_run.bag_speed))
            .data(xy_mean)
            .smooth(config.plotting.smooth)
            .symbol::<Symbol>(config.plotting.marker_type.into())
            .symbol_size(config.plotting.marker_size)
            .line_style(LineStyle::new()
                .width(3)
                .color(algo_run.get_distinct_rgba(0.9))
        );

        let band_line = Line::new()
                .name("")
                .data(confidence_band)
                .line_style(LineStyle::new().opacity(0))
                .area_style(AreaStyle::new().color(algo_run.get_distinct_rgba(0.2)))
                .symbol(Symbol::None);
        
        lines_vec.push([main_line, band_line]);

    }

    let mut chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Time (s)")
                .name_text_style(TextStyle::new().font_size(config.plotting.x_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.x_axis_label_size)),
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("APE (m)")
                .name_text_style(TextStyle::new().font_size(config.plotting.y_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.y_axis_label_size)),
        );


    if config.plotting.show_legend{
        chart = chart.legend(
            Legend::new()
                .show(true)
                .top("top")
                .left("left")
                .orient(Orient::Horizontal)
                .text_style(TextStyle::new().font_size(14))
        )
    }
    

    chart = add_lines(chart, lines_vec, config);


    Ok(chart)

}


pub fn test_rpe_line_chart(data: &HashMap<AlgorithmRun, Vec<Vec<RPE>>>, config: &Config)  -> Result<Chart, PlotError> {

    //For each algorithm//Stats in data I need to get a series!!!
    let mut lines_vec: Vec<[Line;2]> = Vec::new();

    for (algo_run, iterations) in data{

        let mut time_buckets: BTreeMap<i64, Vec<f32>> = BTreeMap::new();

        for iteration in iterations {
            let time_sec: Vec<f32> = iteration.iter().map(|rpe| rpe.time_from_start).collect();
    
            let rpe_values: Vec<f32> = iteration.iter()
                .skip(1)
                .map(|a| {
                    a.value
                })
                .collect();
    
            for (t, rpe) in time_sec.into_iter().zip(rpe_values) {
                let bucket_key = (t * 1000.0) as i64;
                time_buckets.entry(bucket_key).or_default().push(rpe);
            }
        }
    
        let data_items: Vec<DataItem> = time_buckets
            .into_iter()
            .map(|(key, rpe)| {
                let time = (key as f32) / 1000.0;
                let mean = rpe.iter().sum::<f32>() / rpe.len() as f32;
                let variance = rpe.iter().map(|&x| (x - mean).powi(2)).sum::<f32>() / rpe.len() as f32;
                let std_dev = variance.sqrt();

                DataItem {
                    time,
                    value: mean,
                    l: (mean - std_dev).max(0.0),
                    u: (mean + std_dev),
                }
            })
            .collect();
    
        // Create confidence band and mean line points
        let xy_mean = data_items.iter().map(|d| vec![d.time, d.value]).collect::<Vec<_>>();
        let upper = data_items.iter().map(|d| vec![d.time, d.u]).collect::<Vec<_>>();
        let lower = data_items.iter().rev().map(|d| vec![d.time, d.l]).collect::<Vec<_>>();

        let confidence_band = upper
            .into_iter()
            .chain(lower)
            .collect::<Vec<_>>();

        let main_line = Line::new()
            .name(format!("{}_{}x",&algo_run.algo.name, &algo_run.bag_speed))
            .data(xy_mean)
            .symbol::<Symbol>(config.plotting.marker_type.into())
            .symbol_size(config.plotting.marker_size)
            .line_style(LineStyle::new()
                .width(3)
                .color(algo_run.get_distinct_rgba(0.9))
        );

        let band_line = Line::new()
                .name("")
                .data(confidence_band)
                .line_style(LineStyle::new().opacity(0))
                .area_style(AreaStyle::new().color(algo_run.get_distinct_rgba(0.2)))
                .symbol(Symbol::None);
        
        lines_vec.push([main_line, band_line]);

    }

    let mut chart = Chart::new()
        .x_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("Time (s)")
                .name_text_style(TextStyle::new().font_size(config.plotting.x_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.x_axis_label_size)),
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name("RPE (m)")
                .name_text_style(TextStyle::new().font_size(config.plotting.y_axis_title_size).font_weight("bold"))
                .axis_label(AxisLabel::new().font_size(config.plotting.y_axis_label_size)),
        );

    if config.plotting.show_legend{
        chart = chart.legend(
            Legend::new()
                .show(true)
                .top("top")
                .left("left")
                .orient(Orient::Horizontal)
                .text_style(TextStyle::new().font_size(14))
        )
    }
    
    chart = add_lines(chart, lines_vec, config);


    Ok(chart)

}



//Helper functions

fn get_area_from_def(test_def: &TestDefinition) -> Vec<MarkArea>{  

    match &test_def.test_type{
        TestType::Simple => vec![],
        TestType::Speed(_speed_test_params) => vec![],
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

fn add_areas_markers(chart: Chart, area_data: Vec<MarkArea>, config: &Config) -> Chart {

    if config.plotting.show_band{
        area_data.into_iter().fold(chart, |acc_chart, area |{
            acc_chart.series(
                Line::new()
                .mark_area(area)
                .line_style(LineStyle::new().opacity(0)) // Hide the line
            )
        })
    } else {
        chart
    }

}

fn add_lines(chart: Chart, line_data: Vec<[Line;2]>, config: &Config) -> Chart{

    if config.plotting.show_confidence_band{
        line_data.into_iter().fold(chart, |acc_chart, [m,b] |{
            acc_chart.series(m).series(b)
        })
    } else{
        line_data.into_iter().fold(chart, |acc_chart, [m, _] |{
            acc_chart.series(m)
        })
    }

}