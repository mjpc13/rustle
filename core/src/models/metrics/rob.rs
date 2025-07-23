use std::collections::HashMap;

use log::warn;
use serde::{Deserialize, Serialize};

use crate::{models::{metric::StatisticalMetricsStamped, test_definitions::test_definition::{RobustnessType, Sensor}, AlgorithmRun, TestType}, services::error::MetricError};

use super::metric::StatisticalMetrics;



#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobustnessMetric {
    pub sev: f32,                       // Severety of the cut//drop.
    pub adp: DegradationMetric,                      // Absolute Degradation;
    pub rdp: DegradationMetric,                      // Relative Degradation;
    pub art: RecoveryTime,                      // Absolute Recovery Time; Only has values if the algo has loop closure
    pub rrt: RecoveryTime,                      // Relative Recovery Time;
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DegradationMetric{
    pub metric: StatisticalMetrics,
    pub values_list: Vec<(f32,f32)>, //(timestamp, degradation vs the baseline)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecoveryTime{
    pub hash_list_period: HashMap<String, Vec<Option<f32>>>,
    pub hash_list_sensor: HashMap<String, Option<StatisticalMetrics>>,
    pub rt: Option<StatisticalMetrics>
}



impl RobustnessMetric{

    pub fn new(rob_type: RobustnessType, duration: f32, test: &AlgorithmRun, baseline: &AlgorithmRun, threshold: f32) -> Result<Self, MetricError>{

        let sev = match rob_type {
            RobustnessType::Cut(cut_vec) => {

                let mut durations: Vec<f32>= vec![];

                cut_vec.iter().for_each(|cut|{
                    cut.active_periods.iter().for_each(|ap|{
                        durations.push(ap.duration_sec as f32);
                    });
                });
                
                let cut_duration: f32 = durations.iter().sum::<f32>();
                cut_duration / duration
                
            },
            RobustnessType::Drop(drop_vec) => {

                let mut durations: Vec<f32>= vec![];
    
                drop_vec.iter().for_each(|drop|{
                    let d = drop.drop_rate[0] as f32/ drop.drop_rate[1] as f32;

                    drop.active_periods.iter().for_each(|ap|{
                        durations.push(ap.duration_sec as f32 * d);
                    });
                });
                
                let drop_duration: f32 = durations.iter().sum::<f32>();
                drop_duration / duration
            },
        };


        let matched_ape = match_by_timestamp(&test.ape_list, &baseline.ape_list, threshold);
        let matched_rpe = match_by_timestamp(&test.rpe_list, &baseline.rpe_list, threshold);

        let adp = compute_dm(&matched_ape, &sev)?;
        let rdp = compute_dm(&matched_rpe, &sev)?;

        let art = compute_rt(matched_ape, &test.test_type)?;
        let rrt = compute_rt(matched_rpe, &test.test_type)?;

        Ok(RobustnessMetric{
            sev,
            adp,
            rdp,
            art,
            rrt,
        })
    }

}



fn compute_dm(data: &Vec<(&StatisticalMetricsStamped, &StatisticalMetricsStamped)>, sev: &f32) -> Result<DegradationMetric, MetricError>{
    
    let dm_list: Vec<(f32,f32)> = data
        .iter()
        .map(|(test, baseline)|{
            (baseline.timestamp, ((test.stat.mean - baseline.stat.mean).abs()) / sev)
        }).collect();

    let seconds: Vec<f32> = dm_list.iter().map(|&(_, second)| second).collect();


    let metric = StatisticalMetrics::from_values(&seconds, true).ok_or(MetricError::MissingError("Missing values to compute robustness".to_owned()))?;

    Ok(DegradationMetric{
        metric: metric,
        values_list: dm_list,
    })

}

fn match_by_timestamp<'a>(
    test_list: &'a [StatisticalMetricsStamped],
    baseline_list: &'a [StatisticalMetricsStamped],
    threshold: f32
) -> Vec<(&'a StatisticalMetricsStamped, &'a StatisticalMetricsStamped)> {

    let mut matched = Vec::new();

    let mut baseline_iter = baseline_list.iter().peekable();
    let mut last_baseline = None;

    for test_st in test_list {
        // Advance baseline_iter to find potential closest
        while let Some(baseline_st) = baseline_iter.peek() {
            if baseline_st.timestamp < test_st.timestamp {
                last_baseline = baseline_iter.next();
            } else {
                break;
            }
        }

        // Determine closest match
        let closest_baseline = match (last_baseline, baseline_iter.peek()) {
            (Some(prev), Some(next)) => {
                if (next.timestamp - test_st.timestamp).abs() < (test_st.timestamp - prev.timestamp).abs() {
                    Some(*next)
                } else {
                    Some(prev)
                }
            },
            (Some(prev), None) => Some(prev),
            (None, Some(next)) => Some(*next),
            (None, None) => None,
        };

        // Check threshold
        if let Some(baseline_st) = closest_baseline {
            if (baseline_st.timestamp - test_st.timestamp).abs() <= threshold/2.0 {
                matched.push((test_st, baseline_st));
            }
        }
    }

    matched
}


fn compute_rt(data: Vec<(&StatisticalMetricsStamped, &StatisticalMetricsStamped)>, test_type: &TestType) -> Result<RecoveryTime, MetricError>{

    let mut rt_hash: HashMap<String, Vec<Option<f32>>> = HashMap::new();
    let mut rt_hash_sensor: HashMap<String, Option<StatisticalMetrics>> = HashMap::new();
    let mut all_rt_values: Vec<f32> = vec![];

    match test_type {
        TestType::Drop(drop_params) => {
                //Compute stuff for each drop.active_periods
                drop_params.drop_list
                    .iter()
                    .for_each(|d|{
                        let mut failure_time_list: Vec<f32> = vec![];

                        //Push the timestamps of end of failures to then search from there.
                        d.active_periods
                            .iter()
                            .for_each(|ap|{                                
                                match &ap.repeat {
                                    Some(r) => {
                                        let mut failure_time = (ap.start_sec + ap.duration_sec) as f32;
                                        for _ in 0..r.repetitions{
                                            failure_time_list.push(failure_time);
                                            failure_time+= r.interval as f32;
                                        }
                                        failure_time_list.push(failure_time);

                                    },
                                    None => {
                                        let failure_time = (ap.start_sec + ap.duration_sec) as f32;
                                        failure_time_list.push(failure_time);
                                    },
                                }
                            });
                        let recovery_times_list = compute_recovery_times(&data, &failure_time_list);


                        let recovery_vec = rt_hash.entry(d.sensor.to_string()).or_insert(Vec::new());
                        // Append new values
                        recovery_vec.extend(recovery_times_list);

                    });

                rt_hash.clone().into_iter()
                    .for_each(|(k, v)|{
                        let values: Vec<f32> = v.iter()
                            .filter(|x| x.is_some())
                            .map(|x|{
                                x.unwrap()
                            })
                            .collect();
                        let stat = StatisticalMetrics::from_values(&values, true);
                        
                        all_rt_values.extend(values.iter());

                        rt_hash_sensor.insert(k, stat);

                });

                
                let rt = StatisticalMetrics::from_values(&all_rt_values, true);

                return Ok(RecoveryTime{
                    hash_list_period: rt_hash,
                    hash_list_sensor: rt_hash_sensor,
                    rt,
                });
            },
        TestType::Cut(cut_params) => {
                //Compute stuff for each cut.active_periods
                cut_params.cut_list
                    .iter()
                    .for_each(|c|{
                        let mut failure_time_list: Vec<f32> = vec![];

                        //Push the timestamps of end of failures to then search from there.
                        c.active_periods
                            .iter()
                            .for_each(|ap|{                                
                                match &ap.repeat {
                                    Some(r) => {
                                        let mut failure_time = (ap.start_sec + ap.duration_sec) as f32;
                                        for _ in 0..r.repetitions{
                                            failure_time_list.push(failure_time);
                                            failure_time+= r.interval as f32;
                                        }
                                        failure_time_list.push(failure_time);

                                    },
                                    None => {
                                        let failure_time = (ap.start_sec + ap.duration_sec) as f32;
                                        failure_time_list.push(failure_time);
                                    },
                                }
                            });
                        let recovery_times_list = compute_recovery_times(&data, &failure_time_list);


                        let recovery_vec = rt_hash.entry(c.sensor.to_string()).or_insert(Vec::new());
                        // Append new values
                        recovery_vec.extend(recovery_times_list);

                    });

                rt_hash.clone().into_iter()
                    .for_each(|(k, v)|{
                        let values: Vec<f32> = v.iter()
                            .filter(|x| x.is_some())
                            .map(|x|{
                                x.unwrap()
                            })
                            .collect();
                        let stat = StatisticalMetrics::from_values(&values, true);
                        
                        all_rt_values.extend(values.iter());

                        rt_hash_sensor.insert(k, stat);

                });

                
                let rt = StatisticalMetrics::from_values(&all_rt_values, true);

                return Ok(RecoveryTime{
                    hash_list_period: rt_hash,
                    hash_list_sensor: rt_hash_sensor,
                    rt,
                });

        },
        _ => return Err(MetricError::IOError("Wrong test type for the robustness metric.".to_owned())),

    }

}

fn compute_recovery_times(
    matched_data: &Vec<(&StatisticalMetricsStamped, &StatisticalMetricsStamped)>,
    failure_end_timestamps: &Vec<f32>
) -> Vec<Option<f32>> {

    let mut recovery_times = Vec::new();

    for &failure_end in failure_end_timestamps {
        let mut recovered_time: Option<f32> = None;

        for (test, baseline) in matched_data {
            if test.timestamp >= failure_end {

                if test.stat.mean * 0.9 <= baseline.stat.mean {
                    recovered_time = Some(test.timestamp - failure_end);
                    break;
                }
            }
        }

        recovery_times.push(recovered_time);
    }

    recovery_times
}