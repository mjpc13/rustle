use crate::db::{DB, Stats};
use surrealdb::{
    Surreal,
    engine::any
};

struct MetricError{
    max: i32,
    median: i32,
    min: i32,
    rmse: i32,
    sse: i32,
    std: i32
}

enum LocalizationMetric {
    APE(MetricError),
    RTE(MetricError)
}
