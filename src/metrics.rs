

enum MetricError {
    APE,
    RTE,
}

pub struct APE{
    pub max: u32,
    pub median: u32,
    pub min: u32,
    pub rmse: u32,
    pub sse: u32,
    pub std: u32
}

pub struct RPE{
    pub max: u32,
    pub median: u32,
    pub min: u32,
    pub rmse: u32,
    pub sse: u32,
    pub std: u32
}


