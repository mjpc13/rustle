

// I maybe need to add algorithm and test setup???
#[derive(Debug, Clone)]
pub struct ProgressMessage {
    pub iteration_num: usize,
    pub algo: String,
    pub bag_time: f64,
    pub duration: f64,
    pub total_duration: f64,
}

