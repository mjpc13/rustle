use bollard::container::{CPUStats, MemoryStats};
use chrono::{DateTime, Utc};
use serde::{Serialize, Deserialize};
use surrealdb::sql::Thing;
use std::cmp::Ordering;


#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ContainerStats {
    pub id: Option<Thing>,  // SurrealDB ID
    pub memory_stats: MemoryStats,
    pub cpu_stats: CPUStats,
    pub precpu_stats: CPUStats,
    pub num_procs: u32,
    pub created_at: DateTime<Utc>,
}

impl ContainerStats {
    pub fn new(
        memory_stats: MemoryStats,
        cpu_stats: CPUStats,
        precpu_stats: CPUStats,
        num_procs: u32,
    ) -> Self {
        Self {
            id: None,
            memory_stats,
            cpu_stats,
            precpu_stats,
            num_procs,
            created_at: Utc::now(),
        }
    }
}

impl PartialEq for ContainerStats {
    fn eq(&self, other: &Self) -> bool {
        self.created_at == other.created_at
    }
}

impl Eq for ContainerStats {}

impl PartialOrd for ContainerStats {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.created_at.cmp(&other.created_at))
    }
}

impl Ord for ContainerStats {
    fn cmp(&self, other: &Self) -> Ordering {
        self.created_at.cmp(&other.created_at)
    }
}