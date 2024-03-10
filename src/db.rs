use std::{collections::BTreeMap, sync::Arc};

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use surrealdb::{
    sql::{thing, Array, Object, Value},
    dbs::Session,
    kvs::Datastore,
    Response
};

use bollard::container::{MemoryStats, CPUStats};



#[derive(Debug, Serialize, Deserialize)]
pub struct Stats {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    pub title: String,
    pub memory_stats: MemoryStats,
    pub cpu_stats: CPUStats,
    pub num_procs: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub created_at: Option<DateTime<Utc>>,
}

// impl From<Stats> for Value {
//     fn from(val: Stats) -> Self {
//         match val.id {
//             Some(v) => map![
//                     "id".into() => v.into(),
//                     "title".into() => val.title.into(),
//                     "memory".into() => val.memory_stats.into(),
//                     "cpu".into() => val.cpu_stats.into(),
//                     "num_procs".into() => val.num_procs.into()
//             ]
//             .into(),
//             None => map![
//                 "title".into() => val.title.into(),
//                 "memory".into() => val.memory_stats.into(),
//                 "cpu".into() => val.cpu_stats.into(),
//                 "num_procs".into() => val.num_procs.into()
//             ]
//             .into(),
//         }
//     }
// }

#[derive(Debug, Serialize, Deserialize)]
pub struct Metrics{
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    pub ape: u32,
    pub rte: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub created_at: Option<DateTime<Utc>>,
}


#[derive(Clone)]
pub struct DB{
    pub ds: Arc<Datastore>,
    pub sesh: Session,
}
