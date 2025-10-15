use crate::models::tuning::grid_search::GridSearchConfig;

// by default, each configuration is run as a "simple" test

pub enum TuneType {
    GridSearch(GridSearchConfig),
    RandomSearch(),
}