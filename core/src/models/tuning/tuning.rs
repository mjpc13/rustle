use crate::models::tuning::{GridSearchConfig, SimulatedAnnealingConfig};
use argmin::core::ArgminFloat;
use rand_xoshiro::Xoshiro256PlusPlus;
use rand::Rng;

// by default, each configuration is run as a "simple" test

#[derive(Clone, PartialEq, serde::Serialize, serde::Deserialize, Debug)]
pub enum TuneType {
    GridSearch(GridSearchConfig),
    RandomSearch(GridSearchConfig),
    SimulatedAnnealing(SimulatedAnnealingConfig),
}

impl TuneType {

    pub fn as_search(&self) -> Result<&GridSearchConfig, ()> {
        match self {
            TuneType::GridSearch(grid_search_config) => Ok(grid_search_config),
            TuneType::RandomSearch(random_search_config) => Ok(random_search_config),
            _ => Err(()),
        }
    }

    pub fn as_grid_search(&self) -> Result<&GridSearchConfig, ()> {
        match self {
            TuneType::GridSearch(grid_search_config) => Ok(grid_search_config),
            _ => Err(()),
        }
    }

    pub fn as_grid_search_as_mut(&mut self) -> Result<&mut GridSearchConfig, ()> {
        match self {
            TuneType::GridSearch(grid_search_config) => Ok(grid_search_config),
            _ => Err(()),
        }
    }

    pub fn as_random_search(&self) -> Result<&GridSearchConfig, ()> {
        match self {
            TuneType::RandomSearch(random_search_config) => Ok(random_search_config),
            _ => Err(()),
        }
    }

    pub fn as_random_search_as_mut(&mut self) -> Result<&mut GridSearchConfig, ()> {
        match self {
            TuneType::RandomSearch(random_search_config) => Ok(random_search_config),
            _ => Err(()),
        }
    }

    pub fn as_simulated_annealing(&self) -> Result<&SimulatedAnnealingConfig, ()> {
        match self {
            TuneType::SimulatedAnnealing(simulated_annealing_config) => Ok(simulated_annealing_config),
            _ => Err(()),
        }
    }

    pub fn as_simulated_annealing_as_mut(&mut self) -> Result<&mut SimulatedAnnealingConfig, ()> {
        match self {
            TuneType::SimulatedAnnealing(simulated_annealing_config) => Ok(simulated_annealing_config),
            _ => Err(()),
        }
    }

}