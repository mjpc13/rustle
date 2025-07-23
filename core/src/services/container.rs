use crate::{models::Container, db::ContainerRepo};
use super::error::ValidationError;

pub struct ContainerService {
    _repo: ContainerRepo,
}

impl ContainerService {
    pub fn new(repo: ContainerRepo) -> Self {
        Self { _repo: repo }
    }

    //pub async fn create_container(
    //    &self,
    //    image_name: String,
    //    name: String,
    //    tag: String,
    //    url: String
    //) -> Result<Container, ValidationError> {
    //    let container = Container::new(image_name, name, tag, url);
    //    self.validate(&container)?;
    //    self.repo.save(&container).await.map_err(|e| {
    //        ValidationError(format!("Failed to save container: {}", e))
    //    })?;
    //    Ok(container)
    //}

    fn _validate(&self, container: &Container) -> Result<(), ValidationError> {
        if container.name.is_empty() {
            return Err(ValidationError("Container name cannot be empty".into()));
        }
        
        if container.tag.is_empty() {
            return Err(ValidationError("Tag cannot be empty".into()));
        }

        container.validate_url()
            .map_err(|e| ValidationError(e))?;

        Ok(())
    }
}