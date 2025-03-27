use surrealdb::{Surreal, engine::remote::ws::Client};
use crate::models::Container;

pub struct ContainerRepo {
    conn: Surreal<Client>,
}

impl ContainerRepo {
    pub fn new(conn: Surreal<Client>) -> Self {
        Self { conn }
    }

    //pub async fn save(&self, container: &Container) -> Result<(), surrealdb::Error> {
    //    self.conn
    //        .create(("container", &container.id))
    //        .content(container)
    //        .await
    //}

    //pub async fn get_by_name(&self, name: &str) -> Result<Option<Container>, surrealdb::Error> {
    //    self.conn
    //        .query("SELECT * FROM container WHERE name = $name")
    //        .bind(("name", name))
    //        .await?
    //        .take(0)
    //}

    //pub async fn list_by_image(
    //    &self,
    //    image_name: &str
    //) -> Result<Vec<Container>, surrealdb::Error> {
    //    self.conn
    //        .query("SELECT * FROM container WHERE image_name = $image")
    //        .bind(("image", image_name))
    //        .await?
    //        .take(0)
    //}
}