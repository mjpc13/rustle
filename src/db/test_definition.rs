use surrealdb::{Surreal, engine::remote::ws::Client};
use crate::models::TestDefinition;

pub struct TestDefinitionRepo {
    conn: Surreal<Client>,
}

impl TestDefinitionRepo {
    //pub async fn save(&self, def: &TestDefinition) -> Result<(), surrealdb::Error> {
    //    self.conn
    //        .create(("test_definition", &def.id))
    //        .content(def)
    //        .await
    //}

    pub async fn get(&self, id: &str) -> Result<Option<TestDefinition>, surrealdb::Error> {
        self.conn.select(("test_definition", id)).await
    }

    //pub async fn list_by_type(
    //    &self, 
    //    test_type: &str
    //) -> Result<Vec<TestDefinition>, surrealdb::Error> {
    //    self.conn
    //        .query("SELECT * FROM test_definition WHERE test_type.type = $type")
    //        .bind(("type", test_type))
    //        .await?
    //        .take(0)
    //}
}