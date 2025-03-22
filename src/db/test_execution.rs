use surrealdb::{Surreal, engine::remote::ws::Client};
use crate::{models::test_execution::TestExecution, services::error::DbError};

pub struct TestExecutionRepo {
    conn: Surreal<Client>,
}

impl TestExecutionRepo {
    pub fn new(conn: Surreal<Client>) -> Self {
        Self { conn }
    }

    //pub async fn save(&self, execution: &TestExecution) -> Result<(), DbError> {
    //    self.conn
    //        .create(("test_execution", &execution.id))
    //        .content(execution)
    //        .await
    //        .map_err(|e| DbError::Operation(e))
    //}

    pub async fn get(&self, id: &str) -> Result<Option<TestExecution>, DbError> {
        self.conn
            .select(("test_execution", id))
            .await
            .map_err(|e| DbError::Operation(e))
    }

    pub async fn list_active(&self) -> Result<Vec<TestExecution>, DbError> {
        self.conn
            .query("SELECT * FROM test_execution WHERE status IN ['Scheduled', 'Running']")
            .await?
            .take(0)
            .map_err(|e| DbError::Operation(e))
    }
}