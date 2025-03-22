





#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestExecution{
    #[serde(default = "Utc::now")]
    created_at: DateTime<Utc>,
}