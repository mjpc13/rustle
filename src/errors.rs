
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("Value not of type '{0}'")]
    XValueNotOfType(&'static str),

    #[error(transparent)]
    Surreal(#[from] surrealdb::Error),

    #[error(transparent)]
    IO(#[from] std::io::Error),

    #[error("Failed to deserialize JSON: {message}")]
    JsonDataError {
        /// Short section of the json close to the error.
        message: String,
        /// Entire JSON payload. This field is toggled with the **json_data_content** feature cargo flag.
        #[cfg(feature = "json_data_content")]
        contents: String,
        /// Character sequence at error location.
        column: usize,
    },
}
