use thiserror::Error;

#[derive(Debug, Error)]
pub enum RosError {
    #[error("Parse error: {from} -> {to}")]
    ParseError {
        from: String,
        to: String
    },

    #[error("Missing header in {rostype}")]
    MissingHeader {
        rostype: String
    },

    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
}