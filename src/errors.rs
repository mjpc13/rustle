use custom_error::custom_error;
use std::sync::Arc;

custom_error!{pub RosError
    ParseError{name: Arc<str>, value: Arc<str>} = "Cannot format {value} to: {name}",
    FormatError{name: Arc<str>} = "Invalid ROS message format: {name}",
    EmptyCovariance{} = "The covariance matrix is empty",
}

custom_error!{pub EvoError
    CommandError{stderr: Arc<str>} = "Could not run evo tool: {stderr}",
}

