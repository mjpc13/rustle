use pyo3::{prelude::*};
use pyo3::types::IntoPyDict;

use crate::services::error::RunError;

const PY_CODE: &str = r#"
import os
import csv
from pathlib import Path
from rosbags.highlevel import AnyReader

def read_robag(bag_path, gt_path, topic, is_ros2_bag):
    """
    Read all messages from a topic across multiple ROS bag files in a directory
    using the 'rosbags' Python package, and write them to a single CSV.
    """
    bag_dir = Path(bag_path)
    if not bag_dir.exists():
        raise FileNotFoundError(f"Directory does not exist: {bag_dir}")

    bag_files = (
            sorted([p for p in bag_dir.iterdir() if p.suffix == '.bag'])
            if not is_ros2_bag else
            [bag_dir]
        )

    if not bag_files:
        raise FileNotFoundError(f"No .bag files found in: {bag_dir}")


    with open(gt_path, "w", newline="") as csvfile:

        with AnyReader(bag_files) as reader:

            connections = [c for c in reader.connections if c.topic == topic]
            if not connections:
                topic_list = ", ".join([c.topic for c in reader.connections])
                raise Exception(f"no topic named {topic} was found, the topic list is {topic_list}")

            writer, header_written = None, False

            for conn, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, conn.msgtype)
                data = {"time": timestamp * 1e-9}  # convert ns → seconds

                # Handle Odometry and PoseStamped
                if hasattr(msg, "pose"):
                    pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
                    data.update({
                        "x": pose.position.x,
                        "y": pose.position.y,
                        "z": pose.position.z,
                        "qx": pose.orientation.x,
                        "qy": pose.orientation.y,
                        "qz": pose.orientation.z,
                        "qw": pose.orientation.w,
                    })

                # Handle Pose
                elif hasattr(msg, "position") and hasattr(msg, "orientation"):
                    data.update({
                        "x": msg.position.x,
                        "y": msg.position.y,
                        "z": msg.position.z,
                        "qx": msg.orientation.x,
                        "qy": msg.orientation.y,
                        "qz": msg.orientation.z,
                        "qw": msg.orientation.w,
                    })

                # Handle Point or position-only message
                elif hasattr(msg, "x") and hasattr(msg, "y") and hasattr(msg, "z"):
                    data.update({"x": msg.x, "y": msg.y, "z": msg.z})

                # Initialize writer and write header once
                if not header_written:
                    fieldnames = list(data.keys())
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames, delimiter=' ')
                    #writer.writeheader()
                    header_written = True

                # Write the actual data row
                writer.writerow(data)

"#;


pub fn read_rosbag_py(
    bag_path: &str,
    gt_path: &str,
    topic: &str,
    is_ros2_bag: Option<bool>,
) -> Result<(), RunError>{
    Python::with_gil(|py| {
        // Create a Python module from the embedded code
        let embedded_module = PyModule::from_code(py, PY_CODE, "embedded_module", "embedded_module").unwrap();

        // Prepare arguments
        let kwargs = [
            ("bag_path", bag_path.to_object(py)),
            ("gt_path", gt_path.to_object(py)),
            ("topic", topic.to_object(py)),
            ("is_ros2_bag", is_ros2_bag.unwrap_or_default().to_object(py)),
        ].into_py_dict(py);

        // Call the Python function
        embedded_module
            .getattr("read_robag").unwrap()
            .call((), Some(kwargs)).map_err(|e| RunError::Execution(format!("Unable to retrieve ground truth measurements from the rosbag with error: {e}.")))?;
        Ok(())
    })
}
