"""
This script generate a rosbag2 with:
    - a ground truth odometry topic
    - a point cloud 2 topic
The produce bag represents a lidar doing an 8m diameter cirlce 0.5m above the ground
inside of a 10x10x10m box room.
"""
import numpy as np
import shutil
from pathlib import Path

from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

def generate_static_box(dimension=10, density=0.25):
    """Generates a DxDxD meter box centered at (0,0,D/2), where D is the dimension parameter."""
    points = []
    xy_range = np.arange(density - dimension/2.0, dimension/2.0, density)
    z_range = np.arange(density, dimension, density)
    
    # Floor and Ceiling
    for x in xy_range:
        for y in xy_range:
            points.append([x, y, 0.0])
            points.append([x, y, dimension])
            
    # Left/Right Walls
    for y in xy_range:
        for z in z_range:
            points.append([-dimension/2.0, y, z])
            points.append([dimension/2.0, y, z])
            
    # Front/Back Walls
    for x in xy_range:
        for z in z_range:
            points.append([x, -dimension/2.0, z])
            points.append([x, dimension/2.0, z])
            
    return np.array(points, dtype=np.float32)

def main():
    bag_path = Path("./workspace/kiss_icp_demo/input_bag").resolve()
    if bag_path.exists():
        shutil.rmtree(bag_path)
    bag_path.parent.mkdir(parents=True, exist_ok=True)

    lidar_topic = "/ouster/points"
    odom_topic = "/ground_truth"
    
    # Simulation Parameters
    duration = 30.0   # Total length of bag in seconds
    fps = 10
    dt = 1.0 / fps
    total_steps = int(duration * fps)
    
    # Lidar Path Parameters (⌀8m circle -> Radius = 4m)
    radius = 4.0
    z_lidar = 0.5
    omega = (2.0 * np.pi) / duration # One full revolution over the duration
    
    world_points = generate_static_box(density=0.2)
    
    # Extract structural type classes from rosbags engine
    typestore = get_typestore(Stores.LATEST)
    PointCloud2 = typestore.types['sensor_msgs/msg/PointCloud2']
    PointField = typestore.types['sensor_msgs/msg/PointField']
    Header = typestore.types['std_msgs/msg/Header']
    Time = typestore.types['builtin_interfaces/msg/Time']
    
    # Odometry specific types
    Odometry = typestore.types['nav_msgs/msg/Odometry']
    PoseWithCovariance = typestore.types['geometry_msgs/msg/PoseWithCovariance']
    Pose = typestore.types['geometry_msgs/msg/Pose']
    Point = typestore.types['geometry_msgs/msg/Point']
    Quaternion = typestore.types['geometry_msgs/msg/Quaternion']
    TwistWithCovariance = typestore.types['geometry_msgs/msg/TwistWithCovariance']
    Twist = typestore.types['geometry_msgs/msg/Twist']
    Vector3 = typestore.types['geometry_msgs/msg/Vector3']

    # Define fields for PointCloud2 (XYZ float32)
    fields = [
        PointField(name='x', offset=0, datatype=7, count=1),
        PointField(name='y', offset=4, datatype=7, count=1),
        PointField(name='z', offset=8, datatype=7, count=1)
    ]

    print(f"Writing synthetic ROS2 bag to: {bag_path}/")
    
    with Writer(bag_path, version=8) as writer:
        # Register both topic channels in the metadata profile
        lidar_conn = writer.add_connection(lidar_topic, PointCloud2.__msgtype__, typestore=typestore)
        odom_conn = writer.add_connection(odom_topic, Odometry.__msgtype__, typestore=typestore)
        
        for step in range(total_steps):
            t = step * dt
            stamp_ns = int(t * 1e9)
            sec = int(stamp_ns // 1e9)
            nanosec = int(stamp_ns % 1e9)
            
            # -------------------------------------------------------------------------
            # 1. TRAJECTORY CALCULATIONS
            # -------------------------------------------------------------------------
            theta = omega * t
            tx = radius * np.cos(theta)
            ty = radius * np.sin(theta)
            
            heading = theta + (np.pi / 2.0)
            qz = np.sin(heading / 2.0)
            qw = np.cos(heading / 2.0)
            
            # -------------------------------------------------------------------------
            # 2. GENERATE POINT CLOUD (Local Sensor Frame)
            # -------------------------------------------------------------------------
            shifted = world_points - np.array([tx, ty, z_lidar], dtype=np.float32)
            cos_h, sin_h = np.cos(heading), np.sin(heading)
            
            x_local = shifted[:, 0] * cos_h + shifted[:, 1] * sin_h
            y_local = -shifted[:, 0] * sin_h + shifted[:, 1] * cos_h
            z_local = shifted[:, 2]
            
            local_points = np.column_stack((x_local, y_local, z_local)).astype(np.float32)
            flat_data = np.frombuffer(local_points.tobytes(), dtype=np.uint8)
            
            cloud_msg = PointCloud2(
                header=Header(Time(sec, nanosec), "base_link"),
                height=1,
                width=len(local_points),
                fields=fields,
                is_bigendian=False,
                point_step=12,
                row_step=12 * len(local_points),
                data=flat_data,
                is_dense=True
            )
            writer.write(lidar_conn, stamp_ns, typestore.serialize_cdr(cloud_msg, PointCloud2.__msgtype__))
            
            # -------------------------------------------------------------------------
            # 3. GENERATE ODOMETRY (Global Frame reference tracking moving sensor)
            # -------------------------------------------------------------------------
            # Linear velocity in local base_link frame: moving directly forward at v = R * omega
            v_forward = radius * omega
            empty_covariance = np.zeros((36,))
            
            odom_msg = Odometry(
                header=Header(Time(sec, nanosec), "odom"),
                child_frame_id="base_link",
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(tx, ty, z_lidar),
                        orientation=Quaternion(0.0, 0.0, qz, qw)
                    ),
                    covariance=empty_covariance
                ),
                twist=TwistWithCovariance(
                    twist=Twist(
                        linear=Vector3(v_forward, 0.0, 0.0),
                        angular=Vector3(0.0, 0.0, omega)
                    ),
                    covariance=empty_covariance
                )
            )
            writer.write(odom_conn, stamp_ns, typestore.serialize_cdr(odom_msg, Odometry.__msgtype__))
            
    print("Done.")

if __name__ == "__main__":
    main()
