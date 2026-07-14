from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    test_type = LaunchConfiguration('test_type')
    algo_topic = LaunchConfiguration('algo_topic')
    gt_topic = LaunchConfiguration('gt_topic')

    declare_test_type_cmd = DeclareLaunchArgument(
        'test_type', default_value='simple', description='Type of test to run')
    declare_algo_topic_cmd = DeclareLaunchArgument(
        'algo_topic', default_value='/kiss/odometry', description='Algorithm Odometry Topic')
    declare_gt_topic_cmd = DeclareLaunchArgument(
        'gt_topic', default_value='', description='Ground Truth Topic')

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': 57331, 'address': '0.0.0.0', 'use_sim_time': True}]
    )

    kiss_icp_node = Node(
        package='kiss_icp',
        executable='kiss_icp_node',
        name='kiss_icp_node',
        output='screen',
        parameters=[
            '/rustle/config/params.yaml',
            {'use_sim_time': True}
        ],
        remappings=[
            ('pointcloud_topic', '/aeva/ATLAS/point_cloud_compensated')
        ],
        condition=IfCondition(
            PythonExpression(["'", test_type, "' in ['simple', 'drop', 'cut']"])
        )
    )

    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen'
    )

    # drop_manager_node = Node(
    #     package='rustle_ros', 
    #     executable='drop_manager.py',
    #     name='drop_manager',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}],
    #     condition=IfCondition(PythonExpression(["'", test_type, "' == 'drop'"]))
    # )

    return LaunchDescription([
        declare_test_type_cmd,
        declare_algo_topic_cmd,
        declare_gt_topic_cmd,
        rosbridge_node,
        kiss_icp_node,
        rosapi_node,
        # drop_manager_node
    ])
