import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction

from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import AppendEnvironmentVariable

def generate_launch_description():

    package_name='my_robot' #<--- CHANGE ME

    # Ensure Gazebo can find the package resources
    install_dir = get_package_share_directory(package_name)
    # GZ_SIM_RESOURCE_PATH requires the parent directory to find packages via package://
    gazebo_resource_path = os.path.dirname(install_dir)
    
    set_gazebo_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gazebo_resource_path
    )


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )


    
    # Include the Gazebo launch file, provided by the gazebo_ros package
    # Path to default world 
    world_path = os.path.join(get_package_share_directory(package_name),'worlds', 'world.sdf')

    # Launch Arguments
    declare_world = DeclareLaunchArgument(
        name='world', default_value=world_path,
        description='Full path to the world model file to load')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments={'gz_args': ['-r -v1 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Add the Bridge to connect ROS 2 and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"
        ],
        output='screen'
    )

    # Node to bridge camera image with image_transport and compressed_image_transport
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'camera.image.compressed.jpeg_quality': 75},
        ],
    )

    # Relay node to republish /camera/camera_info to /camera/image/camera_info
    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['camera/camera_info', 'camera/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_robot','-z', '0.15'],
                        output='screen')





    # Launch them all!
    return LaunchDescription([
        set_gazebo_resource_path,
        declare_world,
        
        rsp,
        gazebo,
        spawn_entity,
        bridge,
        gz_image_bridge_node,
        relay_camera_info_node,
            
    ])  