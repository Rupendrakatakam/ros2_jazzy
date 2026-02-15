import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros import actions
import os




def generate_launch_description():
    # Get the path to the URDF file

    packageName = 'my_robot_description'

    urdfRelativePath = 'urdf/my_robot.urdf'

    rvizRelativePath = 'config/config.rviz'

    pkgPath = FindPackageShare(package=packageName).find(packageName)

    urdfModelPath = os.path.join(pkgPath, urdfRelativePath)
    rvizConfigPath = os.path.join(pkgPath, rvizRelativePath)


    print("URDF Model Path: ", urdfModelPath)

    with open(urdfModelPath, 'r') as infp:
        robot_desc = infp.read()

    params = {'robot_description': robot_desc}

    robot_state_publisher_node = actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
            arguments=[urdfModelPath]
    )

    rviz_node = actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizConfigPath]
    )

    joint_state_publisher_gui_node = actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name = 'gui' , default_value = 'true', description = 'Flag to enable GUI'),


        robot_state_publisher_node,

        joint_state_publisher_gui_node ,
        rviz_node,

    ])

   

   
