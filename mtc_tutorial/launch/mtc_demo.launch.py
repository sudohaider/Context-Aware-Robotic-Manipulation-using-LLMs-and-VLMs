import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
import xacro
# import yamls

def generate_launch_description():

    panda_description_path = get_package_share_directory('moveit_resources_panda_description')
    # urdf_file_path = "urdf/panda.urdf.xacro"
    # xacro_file = '/home/ajeeb/ros2ws/src/moveit_resources/panda_moveit_config/config/panda.urdf.xacro'
    urdf_file_paths = os.path.join("/home/ajeeb/ros2ws/src/moveit_resources/panda_moveit_config", 'config', 'panda.urdf.xacro')

    # xacro_file = os.path.join(panda_description_path,
    #                           'urdf',
    #                           'panda.urdf.xacro')
    # urdf_file_path = xacro_file
    # # Generate ROBOT_DESCRIPTION for PANDA ROBOT:
    # doc = xacro.parse(open(xacro_file))
    # cell_layout_1 = "true"
    # cell_layout_2 = "false"
    # EE_no = "true"
    # xacro.process_doc(doc, mappings={
    #     "cell_layout_1": cell_layout_1,
    #     "cell_layout_2": cell_layout_2,
    #     "EE_no": EE_no,
    #     # "EE_**": EE_**,
    #     })
    # robot_description_config = doc.toxml()
    # robot_description = {'robot_description': robot_description_config}

    # gazebo_node = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'panda'],
    #                     output='screen')

    # gazebo_node = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', 'panda', '-topic', '/robot_description', '-x', '0.0', '-y', '0.0'],
    #     # arguments=['-entity', 'panda', '-file', urdf_file_path],
    #     output='screen'
    # )


    # # GAZEBO LAUNCH

    # # Get the Gazebo ROS package path
    # gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # # Define the path to the empty world file
    # empty_world_path = os.path.join(gazebo_ros_pkg, 'worlds', 'empty.world')

    # # Declare the world file launch argument
    # declare_world_cmd = DeclareLaunchArgument(
    #     'world',
    #     default_value=empty_world_path,
    #     description='Full path to the world model file to load')

    # # Start Gazebo with the libgazebo_ros_factory.so plugin
    # start_gazebo_cmd = ExecuteProcess(
    #     cmd=['gazebo', '-s', 'libgazebo_ros_factory.so'],
    #     output='screen'
    # )

    # # GAZEBO LAUNCH END



    # planning_context
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro") # Should we subscribe to the robot_desctiption OR pass URDF?
        # .robot_description(robot_description_config)
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    # # RViz
    rviz_config_file = (
        get_package_share_directory("moveit2_tutorials") + "/launch/mtc.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )


    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        # arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
        arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "panda_arm_controller",
        "panda_hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]
    


    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
        ]
        + load_controllers
    )