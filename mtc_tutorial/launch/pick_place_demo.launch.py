from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import LogInfo

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()
    
    #moveit_config["stage_name"] = "open hand"
    #moveit_config["stage_name"] = "move to goal"
    #moveit_config["stage_name"] = "pick object"
    #moveit_config["stage_name"] = "move to place"
    #moveit_config["stage_name"] = "place object"


    # MTC Demo node
    pick_place_demo = Node(
        package="mtc_tutorial",
        executable="mtc_chuppa",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])
