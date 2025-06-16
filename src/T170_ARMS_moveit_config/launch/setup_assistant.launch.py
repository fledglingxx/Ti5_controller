from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("T170_ARMS", package_name="set_ass").to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)
