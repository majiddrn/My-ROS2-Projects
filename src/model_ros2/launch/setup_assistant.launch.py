from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("FUMTI_1401FT_GAZEBOURDF_SLDASM", package_name="model_ros2").to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)
