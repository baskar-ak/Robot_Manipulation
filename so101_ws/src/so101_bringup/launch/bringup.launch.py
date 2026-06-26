from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "so101_new_calib",
            package_name="so101_moveit_config"
        )
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .trajectory_execution()
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 1. Robot state publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # 2. ros2_control node
    ros2_ctrl = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                get_package_share_directory("so101_moveit_config"),
                "config/ros2_controllers.yaml"
            ),
        ],
    )

    # 3. Spawn joint_state_broadcaster
    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # 4. Spawn arm controller
    arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    # 5. Spawn gripper controller
    gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )

    # 6. move_group — delayed 3s to let controllers start
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # 7. RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([
        rsp,
        ros2_ctrl,
        jsb,
        arm,
        gripper,
        TimerAction(period=3.0, actions=[move_group]),
        rviz,
    ])