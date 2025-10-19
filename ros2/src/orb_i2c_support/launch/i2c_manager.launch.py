"""Launch file for the ORB I2C manager node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="simulation",
        description="Backend to use for I2C transactions",
    )
    bus_arg = DeclareLaunchArgument(
        "bus_id",
        default_value="i2c-1",
        description="I2C bus identifier passed to the backend",
    )
    backend_plugin_arg = DeclareLaunchArgument(
        "backend_plugin",
        default_value="",
        description="Optional backend plugin in module:Class format",
    )
    backend_config_arg = DeclareLaunchArgument(
        "backend_configuration",
        default_value="{}",
        description="JSON configuration forwarded to the backend",
    )

    node = Node(
        package="orb_i2c_support",
        executable="i2c_manager",
        name="orb_i2c_manager",
        parameters=[
            {
                "backend": LaunchConfiguration("backend"),
                "bus_id": LaunchConfiguration("bus_id"),
                "backend_plugin": LaunchConfiguration("backend_plugin"),
                "backend_configuration": LaunchConfiguration("backend_configuration"),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [backend_arg, bus_arg, backend_plugin_arg, backend_config_arg, node]
    )
