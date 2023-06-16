import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    if not os.environ.get("TURTLEBOT3_MODEL"):
        os.environ["TURTLEBOT3_MODEL"] = "burger"

    # declare parameters
    #   this->declare_parameter<double>("sigma", 1.0);
    #   this->declare_parameter<double>("Kp", 1.0);
    #   this->declare_parameter<double>("Ka", 4.0);
    #   this->declare_parameter<double>("ksid", 1.0);
    #   this->declare_parameter<double>("v_d", -0.25);
    #   this->declare_parameter<double>("max_speed", 1.5);

    sigma = LaunchConfiguration("sigma", default="1.0")
    Kp = LaunchConfiguration("Kp", default="1.0")
    Ka = LaunchConfiguration("Ka", default="2.0")
    ksid = LaunchConfiguration("ksid", default="1.0")
    v_d = LaunchConfiguration("v_d", default="-0.25")
    max_speed = LaunchConfiguration("max_speed", default="2.0")

    # declare launch arguments

    sigma_arg = DeclareLaunchArgument("sigma", default_value=sigma, description="sigma")
    Kp_arg = DeclareLaunchArgument("Kp", default_value=Kp, description="Kp")
    Ka_arg = DeclareLaunchArgument("Ka", default_value=Ka, description="Ka")
    ksid_arg = DeclareLaunchArgument("ksid", default_value=ksid, description="ksid")
    v_d_arg = DeclareLaunchArgument("v_d", default_value=v_d, description="v_d")
    max_speed_arg = DeclareLaunchArgument(
        "max_speed", default_value=max_speed, description="max_speed"
    )

    path_type = os.environ.get("PATH_TYPE", "ellipse")

    launch_file_dir = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")

    world = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "worlds", "empty_world.world"
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    vfo_controller = Node(
        package="python_vfo",
        executable="vfo",
        name="vfo",
        output="screen",
    )
    vfo_cpp = Node(
        package="cpp_vfo",
        executable="vfo",
        name="vfo",
        output="screen",
        parameters=[
            {"sigma": sigma},
            {"Kp": Kp},
            {"Ka": Ka},
            {"ksid": ksid},
            {"v_d": v_d},
            {"max_speed": max_speed},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(get_package_share_directory("tb3_sim"), "rviz", "vfo.rviz"),
        ],
    )

    return LaunchDescription(
        [
            gzserver_cmd,
            gzclient_cmd,
            robot_state_publisher_cmd,
            rviz,
            spawn_turtlebot_cmd,
            sigma_arg,
            Kp_arg,
            Ka_arg,
            ksid_arg,
            v_d_arg,
            max_speed_arg,
            # vfo_controller,
            vfo_cpp,
        ]
    )
