from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_scan_context_loop_closure", default_value="false"),
        DeclareLaunchArgument("use_gps_factor", default_value="false"),
        DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered"),
        DeclareLaunchArgument("odom_topic", default_value="/Odometry"),
        DeclareLaunchArgument("gps_topic", default_value="/fmu/out/vehicle_gps_position"),
        DeclareLaunchArgument("map_file_path", default_value="/tmp/p30_pgo_map.pcd"),
        DeclareLaunchArgument("save_directory", default_value="/tmp/p30_pgo/"),
        DeclareLaunchArgument("sc_dist_thres", default_value="0.15"),
        DeclareLaunchArgument("sc_max_radius", default_value="10.0"),
        DeclareLaunchArgument("gps_min_fix_type", default_value="3.0"),
        DeclareLaunchArgument("gps_horizontal_sigma_m", default_value="1.0"),
        DeclareLaunchArgument("gps_vertical_sigma_m", default_value="2.0"),
        DeclareLaunchArgument("gps_sigma_inflate", default_value="2.0"),
        DeclareLaunchArgument("gps_huber_threshold_m", default_value="3.0"),
        DeclareLaunchArgument("gps_anchor_warmup_sec", default_value="5.0"),
        DeclareLaunchArgument("gps_match_window_sec", default_value="0.05"),
    ]

    pgo = Node(
        package="aloam_velodyne",
        executable="alaserPGO",
        name="laserPGO",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_scan_context_loop_closure": LaunchConfiguration("use_scan_context_loop_closure"),
            "use_gps_factor": LaunchConfiguration("use_gps_factor"),
            "cloud_topic": LaunchConfiguration("cloud_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "gps_topic": LaunchConfiguration("gps_topic"),
            "map_file_path": LaunchConfiguration("map_file_path"),
            "save_directory": LaunchConfiguration("save_directory"),
            "sc_dist_thres": LaunchConfiguration("sc_dist_thres"),
            "sc_max_radius": LaunchConfiguration("sc_max_radius"),
            "gps_min_fix_type": LaunchConfiguration("gps_min_fix_type"),
            "gps_horizontal_sigma_m": LaunchConfiguration("gps_horizontal_sigma_m"),
            "gps_vertical_sigma_m": LaunchConfiguration("gps_vertical_sigma_m"),
            "gps_sigma_inflate": LaunchConfiguration("gps_sigma_inflate"),
            "gps_huber_threshold_m": LaunchConfiguration("gps_huber_threshold_m"),
            "gps_anchor_warmup_sec": LaunchConfiguration("gps_anchor_warmup_sec"),
            "gps_match_window_sec": LaunchConfiguration("gps_match_window_sec"),
        }],
    )

    return LaunchDescription(args + [pgo])
