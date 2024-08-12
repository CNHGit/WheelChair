import os
import launch
from launch import launch_description_sources
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import launch_ros
from launch import actions
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    arg_sim = launch_ros.actions.SetParameter(name='use_sim_time', value=False)

    hw_description_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wheel_chair'),
                'launch',
                'wheelchair.launch.py'
            ])
        ]),
        launch_arguments={
            'rvizconfig': PathJoinSubstitution([
                FindPackageShare('navigation'),
                'rviz',
                'nav.rviz'
            ])
        }.items()
    )


    #Launch argument that enables or disables Vision sensors
    #Enable 2D Sensors------------------------------------------------
    DeclareLaunchArgument(
        "two_d_slam", 
        default_value='false',
        description='Determines Lidar packages and SLAM should be enabled'
    )
    #-----------------------------------------------------------------
    #Enable 3D Sensors------------------------------------------------
    DeclareLaunchArgument(
        "three_d_slam", 
        default_value='false',
        description='Determines if Realse sense based slam should be enabled'
    )
    #-----------------------------------------------------------------

    

    #Launch SLAM package----------------------------------------------
    perception_dir = get_package_share_directory('perception')
    #Launch Realsense Package-----------------------------------------
    perception_realsense_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
                perception_dir + '/launch/realsense2.launch.py'),
        condition=IfCondition(LaunchConfiguration('three_d_slam')))
    #-----------------------------------------------------------------
    #Launch RPLidar Package-------------------------------------------
    perception_rplidar_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
                perception_dir + '/launch/rplidar.launch.py'),
        condition=IfCondition(LaunchConfiguration('two_d_slam')))
    #-----------------------------------------------------------------

    
    #Launch SLAM package----------------------------------------------
    slam_dir = get_package_share_directory('slam')
    #Launch 3D SLAM package-------------------------------------------
    slam_dir = get_package_share_directory('slam')
    slam_realsense_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
                slam_dir + '/launch/slam_wheel_chair.launch.py'),
        condition=IfCondition(LaunchConfiguration('three_d_slam')))
    #-----------------------------------------------------------------
    #Launch 2D SLAM package-------------------------------------------
    slam_rplidar_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
                slam_dir + '/launch/cartographer.launch.py'),
        condition=IfCondition(LaunchConfiguration('two_d_slam')))

    slam_rplidar_octomap_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
                slam_dir + '/launch/occupancy_grid.launch.py'),
        condition=IfCondition(LaunchConfiguration('two_d_slam')))
    #-----------------------------------------------------------------
    


    



    #Make launch description
    ld = launch.LaunchDescription()

    #Add nodes
    ld.add_action(arg_sim)
    ld.add_action(hw_description_launch)
    ld.add_action(perception_rplidar_launch)
    ld.add_action(slam_rplidar_launch)
    ld.add_action(slam_rplidar_octomap_launch)
    ld.add_action(perception_realsense_launch)
    ld.add_action(slam_realsense_launch)

    return ld
