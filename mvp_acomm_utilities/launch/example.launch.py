
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    robot_name = 'ship'
    urdf_file_name = 'ship.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('mvp_acomm_utilities'),'config', 'urdf', urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    print(urdf)

    return LaunchDescription([
        Node(
            package='mvp_acomm_utilities',
            executable='acomm_geopoint_node',
            name='acomm_geopoint_node',
            namespace=robot_name,
            output='screen',
            prefix=['stdbuf -o L'],
            parameters=[
                # {'tf_prefix': robot_name},
                {'use_reference_geopose_orientation': True},
                {'geopose_frame_id': 'gps'},
                ],
            remappings=[
                    ('reference_geopose', 'gps'),
                ],
           ),

        #    #USBL to ship TF setup
        #    Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='ship2usbl',
        #     arguments = ["-8.22", "-18.586", "17.684", "-1.57079632679", "0.0", "0.0", robot_name+'/gps', robot_name+'/usbl']    
        #     #              x, y,z,yaw,pitch,roll
        #     ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]),
])
