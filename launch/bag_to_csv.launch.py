# Copyright 2024 National Research Counchil STIIMA & Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('bag_to_csv')

    declare_bag_file_path_cmd = DeclareLaunchArgument(
        'bag_file_path',
        default_value=pkg_dir + '/data/executions_data_first_test/executions_data_first_test_0.mcap',
        description='Full path to the bag file to analyze')
    declare_csv_output_path_cmd = DeclareLaunchArgument(
        'output_csv_path',
        default_value=pkg_dir + '/data/output.csv',
        description='Full path to the output CSV file')
    declare_config_file_path_cmd = DeclareLaunchArgument(
        'config_file_path',
        default_value=pkg_dir + '/config/config.yaml',
        description='Config file path to register custom msgs')
    
    bag_to_csv_node = Node(
        package='bag_to_csv',
        executable='bag_to_csv_node',
        name='bag_to_csv_node',
        output='screen',
        parameters=[
            {'bag_file_path': LaunchConfiguration('bag_file_path')},
            {'output_csv_path': LaunchConfiguration('output_csv_path')},
            LaunchConfiguration('config_file_path')
        ])
    
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_bag_file_path_cmd)
    ld.add_action(declare_csv_output_path_cmd)
    ld.add_action(declare_config_file_path_cmd)

    ld.add_action(bag_to_csv_node)
    return ld
