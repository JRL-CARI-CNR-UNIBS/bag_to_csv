# Copyright 2024 National Research Council STIIMA
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

import importlib
import sys
from pathlib import Path

import pandas as pd
import rclpy
from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_share_directory)
from rclpy.node import Node
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore


class BagToCsvNode(Node):
    def __init__(self):
        super().__init__('bag_to_csv_node')

        self.declare_parameter('bag_file_path', '')
        self.declare_parameter('output_csv_path', '')
        self.declare_parameter('packages_to_register', [''])
        self.declare_parameter('extractors_to_import', [''])

        bag_file_path = self._get_param('bag_file_path').string_value
        csv_file_path = self._get_param('output_csv_path').string_value
        packages_to_register = self._get_param('packages_to_register').string_array_value
        extractors_to_import = self._get_param('extractors_to_import').string_array_value

        if not bag_file_path or not csv_file_path:
            self.get_logger().error('Both bag_file_path and csv_file_path parameters are required.')
            sys.exit(1)
        
        self.typestore = get_typestore(Stores.ROS2_HUMBLE)
        try:
            self.register_message_types(packages_to_register)
        except Exception as e:
            self.get_logger().error(f'Error registering message types: {e}')
            sys.exit(1)
        
        self.info_extractors = {}
        self.topics = []
        for extractor in extractors_to_import:
            self.declare_parameter(f'{extractor}.package_name', '')
            self.declare_parameter(f'{extractor}.module_name', '')
            self.declare_parameter(f'{extractor}.class_name', '')
            self.declare_parameter(f'{extractor}.topics', [''])

            package_name = self._get_param(f'{extractor}.package_name').string_value
            module_name = self._get_param(f'{extractor}.module_name').string_value
            class_name = self._get_param(f'{extractor}.class_name').string_value

            topics_name = self._get_param(f'{extractor}.topics').string_array_value
            self.topics.extend(topics_name)
            
            try:
                module = importlib.import_module(f'{package_name}.{module_name}')
                if extractor not in self.info_extractors:
                    extractor = getattr(module, class_name)()
                    self.info_extractors[extractor.get_msg_type()] = extractor
            except Exception as e:
                self.get_logger().error(f'Error importing extractor: {e}')
                sys.exit(1)

        self.convert_bag_to_csv(bag_file_path, csv_file_path)
    
    def _get_param(self, param_name):
        return self.get_parameter(param_name).get_parameter_value()
    
    def register_message_types(self, packages_to_register):
        for package in packages_to_register:
            try:
                package_share_directory = get_package_share_directory(package)
            except (PackageNotFoundError, ValueError) as exception:
                raise exception
            self.declare_parameter(package, 'msg')
            msgs_folder = self._get_param(package).string_value
            msgs_folder_path = Path(f'{package_share_directory}/{msgs_folder}') 

            for msg_file_path in msgs_folder_path.glob('*.msg'):
                msg_name = msg_file_path.stem
                msg_text = msg_file_path.read_text()
                self.typestore.register(get_types_from_msg(msg_text, f'{package}/{msgs_folder}/{msg_name}'))
                self.get_logger().info(f'Registered message type: {msg_name}')

    def _extract_info_from_msg(self, msg, msg_type):
        return self.info_extractors[msg_type].extract_info_from_msg(msg, msg_type)
    
    def _extract_data(self, reader, connections):
        data = []
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = self.typestore.deserialize_cdr(rawdata, connection.msgtype)
            # extract_info_from_msg(msg, connection.msgtype)
            data.append(
                self._extract_info_from_msg(msg, connection.msgtype)
            )
        return data
    
    def convert_bag_to_csv(self, bag_file_path, csv_file_path):
        with AnyReader([Path(bag_file_path)]) as reader:
            for topic_name in self.topics:
                topic_connections = [x for x in reader.connections if x.topic == topic_name]
                topic_data = self._extract_data(reader, topic_connections)
                df = pd.DataFrame(topic_data)
                print(f'{csv_file_path}exported_{topic_name}.csv')
                df.to_csv(f'{csv_file_path}exported_{topic_name}.csv', index=False)
                self.get_logger().info(f'CSV file created for {topic_name}: {csv_file_path}')
    
def main(args=None):
    rclpy.init(args=args)
    node = BagToCsvNode()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
