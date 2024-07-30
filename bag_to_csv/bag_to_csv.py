import rclpy
from rclpy.node import Node
from rosbags.rosbag2 import Reader
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
# from plansys2_msgs.msg import ActionExecutionDataCollection, PlanExecutionDataCollection
from pathlib import Path
import sys
from builtin_interfaces.msg import Time
import pandas as pd
# from rosidl_runtime_py.convert import get_message_slot_types, message_to_csv
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import importlib

class BagToCsvNode(Node):
    def __init__(self):
        super().__init__('bag_to_csv_node')

        self.declare_parameter('bag_file_path', '')
        self.declare_parameter('output_csv_path', '')
        self.declare_parameter('packages_to_register', [''])
        self.declare_parameter('extractors_to_import', [''])

        bag_file_path = self.get_parameter('bag_file_path').get_parameter_value().string_value
        csv_file_path = self.get_parameter('output_csv_path').get_parameter_value().string_value
        packages_to_register = self.get_parameter('packages_to_register').get_parameter_value().string_array_value
        extractors_to_import = self.get_parameter('extractors_to_import').get_parameter_value().string_array_value

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
            self.declare_parameter(f"{extractor}.package_name", '')
            self.declare_parameter(f"{extractor}.module_name", '')
            self.declare_parameter(f"{extractor}.class_name", '')
            self.declare_parameter(f"{extractor}.topics", [''])

            package_name = self.get_parameter(f"{extractor}.package_name").get_parameter_value().string_value
            module_name = self.get_parameter(f"{extractor}.module_name").get_parameter_value().string_value
            class_name = self.get_parameter(f"{extractor}.class_name").get_parameter_value().string_value

            topics_name = self.get_parameter(f"{extractor}.topics").get_parameter_value().string_array_value
            self.topics.extend(topics_name)
            
            try:
                module = importlib.import_module(f"{package_name}.{module_name}")
                if extractor not in self.info_extractors:
                    extractor = getattr(module, class_name)()
                    self.info_extractors[extractor.get_msg_type()] = extractor
            except Exception as e:
                self.get_logger().error(f'Error importing extractor: {e}')
                sys.exit(1)

        self.convert_bag_to_csv(bag_file_path, csv_file_path)

    def register_message_types(self, packages_to_register):
        for package in packages_to_register:
            try:
                package_share_directory = get_package_share_directory(package)
            except (PackageNotFoundError, ValueError) as exception:
                raise exception
            self.declare_parameter(package, 'msg')
            msgs_folder = self.get_parameter(package).get_parameter_value().string_value
            msgs_folder_path = Path(f"{package_share_directory}/{msgs_folder}") 

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
                print(f"{csv_file_path}exported_{topic_name}.csv")
                df.to_csv(f"{csv_file_path}exported_{topic_name}.csv", index=False)
                self.get_logger().info(f'CSV file created for {topic_name}: {csv_file_path}')

def main(args=None):
    rclpy.init(args=args)
    node = BagToCsvNode()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
