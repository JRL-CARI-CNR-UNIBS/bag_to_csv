# Bag to CSV

This repository contains a ROS 2 node for extracting interesting topics from bag files and converting them to CSV files with the desired information. Here the project [WEBPAGE](https://jrl-cari-cnr-unibs.github.io/bag_to_csv/)

## Main Features

- Select specific topics to extract.
- Define and format the desired information from the topics.
- Generate CSV files for each selected topic with the extracted information.

## Configuration

To configure the node to select topics and define the desired information, follow these steps:

### 1. Selecting Topics

Specify the topics you want to extract in the configuration file. Here's an example configuration:

```yaml
bag_to_csv_node:
  ros__parameters:
    bag_file_path: "/path/to/your/bagfile"
    output_csv_path: "/path/to/output/csv/"
    packages_to_register:
      - your_custom_msgs_package
    extractors_to_import:
      - example_extractor # joint_state_extractor, yolo_msg_extractor
    example_extractor:
      package_name: "example_package_name"
      module_name: "example_module_name"
      class_name: "example_class_name"
      topics: ["interesting topics following this extractor"]
```

- `bag_file_path`: The path to the bag file you want to convert. Can be set in launch args
- `output_csv_path`: The path where the CSV file(s) will be saved. Can be set in launch args.
- `packages_to_register`: List of ROS 2 packages containing custom message types to register.
- `extractors_to_import`: List of extractor modules to use for custom data extraction.

### 2. Writing an Extractor

An extractor defines how to extract and format the desired information from the messages. Here is a minimal example of an extractor:

```python
from bag_to_csv.info_extractor import InfoExtractor

class CustomInfoExtractor(InfoExtractor):
    def __init__(self):
        super().__init__("your_msgs/msg/YourMessageType")

    def extract_info_from_msg(self, msg, msg_type):
        self._check_msg_type(msg_type)
        return {
            'field1': msg.field1,
            'field2': msg.field2,
            # Add more fields as needed
        }
```

In this example:

- `your_msgs/msg/YourMessageType` is the type of message you are extracting information from.
- The `extract_info_from_msg` method extracts the desired fields from the message and returns them in a dictionary format.

### Running the Node with a Launch File

You can run the `BagToCsvNode` using a launch file, which allows you to easily set parameters and start the node. Here's an example of how to run the node with a launch file:

```sh
ros2 launch bag_to_csv bag_to_csv.launch.py bag_file_path:=/path/to/your/bagfile output_csv_path:=/path/to/output/csv
```
- `bag_file_path` is the path to the bag file you want to convert.
- `output_csv_path` is the path where the CSV file(s) will be saved.
