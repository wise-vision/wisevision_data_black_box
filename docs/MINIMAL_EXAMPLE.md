# Minimal example
This minimal example demonstrates how to use the wisevision_data_black_box package to mange influx data base through zenoh. It guides you through add and delete data, add influx plugin to zenoh and add storage in influx.
## Prerequisites
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) or later
- [Build and run](BUILD.md)
- [InfluxDB](INSTALL_INFLUXDB.md) (Run before run zenoh server)
- [Zenoh server](INSTALL_ZENOH.md)

## Local run
Assuming that after building the package, you can source the workspace, and run the node:
``` bash
source install/setup.bash
ros2 run wisevision_data_black_box black_box
```
## Communcation over WAN
To communicate over WAN:
- Run this severe on machine outsied local newtork [Zenoh server with ROS2 bridge](INSTALL_ZENOH_ROS2.md)

## Using the wisevision_data_black_box

The `wisevision_data_black_box` provides services to add and delete data in database, add influx pluging on zenoh server, add storage in influx through zenoh server.

Services:
- `/get_messages` (lora_msgs/srv/GetMessages): Take data from data base. It it posibble to:
    - take last N messages: in data range pass only 0
    - take last N messages from data ranger: pass date range and number of last messages
    - take all mesages fromm date range pass date range and number of mesagges with 0
- `/create_database` (wisevision_msgs/srv/CreateDataBase): add influxdb storage plugin to zenoh server.
- `/add_storage_to_database` (wisevision_msgs/srv/AddStorageToDataBase): add new database, wich follows pointed topic.
- `/add_data_to_database`  (wisevision_msgs/srv/AddDataToDataBase): add data to database on pointed topic.
- `/delete_data_from_database` (wisevision_msgs/srv/DeleteDataFromDataBase): delete data from database for pointed topic

## Examples

### Get messages from database
To take data from database, call the `/get_messages`:
``` bash
ros2 service call /get_messages lora_msgs/srv/GetMessages "{topic_name: 'devices_data', message_type: 'lora_msgs/MicroPublisher', time_start: {year: 0, month: 0, day: 0, hour: 0, minute: 0, second: 0, nanosecond: 0}, time_end: {year: 0, month: 0, day: 0, hour: 0, minute: 0, second: 0, nanosecond: 0}, number_of_msgs: 5}"
```

### Add influxdb storage plugin to zenoh server
To add influxdb to zenoh server, call the `/create_database`:
``` bash
ros2 service call /create_database wisevision_msgs/srv/CreateDataBase "{key_expr: 'demo/example/**', storage_name: 'influxdb', db_name: 'zenoh_stabilzation', create_db: true}"
```

### Add storage do database (indicating the topic to save)
To add storage in databas with information wich topic will be stored there, call the `/add_storage_to_database`:
``` bash
ros2 service call /add_storage_to_database wisevision_msgs/srv/AddStorageToDataBase "{storage_name: 'influxdb'}"
```
### Add data to data base
To add data to database, call the `/add_data_to_database`:
``` bash
ros2 service call /add_data_to_database wisevision_msgs/srv/AddDataToDataBase "{db_path: '/demo/example/test/id', query: '{\"key\": 1}'}"
```
### Delete data from database
To delete data from pointed topic, call the `/delete_data_from_database`:
``` bash
ros2 service call /delete_data_from_database wisevision_msgs/srv/DeleteDataFromDataBase "{db_path: '/demo/example/test/id'}"
```