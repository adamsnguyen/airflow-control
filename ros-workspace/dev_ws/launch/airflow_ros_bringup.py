from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    computer_vision_node = Node(
        package="computer_vision",
        executable="computer_vision",
    )
    modbus_node = Node(
        package="modbus_rtu",
        executable="modbus_rtu"
    )
    mqtt_sub_ros_pub_node = Node(
        package="mqtt_ros_interface",
        executable="mqtt_sub_ros_pub_exec",
    )
    mqtt_pub_ros_sub_node = Node(
        package="mqtt_ros_interface",
        executable="mqtt_pub_ros_sub"
    )

    height_pid = Node(
        package="height_pid",
        executable="height_pid"
    )


    ld.add_action(computer_vision_node)
    ld.add_action(modbus_node)
    ld.add_action(mqtt_sub_ros_pub_node)
    ld.add_action(mqtt_pub_ros_sub_node)
    ld.add_action(height_pid)
    return ld
