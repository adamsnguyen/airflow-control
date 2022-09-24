
#ifndef MQTT_SUB_ROS_PUB_H // include guard
#define MQTT_SUB_ROS_PUB_H

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float32.hpp>


class MqttInterfaceRosPub : public rclcpp::Node
{
public:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motorStartStopPub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr offsetPub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rpmSetpointPub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pTermPub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr iTermPub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dTermPub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pidTogglePub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rpmModPub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heightSetpointPub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr clearErrorPub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr baseRPMPub;

  std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> floatPublishers;
  std::vector<std::string> TOPICS;

  // mqtt message pointer
  mqtt::message_ptr pubmsg;

  // Contstructor
  MqttInterfaceRosPub();

};
#endif 
