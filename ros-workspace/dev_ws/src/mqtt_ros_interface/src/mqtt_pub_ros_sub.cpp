#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <map>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>

#include "mqtt/async_client.h"
#include "mqtt/topic.h"

#include "mqtt_helper.h"

const std::string SERVER_ADDRESS("tcp://localhost:1883");
const std::string CLIENT_ID("mqtt_pub_ros_sub");
// const std::string TOPIC("test");
const int QOS = 10;
const auto TIMEOUT = std::chrono::seconds(10);
mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID, 100000);
using std::placeholders::_1;

using namespace std::chrono_literals;

class MqttInterface : public rclcpp::Node
{

public:
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr objectHeightSub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr motorRPMSub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr airflowSpeedSub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr systemPropertyJSONSub;

  // mqtt message pointer
  mqtt::message_ptr pubmsg;

  // Contstructor
  MqttInterface()
      : Node("mqtt_interface")
  {
    // this->publisher = this->create_publisher<std_msgs::msg::Int16>(TOPIC, QOS);
    objectHeightSub = this->create_subscription<std_msgs::msg::Float32>(
        "object_height", 10, std::bind(&MqttInterface::computerVisionCallback, this, _1));

    motorRPMSub = this->create_subscription<std_msgs::msg::Float32>(
        "motorSpeed", 10, std::bind(&MqttInterface::motorRPMCallback, this, _1));

    airflowSpeedSub = this->create_subscription<std_msgs::msg::Float32>(
        "airflowSpeed", 10, std::bind(&MqttInterface::airflowSpeedCallback, this, _1));

    systemPropertyJSONSub = this->create_subscription<std_msgs::msg::String>(
        "system_property_JSON", 10, std::bind(&MqttInterface::systemPropertyJSONCallback, this, _1));
  }

  void computerVisionCallback(const std_msgs::msg::Float32::SharedPtr msg) const
  {
    std::stringstream output_message;
    output_message << "{\"data\":\"" << msg->data << "\",\"type\":\"objectHeight\"}";
    mqtt::message_ptr pubmsg = mqtt::make_message("objectHeight", output_message.str());
    pubmsg->set_qos(1);
    client.publish(pubmsg)->wait_for(TIMEOUT);
  }

  void motorRPMCallback(const std_msgs::msg::Float32::SharedPtr msg) const
  {
    std::stringstream output_message;
    output_message << "{\"data\":\"" << msg->data << "\",\"type\":\"motorSpeed\"}";
    mqtt::message_ptr pubmsg = mqtt::make_message("motorSpeed", output_message.str());
    pubmsg->set_qos(1);
    client.publish(pubmsg)->wait_for(TIMEOUT);
  }

  void airflowSpeedCallback(const std_msgs::msg::Float32::SharedPtr msg) const
  {
    std::stringstream output_message;
    output_message << "{\"data\":\"" << msg->data << "\",\"type\":\"airflowSpeed\"}";
    mqtt::message_ptr pubmsg = mqtt::make_message("airflowSpeed", output_message.str());
    pubmsg->set_qos(1);
    client.publish(pubmsg)->wait_for(TIMEOUT);
  }

  void systemPropertyJSONCallback(const std_msgs::msg::String::SharedPtr msg) const
  {
    std::stringstream output_message;

    mqtt::message_ptr pubmsg = mqtt::make_message("system_property_JSON", msg->data);
    pubmsg->set_qos(1);
    client.publish(pubmsg)->wait_for(TIMEOUT);
  }

}

;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto mqttInterfaceNode = std::make_shared<MqttInterface>();

  mqtt::connect_options connection_options;
  connection_options.set_clean_session(false);
  connection_options.set_automatic_reconnect(true);
  connection_options.set_max_inflight(65000);
  connection_options.set_mqtt_version(3);
  // Install the callback(s) before connecting.

  mqtt::token_ptr conntok = client.connect(connection_options);

  while (rclcpp::ok())
  {

    rclcpp::spin_some(mqttInterfaceNode);
  }

  rclcpp::shutdown();

  return 0;
}
