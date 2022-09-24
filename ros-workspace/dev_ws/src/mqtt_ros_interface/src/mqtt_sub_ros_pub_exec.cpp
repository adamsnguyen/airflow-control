#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <map>

#include "mqtt/async_client.h"
#include "mqtt/topic.h"
#include "mqtt_helper.h"
#include "mqtt_sub_ros_pub.h"

const std::string SERVER_ADDRESS("tcp://localhost:1883");
const std::string CLIENT_ID("mqtt_ros_interface");
const auto TIMEOUT = std::chrono::seconds(10);
mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID, 100000);
using std::placeholders::_1;

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto mqttInterfaceNode = std::make_shared<MqttInterfaceRosPub>();

  mqtt::connect_options connection_options;
  connection_options.set_clean_session(false);
  connection_options.set_automatic_reconnect(true);
  connection_options.set_max_inflight(65000);
  // Install the callback(s) before connecting.

  mqttHelper::callback cb(client, connection_options, mqttInterfaceNode);

  client.set_callback(cb);

  client.connect(connection_options, nullptr, cb);

  while (rclcpp::ok())
  {

    rclcpp::spin_some(mqttInterfaceNode);
  }

  rclcpp::shutdown();

  return 0;
}
