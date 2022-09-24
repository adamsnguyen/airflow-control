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

  // Contstructor
  MqttInterfaceRosPub::MqttInterfaceRosPub()
      : Node("mqtt_interface_ros_pub")
  {
    motorStartStopPub = this->create_publisher<std_msgs::msg::Float32>("motor", 10);
    offsetPub = this->create_publisher<std_msgs::msg::Float32>("offset", 10);
    rpmSetpointPub = this->create_publisher<std_msgs::msg::Float32>("rpmSetpoint", 10);
    rpmModPub = this->create_publisher<std_msgs::msg::Float32>("rpmMod", 10);
    heightSetpointPub = this->create_publisher<std_msgs::msg::Float32>("height_setpoint", 10);
    pidTogglePub = this->create_publisher<std_msgs::msg::Float32>("pid_toggle", 10);
    pTermPub = this->create_publisher<std_msgs::msg::Float32>("p_term", 10);
    iTermPub = this->create_publisher<std_msgs::msg::Float32>("i_term", 10);
    dTermPub = this->create_publisher<std_msgs::msg::Float32>("d_term", 10);
    clearErrorPub = this->create_publisher<std_msgs::msg::Float32>("clear_error", 10);
    baseRPMPub = this->create_publisher<std_msgs::msg::Float32>("base_rpm", 10);

    floatPublishers.push_back(pTermPub); //[0]
    TOPICS.push_back("p_term"); //[0]
    floatPublishers.push_back(iTermPub); //[1]
    TOPICS.push_back("i_term"); //[1]
    floatPublishers.push_back(dTermPub); //[2]
    TOPICS.push_back("d_term");  //[2]
    floatPublishers.push_back(motorStartStopPub); //[3]
    TOPICS.push_back("motor"); //[3]
    floatPublishers.push_back(offsetPub); //[4]
    TOPICS.push_back("offset"); //[4]
    floatPublishers.push_back(rpmSetpointPub); //[5]
    TOPICS.push_back("rpmSetpoint"); //[5]
    floatPublishers.push_back(pidTogglePub); //[6]
    TOPICS.push_back("pid_toggle"); //[6]
    floatPublishers.push_back(rpmModPub); //[7]
    TOPICS.push_back("rpmMod"); //[7]
    floatPublishers.push_back(heightSetpointPub); //[8]
    TOPICS.push_back("height_setpoint"); //[8]
    floatPublishers.push_back(clearErrorPub); //[9]
    TOPICS.push_back("clear_error"); //[9]
    floatPublishers.push_back(baseRPMPub); //[10]
    TOPICS.push_back("base_rpm"); //[10]

  }
