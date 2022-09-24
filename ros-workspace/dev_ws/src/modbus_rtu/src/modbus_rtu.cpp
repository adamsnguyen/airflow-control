#include <cstdio>
#include <iostream>
#include <sstream>
#include <random>
#include <stdlib.h>

#include <modbus.h>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>

#define MAX_RPM 1800.00      // Set on VFD
#define MAX_FREQ 600.00      // Hz * 10
#define DEBUG 1           // Turn on for ROS logging
#define CALLBACK_FREQ 150 // ms

using std::placeholders::_1;
using namespace std;
int rpm_state = 0;

/*
Libmodbus Register Note:

Note that in the libmodbus library the register address is offset by one
with respect to a container's index number, eg. register 1 will be located at index 0.
*/

// Helper functions

void modbus_write_helper(int slaveNumber, modbus_t *ctx, int addr, int nb, uint16_t *dest, string slaveDescr, string registerDesc)
{
  modbus_set_slave(ctx, slaveNumber);
  int num;
  // std::stringstream output_message;

  num = modbus_write_registers(ctx, addr, nb, dest);

  if (num != nb)
  {
    // output_message.str(std::string()); // Clears buffer
    cout << "Error: Failed writing " << dest[0] << "to " << slaveDescr << "of registers" << registerDesc;
    // number of read registers is not the one expected
    fprintf(stderr, "Error: %s\n", modbus_strerror(errno));
  }
}

void modbus_read_helper(int slaveNumber, modbus_t *ctx, int addr, int nb, uint16_t *dest, string slaveDescr, string registerDesc)
{
  modbus_set_slave(ctx, slaveNumber);
  int num;
  // std::stringstream output_message;

  num = modbus_read_registers(ctx, addr, nb, dest);

  if (num != nb)
  {
    // output_message.str(std::string()); // Clears buffer
    cout << "Error: Failed reading " << dest[0] << "to " << slaveDescr << "of registers" << registerDesc;
    // number of read registers is not the one expected
    fprintf(stderr, "Error: %s\n", modbus_strerror(errno));
  }
}

void publish_msg_helper(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher, int messageData)
{
  auto message = std_msgs::msg::Float32();
  message.data = messageData;

  publisher->publish(message);
}

class Modbus_RTU : public rclcpp::Node
{
public:
  Modbus_RTU() : Node("modbus_rtu")
  {

    // Initializing Publishers
    motorSpeedPublisher = this->create_publisher<std_msgs::msg::Float32>("motorSpeed", 10);
    airflowSpeedPublisher = this->create_publisher<std_msgs::msg::Float32>("airflowSpeed", 10);

    motorStateSubscriber = this->create_subscription<std_msgs::msg::Float32>(
        "motor", 10, std::bind(&Modbus_RTU::motorStateCallback, this, _1));
    rpmSetpointSubscriber = this->create_subscription<std_msgs::msg::Float32>(
        "rpmSetpoint", 10, std::bind(&Modbus_RTU::rpmSetpointCallback, this, _1));
    rpmModSubscriber = this->create_subscription<std_msgs::msg::Float32>(
        "rpmMod", 10, std::bind(&Modbus_RTU::rpmModCallback, this, _1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(CALLBACK_FREQ),
        std::bind(&Modbus_RTU::timerCallback, this));

    // Creating Modbus RTU Connection Context
    char *pTmp;
    pTmp = std::getenv("RS485");
    

    this->declare_parameter<std::string>("usb_modbus_location", pTmp);

    this->get_parameter("usb_modbus_location", this->modbus_usb_location);

    const char *modbus_usb_location_const_char = this->modbus_usb_location.c_str();

    // RCLCPP_INFO(this->get_logger(), modbus_usb_location_const_char);

    this->ctx = modbus_new_rtu(modbus_usb_location_const_char, 9600, 'N', 8, 1);

    // Making sure the context was successfully created.

    if (!this->ctx)
    {
      // RCLCPP_INFO(this->get_logger(), "Failed to create the context");
      fprintf(stderr, "Failed to create the context: %s\n", modbus_strerror(errno));
      exit(1);
    }

    if (modbus_connect(this->ctx) == -1)
    {
      // RCLCPP_INFO(this->get_logger(), "Unable to connect");
      fprintf(stderr, "Unable to connect: %s\n", modbus_strerror(errno));
      modbus_free(this->ctx);
      exit(1);
    }
  }

private:
  std::string modbus_usb_location;
  modbus_t *ctx;

  // Declaration of the publishers

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motorSpeedPublisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr airflowSpeedPublisher;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr motorStateSubscriber;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rpmSetpointSubscriber;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rpmModSubscriber;

  void rpmSetpointCallback(const std_msgs::msg::Float32::SharedPtr msg) const
  {
    std::stringstream output_message;
    uint16_t reg[1];
    float rpm_float = ((msg->data) * MAX_FREQ) / MAX_RPM;
    int rpm = rpm_float;
    reg[0] = rpm;
    rpm_state = rpm;
    // output_message << rpm;

    // reg[0] = std::strtoul(output_message.str().c_str(), NULL, 10);

    output_message.str(std::string());
    output_message << msg->data << rpm_float << rpm << reg[0];
    // RCLCPP_INFO(this->get_logger(), output_message.str());

    modbus_write_helper(1, this->ctx, 1, 1, reg, "VFD", "RPM Setpoint");
  }

  void rpmModCallback(const std_msgs::msg::Float32::SharedPtr msg) const
  {
    uint16_t reg[1];
    // int rpm_mod_int = msg->data;
    float rpm_mod_float = msg->data * (MAX_FREQ / MAX_RPM);
    int rpm_mod = rpm_mod_float;
    rpm_state = rpm_state + rpm_mod;
    reg[0] = rpm_state;

    modbus_write_helper(1, this->ctx, 1, 1, reg, "VFD", "RPM Setpoint");

    if (DEBUG)
    {
      std::stringstream output_message;
      output_message << msg->data << " " << rpm_mod_float << " " << rpm_mod << " " << reg[0] << rpm_state;
      RCLCPP_INFO(this->get_logger(), output_message.str());
    }

    
  }

  void motorStateCallback(const std_msgs::msg::Float32::SharedPtr msg) const
  {
    // RCLCPP_INFO(this->get_logger(), "Subscriber callback called");

    modbus_set_slave(this->ctx, 1);

    uint16_t reg_1[1];
    uint16_t run = 1;
    uint16_t stop = 0;
    int num;
    std::stringstream output_message;

    if (msg->data > 0)
    {
      reg_1[0] = run;

      num = modbus_write_registers(this->ctx, 0, 1, reg_1);
    }
    else
    {
      reg_1[0] = stop;

      num = modbus_write_registers(this->ctx, 0, 1, reg_1);
    }

    if (num != 1)
    {
      RCLCPP_INFO(this->get_logger(), "Failed to get vfd");
      // number of read registers is not the one expected
      fprintf(stderr, "Error: %s\n", modbus_strerror(errno));
    }
    else
    {
      if (DEBUG == 1)
      {
        output_message.str(std::string());
        output_message << "wrote: " << reg_1[0];
        RCLCPP_INFO(this->get_logger(), output_message.str());
      }
    }
  }

  void timerCallback()
  {
    // Creating a reading register array of size 1
    uint16_t reg[1];
    std::stringstream output_message;

    //*******************************************************************************************
    // Process VFD
    //*******************************************************************************************
    // Motor RPM: Read MODBUS and publish ROS
    modbus_read_helper(1, this->ctx, 6, 1, reg, "VFD RPM", "RPM");
    int hz = reg[0];
    float rpm_float = hz * MAX_RPM / MAX_FREQ;
    int rpm = rpm_float;

    if (DEBUG == 1)
    {
      output_message.str(std::string());
      output_message << "RPM OUT wrote: " << reg[0] << " " << rpm_float << " " << rpm;
      RCLCPP_INFO(this->get_logger(), output_message.str());
    }

    publish_msg_helper(this->motorSpeedPublisher, rpm_float);

    //*******************************************************************************************
    // Airflow
    //*******************************************************************************************
    // Airflor speed: Read MODBUS and publish ROS

    modbus_read_helper(1, this->ctx, 19, 1, reg, "Airspeed", "RPM Out");
    float fps_1000 = reg[0]; // fps from 0 to 328 ft/s expressed from 0 to 1000, also approximately equivalent to cfm
    float fps = (fps_1000/1000) * 328;

    if (DEBUG == 1)
    {
      output_message.str(std::string());
      output_message << "CFM OUT wrote: " << fps_1000;
      RCLCPP_INFO(this->get_logger(), output_message.str());
    }

    publish_msg_helper(this->airflowSpeedPublisher, fps_1000);
  }
}

;
int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  auto node = std::make_shared<Modbus_RTU>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
