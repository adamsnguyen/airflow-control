#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <deque>
#include <array>

#include "rclcpp/rclcpp.hpp"
// #include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

#define TIMER_MS 80
#define DEBUG 0
#define BASE 1200.0f
#define MAX_ERROR 200
#define MAX_RPM 1500

#define P_GAIN 0.3f
#define I_GAIN 0.006f
#define D_GAIN 170.0f

/*
//Deprecated
#define DEQUE_MAX_SIZE 100
#define OVER_BASE_ERROR_COUNT_MAX 20
#define OVER_BASE_ERROR 5
#define BASE_MIN 1100
#define BASE_MAX 1200
*/


using namespace std;

float deque_sum_helper(deque<float> deque_error_container)
{
  deque<float>::iterator it;
  float sum = 0;
  for (it = deque_error_container.begin(); it != deque_error_container.end(); ++it)
  {
    sum = sum + *it;
  }
  return sum;
}

class HeightPID : public rclcpp::Node
{
public:
  // Constructor for our PID Node
  HeightPID() : Node("height_pid")
  {

    // ROS Parameters list
    this->declare_parameter("setpoint", 0.0f);
    this->declare_parameter("p", P_GAIN);
    this->declare_parameter("i",I_GAIN);
    this->declare_parameter("d", D_GAIN);
    this->declare_parameter("base", BASE);

    // Publishers
    this->new_rpm = this->create_publisher<std_msgs::msg::Float32>("rpmSetpoint", 1);
    this->system_property_JSON_publisher = this->create_publisher<std_msgs::msg::String>("system_property_JSON", 1);

    // Subscibers
    this->object_height_subscriber =
        this->create_subscription<std_msgs::msg::Float32>(
            "object_height", 10, std::bind(&HeightPID::feedback_update_callback, this, _1));
    this->p_subscriber =
        this->create_subscription<std_msgs::msg::Float32>(
            "p_term", 1, std::bind(&HeightPID::p_callback, this, _1));
    this->i_subscriber =
        this->create_subscription<std_msgs::msg::Float32>(
            "i_term", 1, std::bind(&HeightPID::i_callback, this, _1));
    this->d_subscriber =
        this->create_subscription<std_msgs::msg::Float32>(
            "d_term", 1, std::bind(&HeightPID::d_callback, this, _1));
    this->pid_toggle_subscriber =
        this->create_subscription<std_msgs::msg::Float32>(
            "pid_toggle", 1, std::bind(&HeightPID::toggle_callback, this, _1));
    this->height_setpoint_subscriber =
        this->create_subscription<std_msgs::msg::Float32>(
            "height_setpoint", 1, std::bind(&HeightPID::setpoint_callback, this, _1));
    this->clear_error_subscriber =
        this->create_subscription<std_msgs::msg::Float32>(
            "clear_error", 1, std::bind(&HeightPID::clear_error_callback, this, _1));
    this->base_rpm_subscriber =
        this->create_subscription<std_msgs::msg::Float32>(
            "base_rpm", 1, std::bind(&HeightPID::base_rpm_callback, this, _1)); //depcrecated

    // timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(TIMER_MS),
    //     std::bind(&HeightPID::pid_loop_callback, this));

    system_property_JSON_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(3000),
        std::bind(&HeightPID::system_property_JSON_timer_callback, this));

    this->p_term = this->get_parameter("p").as_double();
    this->i_term = this->get_parameter("i").as_double();
    this->d_term = this->get_parameter("d").as_double();
    this->setpoint = this->get_parameter("setpoint").as_double();
    this->base = this->get_parameter("base").as_double();
  }

  // Node Data Member
  float p_term;
  float i_term;
  float d_term;
  bool controller_active = false;
  float setpoint;
  int base;
  int feedback; 
  deque<float> error_container; //depcrecated
  array<float, 2> derivative_error_container{0.0f, 0.0f};
  float integral_error_sum;
  //int over_base_error_pos_count = 0; //depcrecated
  //int over_base_error_neg_count = 0; //depcrecated
  //int overbase_error = OVER_BASE_ERROR; //depcrecated

private:
  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr p_subscriber;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr i_subscriber;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr d_subscriber;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pid_toggle_subscriber;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr height_setpoint_subscriber;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr object_height_subscriber;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr clear_error_subscriber;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr base_rpm_subscriber;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr new_rpm;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_property_JSON_publisher;

  // Timer for publisher callback
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr system_property_JSON_timer_;

  void system_property_JSON_timer_callback()
  {
    auto message = std_msgs::msg::String();
    std::stringstream output_message;

    output_message 
    << "{\"p_term\":"<<this->p_term<<", "
    << "\"i_term\":"<<this->i_term<<", "
    << "\"d_term\":"<<this->d_term<<", "
    << "\"base_rpm\":"<<this->base<<", "
    << "\"height_setpoint\":"<<this->setpoint<<", "
    << "\"pid_toggle\":"<<this->controller_active
    <<"}";

    message.data = output_message.str();

    this->system_property_JSON_publisher->publish(message);

    if (DEBUG)
    {
      std::stringstream ros_log_output_message;
      ros_log_output_message << "System characteristics: " << output_message.str();
      RCLCPP_INFO(this->get_logger(), ros_log_output_message.str());

    }
  
  }

  void p_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    this->set_parameter(rclcpp::Parameter("p", msg->data));
    this->p_term = this->get_parameter("p").as_double();
    if (DEBUG)
    {
      std::stringstream output_message;
      output_message << "P Callback: " << msg->data;
      RCLCPP_INFO(this->get_logger(), output_message.str());
      output_message.str(std::string());
      output_message << "P Data member: " << this->p_term;
      RCLCPP_INFO(this->get_logger(), output_message.str());
    }
  }

  void i_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    this->set_parameter(rclcpp::Parameter("i", msg->data));
    this->i_term = this->get_parameter("i").as_double();
    if (DEBUG)
    {
      std::stringstream output_message;
      output_message << "I Callback: " << msg->data;
      RCLCPP_INFO(this->get_logger(), output_message.str());
      output_message.str(std::string());
      output_message << "I Data member: " << this->i_term;
      RCLCPP_INFO(this->get_logger(), output_message.str());
    }
  }

  void d_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    this->set_parameter(rclcpp::Parameter("d", msg->data));
    this->d_term = this->get_parameter("d").as_double();
    if (DEBUG)
    {
      std::stringstream output_message;
      output_message << "D Callback: " << msg->data;
      RCLCPP_INFO(this->get_logger(), output_message.str());
      output_message.str(std::string());
      output_message << "D Data member: " << this->d_term;
      RCLCPP_INFO(this->get_logger(), output_message.str());
    }
  }

  void setpoint_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    this->set_parameter(rclcpp::Parameter("setpoint", msg->data));
    this->setpoint = this->get_parameter("setpoint").as_double();
    if (DEBUG)
    {
      std::stringstream output_message;
      output_message << "Height Setpoint Callback: " << msg->data;
      RCLCPP_INFO(this->get_logger(), output_message.str());
    }
  }

  void base_rpm_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    this->set_parameter(rclcpp::Parameter("base", msg->data));
    this->base = this->get_parameter("base").as_double();
    if (DEBUG)
    {
      std::stringstream output_message;
      output_message << "Base RPM Callback: " << msg->data;
      RCLCPP_INFO(this->get_logger(), output_message.str());
    }
  }

  void clear_error_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    this->integral_error_sum = 0;
    // this->error_container.clear();
    this->derivative_error_container[0] = 0;
    this->derivative_error_container[1] = 0;
    // this->over_base_error_pos_count = 0;
    // this->over_base_error_neg_count = 0;

    if (DEBUG)
    {
      std::stringstream output_message;
      output_message << "Clear error Callback called";
      RCLCPP_INFO(this->get_logger(), output_message.str());
    }
  }

  void toggle_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    this->error_container.clear();
    int result = msg->data;

    if (result == 1)
    {
      this->controller_active = true;
    }
    else
    {
      this->controller_active = false;
    }
    if (DEBUG)
    {
      std::stringstream output_message;
      output_message << "PID toggle Callback: " << msg->data;
      RCLCPP_INFO(this->get_logger(), output_message.str());
      output_message.str(std::string());
      output_message << "PID Toggle Data member: " << this->controller_active;
      RCLCPP_INFO(this->get_logger(), output_message.str());
    }
  }

  void pid_loop_callback()
  {

    if (!(this->controller_active))
    {
      return;
    }

    float p_component;
    float i_component;
    float d_component;
    int new_rpm_setpoint;
    auto message = std_msgs::msg::Float32();
    float derivative;
    int y = this->feedback;
    float r = this->setpoint;
    float e = r - y;
    float integrator_error = e;

    //This is deprecated
    /*
    // Self correcting to find base effort (effort to maintain height)
    if (abs(e) > this->overbase_error)
    {
      if (e > 0)
      {
        this->over_base_error_pos_count++;

        if( this->over_base_error_neg_count > 0)
        {
           this->over_base_error_neg_count--;
        }
      }
      else
      {
        this->over_base_error_neg_count++;

        if( this->over_base_error_pos_count > 0)
        {
           this->over_base_error_pos_count--;
        }
      }

      if (this->over_base_error_pos_count > OVER_BASE_ERROR_COUNT_MAX)
      {
        this->over_base_error_pos_count = 0;

        if (this->base <= BASE_MAX)
        {
          this->base++;
        }
      }

      else if (this->over_base_error_neg_count > OVER_BASE_ERROR_COUNT_MAX)
      {
        this->over_base_error_neg_count = 0;
        
        if (this->base >= BASE_MIN)
        {
          this->base--;
        }
      }
    }

    // Add the error term to the integral container
    if (this->error_container.size() >= 1000)
    {
      this->error_container.pop_front();
    }
    else
    {
      if (abs(e) >= MAX_ERROR)
      {
        if (e > 0)
        {
          this->error_container.push_back(MAX_ERROR);
        }
        else
        {
          this->error_container.push_back(MAX_ERROR * -1);
        }
      }
      else
      {
        this->error_container.push_back(e);
      }
    }
    */

   //Anti-windup scheme: We set a maximum allowed error

   if (abs(integrator_error) >= MAX_ERROR)
   {
      if (integrator_error > 0)
      {
        integrator_error = MAX_ERROR;
      }
      else
      {
        integrator_error = MAX_ERROR * -1;
      }
   }

   // Add the integrator error

    this->integral_error_sum = this->integral_error_sum + integrator_error;

    // Add the error term to the derivative container
    this->derivative_error_container[1] = this->derivative_error_container[0];
    this->derivative_error_container[0] = e;

    // Calculate each PID component

    // P
    p_component = e * this->p_term;

    // I
    //integral_sum = deque_sum_helper(this->error_container); //Deprecated
    i_component = this->i_term * this->integral_error_sum;

    // D
    derivative = ((this->derivative_error_container[0] - this->derivative_error_container[1]) / TIMER_MS);
    d_component = this->d_term * (derivative);

    // New Setpoint
    new_rpm_setpoint = this->base + p_component + i_component + d_component;

    // New setpoint cannot be greater than max RPM
    new_rpm_setpoint = new_rpm_setpoint > MAX_RPM ? MAX_RPM : new_rpm_setpoint;
    
    message.data = new_rpm_setpoint;

    // Publish

    if (this->controller_active)
    {
      this->new_rpm->publish(message);
    }

    if (DEBUG)
    {
      std::stringstream output_message;
      output_message << "Calculated new RPM: " << new_rpm_setpoint << "New RPM: " << message.data << " Error: " << e << " Integral sum error: "
                     << this->integral_error_sum << " i componenent: " << i_component << " error derivative: " << derivative << 
                     " Derivatve component: " << d_component << " Last error: " << this->derivative_error_container[1]
                     << " Current error: " << this->derivative_error_container[0];
      RCLCPP_INFO(this->get_logger(), output_message.str());
    }
  }

  void feedback_update_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {

    this->feedback = msg->data;

    if (this->controller_active)
    {
      this->pid_loop_callback();
    }

    if (DEBUG)
    {
      std::stringstream output_message;
      output_message << "feedback: " << msg->data;
      RCLCPP_INFO(this->get_logger(), output_message.str());
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HeightPID>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
