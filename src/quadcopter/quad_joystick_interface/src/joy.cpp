/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Licensed under the Apache License, Version 2.0
 *
 * ROS 2 Jazzy rewrite — publishes geometry_msgs/Twist
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

class QuadJoystickInterface : public rclcpp::Node
{
public:
  QuadJoystickInterface() : Node("quad_joystick_interface")
  {
    this->declare_parameter("axis_roll", 3);
    this->declare_parameter("axis_pitch", 4);
    this->declare_parameter("axis_thrust", 1);
    this->declare_parameter("axis_yaw", 0);

    this->declare_parameter("axis_direction_roll", -1);
    this->declare_parameter("axis_direction_pitch", 1);
    this->declare_parameter("axis_direction_thrust", 1);
    this->declare_parameter("axis_direction_yaw", -1);

    this->declare_parameter("max_roll", 25.0 * M_PI / 180.0);
    this->declare_parameter("max_pitch", 25.0 * M_PI / 180.0);
    this->declare_parameter("max_yaw_rate", 50.0 * M_PI / 180.0);
    this->declare_parameter("max_thrust", 10.0);
    this->declare_parameter("thrust_offset", 2.75);

    this->declare_parameter("cmd_vel_topic", "/model/X3/cmd_vel");

    axes_.roll = this->get_parameter("axis_roll").as_int();
    axes_.pitch = this->get_parameter("axis_pitch").as_int();
    axes_.thrust = this->get_parameter("axis_thrust").as_int();
    axes_.yaw = this->get_parameter("axis_yaw").as_int();

    axes_.roll_direction = this->get_parameter("axis_direction_roll").as_int();
    axes_.pitch_direction = this->get_parameter("axis_direction_pitch").as_int();
    axes_.thrust_direction = this->get_parameter("axis_direction_thrust").as_int();
    axes_.yaw_direction = this->get_parameter("axis_direction_yaw").as_int();

    max_.roll = this->get_parameter("max_roll").as_double();
    max_.pitch = this->get_parameter("max_pitch").as_double();
    max_.rate_yaw = this->get_parameter("max_yaw_rate").as_double();
    max_.thrust = this->get_parameter("max_thrust").as_double();
    max_.thrust_offset = this->get_parameter("thrust_offset").as_double();

    std::string cmd_topic = this->get_parameter("cmd_vel_topic").as_string();

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&QuadJoystickInterface::JoyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "QuadJoystickInterface started on %s", cmd_topic.c_str());
  }

private:
  struct Axes {
    int roll, pitch, thrust, yaw;
    int roll_direction, pitch_direction, thrust_direction, yaw_direction;
  } axes_;

  struct Max {
    double roll, pitch, rate_yaw, thrust, thrust_offset;
  } max_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->axes.size() < 5) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Joy message has fewer axes than expected");
      return;
    }

    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = msg->axes[axes_.pitch] * max_.pitch * axes_.pitch_direction * 0.5;
    twist.linear.y = msg->axes[axes_.roll] * max_.roll * axes_.roll_direction * 0.5;
    twist.linear.z = max_.thrust_offset + (msg->axes[axes_.thrust] + 1.0) * max_.thrust / 2.0 * axes_.thrust_direction;
    twist.angular.z = msg->axes[axes_.yaw] * max_.rate_yaw * axes_.yaw_direction;

    cmd_pub_->publish(twist);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadJoystickInterface>());
  rclcpp::shutdown();
  return 0;
}
