#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>

template class rclcpp::Publisher<geometry_msgs::msg::Vector3>;
template class rclcpp::Subscription<geometry_msgs::msg::Vector3>;
template class rclcpp::Subscription<nav_msgs::msg::Odometry>;