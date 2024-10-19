#include "simplesim/sim.hpp"

#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "simplesim/interfaces/resettable.hpp"

extern template class rclcpp::Publisher<geometry_msgs::msg::Vector3>;

SimNode::SimNode(const std::string& name) : Resettable(name), rclcpp::Node(name) {
    this->manualWaypointPublisher =
        this->create_publisher<geometry_msgs::msg::Vector3>("/simplesim/drone/waypoint", 10);
    this->clockPublisher = this->create_publisher<builtin_interfaces::msg::Time>("/clock", 10);
}

bool SimNode::reset() {
    RCLCPP_INFO(this->get_logger(), "Resetting simulation");
    return true;
}

void SimNode::publishManualWaypoint(sf::Event::MouseButtonEvent& event) {
    geometry_msgs::msg::Vector3 msg;
    msg.x = event.x;
    msg.y = event.y;
    this->manualWaypointPublisher->publish(msg);
}

void SimNode::publishClock(sf::Time currentTime) {
    float integral;
    float decimal = std::modf(currentTime.asSeconds(), &integral);
    builtin_interfaces::msg::Time msg;
    msg.sec = static_cast<int>(integral);
    msg.nanosec = static_cast<int>(decimal * 1E9);
    this->clockPublisher->publish(msg);
};