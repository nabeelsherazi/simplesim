#include "simplesim/sim.hpp"

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "simplesim/interfaces/resettable.hpp"

SimNode::SimNode(const std::string& name) : Resettable(name), rclcpp::Node(name) {
    this->manualWaypointPublisher =
        this->create_publisher<geometry_msgs::msg::Vector3>("/simplesim/drone/waypoint", 10);
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