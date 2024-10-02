#pragma once

#include <vector>

#include <SFML/Graphics.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>

#include "simplesim/interfaces/resettable.hpp"

class SimNode : public Resettable, virtual public rclcpp::Node {
   public:
    SimNode(const std::string& name);
    bool reset() override;

    void publishManualWaypoint(sf::Event::MouseButtonEvent& event);

   private:
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr manualWaypointPublisher;
};