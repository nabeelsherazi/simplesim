#pragma once

#include <vector>

#include <SFML/Graphics.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include "simplesim/interfaces/resettable.hpp"

class SimNode : public Resettable, virtual public rclcpp::Node {
   public:
    SimNode(const std::string& name);
    bool reset() override;

    void publishManualWaypoint(sf::Event::MouseButtonEvent& event);

    void publishClock(sf::Time currentTime);

   private:
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr manualWaypointPublisher;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr clockPublisher;
};