#pragma once

#include <vector>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <SFML/Graphics.hpp>
#include <cmath>
#include <memory>
#include "simplesim/utils.hpp"

using namespace std::placeholders;


class Controller : public rclcpp::Node {

public:
    Controller(std::string node_name, sf::Vector2f initialPosition);

    void parameterCallback(const rclcpp::Parameter& param);

    void addWaypoint(sf::Vector2f wpt);

    void tick(sf::Time dt);

    void reset();

    float waypointEpsilon;

    float kp_position;
    float kd_position;
    sf::Vector2f lastPositionError;

    float kp_velocity;
    float kd_velocity;
    sf::Vector2f lastVelocityError;

    float maxAcceleration;

    std::vector<sf::Vector2f> waypointList;

    sf::Vector2f currentPosition;
    sf::Vector2f currentVelocity;
    int currentWaypointIndex = 0;

    sf::Vector2f positionError;
    sf::Vector2f deltaError;
    sf::Vector2f velocityCommand;

private:

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr positionSubscriber;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr velocitySubscriber;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr waypointSubscriber;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr accelerationCommandPublisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocityCommandPublisher;
    std::shared_ptr<rclcpp::ParameterEventHandler> parameterSubscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> parameterCallbackHandle;
};