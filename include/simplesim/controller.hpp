#pragma once

#include <SFML/Graphics.hpp>
#include <cmath>
#include <filesystem>
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "simplesim/renderable.hpp"
#include "simplesim/shapes.hpp"
#include "simplesim/utils.hpp"

using namespace std::placeholders;

struct PidCoefficients {
    float kp;
    float ki;
    float kd;
};

struct ControllerOptions {
    PidCoefficients positionControllerTune = {.kp = 2.0f, .kd = -0.05f};
    PidCoefficients velocityControllerTune = {.kp = 0.5f, .kd = -0.05f};
    sf::Vector2f initialPosition = {0.0f, 0.0f};
    float waypointEpsilon = 50.0f;
    float maxAcceleration = 10.0f;
    float lookaheadDistance = 50.0f;
    float lookaheadDistanceResolution = 5.0f;
};

class Controller : public rclcpp::Node, public Renderable {
   public:
    Controller(std::string node_name, ControllerOptions& options);

    void parameterCallback(const rclcpp::Parameter& param);

    void addWaypoint(sf::Vector2f wpt);

    void tick(sf::Time dt);

    void reset();

    std::vector<const sf::Drawable*> getDrawables() const override;

    ControllerOptions options;

    float waypointEpsilon;

    float kp_position;
    float kd_position;
    sf::Vector2f lastPositionError;

    float kp_velocity;
    float kd_velocity;
    sf::Vector2f lastVelocityError;

    float maxAcceleration;

    std::vector<sf::Vector2f> waypointList;
    CrossShapeArray waypointMarks;
    sf::VertexArray waypointPathLines;
    sf::CircleShape lookaheadCircle;
    sf::CircleShape lookaheadPoint;
    float lookaheadDistance;

    sf::Vector2f currentPosition;
    sf::Vector2f currentVelocity;
    int currentWaypointIndex = 0;
    sf::Vector2f currentSetpoint;

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