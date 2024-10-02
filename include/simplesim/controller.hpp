#pragma once

#include <cmath>
#include <filesystem>
#include <memory>
#include <vector>

#include <SFML/Graphics.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>

#include "simplesim/interfaces/renderable.hpp"
#include "simplesim/interfaces/resettable.hpp"
#include "simplesim/shapes.hpp"
#include "simplesim/utils.hpp"

using namespace std::placeholders;

struct PidCoefficients {
    float kp;
    float ki;
    float kd;
};

struct ControllerOptions {
    /// @brief PID coefficients for the position controller
    PidCoefficients positionControllerTune = {.kp = 3.0f, .kd = -0.30f};

    /// @brief PID coefficients for the cascaded velocity controller
    PidCoefficients velocityControllerTune = {.kp = 2.0f, .kd = -0.04f};

    /// @brief Initial position reading for the controller
    sf::Vector2f initialPosition = {0.0f, 0.0f};

    /// @brief How close the drone's position needs to be to a waypoint before the control considers it reached
    float waypointEpsilon = 20.0f;

    /// @brief Maximum acceleration command the controller will issue
    float maxAcceleration = 10.0f;

    /// @brief Whether or not to use pure pursuit
    bool usePurePursuit = false;

    /// @brief Lookahead distance for pure pursuit
    float lookaheadDistance = 50.0f;

    /// @brief Resolution of lookahead search
    float lookaheadDistanceResolution = 5.0f;
};

class Controller : public Renderable, public Resettable {
   public:
    Controller(const std::string& node_name, ControllerOptions& options);

    void parameterCallback(const rclcpp::Parameter& param);

    void addWaypoint(sf::Vector2f wpt);

    void tick(sf::Time dt);

    bool reset() override;

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
    sf::Vector2f deltaPositionError;
    sf::Vector2f velocityError;
    sf::Vector2f deltaVelocityError;
    sf::Vector2f velocityCommand;
    sf::Vector2f accelerationCommand;

   private:
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr positionSubscriber;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr velocitySubscriber;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr waypointSubscriber;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr accelerationCommandPublisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocityCommandPublisher;
    std::shared_ptr<rclcpp::ParameterEventHandler> parameterSubscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> parameterCallbackHandle;
};