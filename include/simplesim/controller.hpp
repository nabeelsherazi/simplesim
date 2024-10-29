#pragma once

#include <cmath>
#include <filesystem>
#include <memory>
#include <vector>

#include <fmt/core.h>
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
    PidCoefficients positionControllerTune = {.kp = 1.5f, .kd = -0.025f};

    /// @brief PID coefficients for the cascaded velocity controller
    PidCoefficients velocityControllerTune = {.kp = 1.0f, .kd = -0.005f};

    /// @brief Initial position reading for the controller
    sf::Vector2f initialPosition = {0.0f, 0.0f};

    /// @brief How close the drone's position needs to be to a waypoint before the control considers it reached
    float waypointEpsilon = 20.0f;

    /// @brief Maximum acceleration command the controller will issue
    float maxAcceleration = 500.0f;

    /// @brief Whether or not to use pure pursuit
    bool usePurePursuit = true;

    /// @brief Lookahead distance for pure pursuit
    float lookaheadDistance = 100.0f;
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
    long int tickCount = 0;

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr positionSubscriber;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr velocitySubscriber;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr waypointSubscriber;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr accelerationCommandPublisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocityCommandPublisher;
    std::shared_ptr<rclcpp::ParameterEventHandler> parameterSubscriber;
    std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> parameterCallbackHandles;

    friend class ControllerDebugInfo;
};

class ControllerDebugInfo {
   private:
    std::weak_ptr<Controller> controller;

   public:
    explicit ControllerDebugInfo(std::shared_ptr<Controller> controller) : controller(controller) {};

    std::vector<std::function<std::string(void)>> allDebugInfo() {
        std::vector<std::function<std::string(void)>> allDebugInfo;
        allDebugInfo.push_back(positionErrorDebugInfo);
        allDebugInfo.push_back(positionPidDebugInfo);
        allDebugInfo.push_back(velocityPidDebugInfo);
        return allDebugInfo;
    }

    std::function<std::string(void)> currentWaypointDebugInfo = [this]() {
        if (auto controller = this->controller.lock()) {
            // clang-format off
            return fmt::format(
                "current waypoint: {} @ ({}, {})",
                controller->currentWaypointIndex,
                controller->currentSetpoint.x,
                controller->currentSetpoint.y
            );
            // clang-format on
        }
        return std::string();
    };

    std::function<std::string(void)> positionErrorDebugInfo = [this]() {
        if (auto controller = this->controller.lock()) {
            // clang-format off
            return fmt::format(
                "x error: {:.2f}, y error: {:.2f}, norm: {:.2f}",
                controller->positionError.x,
                controller->positionError.y,
                simplesim::norm(controller->positionError)
            );
            // clang-format on
        }
        return std::string();
    };

    std::function<std::string(void)> positionPidDebugInfo = [this]() {
        if (auto controller = this->controller.lock()) {
            // clang-format off
            return fmt::format(
                "kp_p:={:.2f} * ({:.2f},{:.2f}) + kd_p:{:.2f} * ({:.2f},{:.2f}) = ({:.2f}, {:.2f})",
                controller->kp_position,
                controller->positionError.x,
                controller->positionError.y,
                controller->kd_position,
                controller->deltaPositionError.x,
                controller->deltaPositionError.y,
                controller->velocityCommand.x,
                controller->velocityCommand.y
            );
            // clang-format on
        }
        return std::string();
    };

    std::function<std::string(void)> velocityPidDebugInfo = [this]() {
        if (auto controller = this->controller.lock()) {
            // clang-format off
            return fmt::format(
                "kp_v:={:.2f} * ({:.2f},{:.2f}) + kd_v:{:.2f} * ({:.2f},{:.2f}) = ({:.2f}, {:.2f})",
                controller->kp_velocity,
                controller->velocityError.x,
                controller->velocityError.y,
                controller->kd_velocity,
                controller->deltaVelocityError.x,
                controller->deltaVelocityError.y,
                controller->accelerationCommand.x,
                controller->accelerationCommand.y
            );
            // clang-format on
        }
        return std::string();
    };
};