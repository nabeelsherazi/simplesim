#pragma once

#include <filesystem>
#include <functional>
#include <random>

#include <fmt/core.h>
#include <tf2_ros/transform_broadcaster.h>
#include <SFML/Graphics.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "simplesim/interfaces/renderable.hpp"
#include "simplesim/interfaces/resettable.hpp"
#include "simplesim/managed_sprite.hpp"
#include "simplesim/utils.hpp"
#include "simplesim/visuals.hpp"

using namespace std::placeholders;

struct DroneOptions {
    /// @brief Enum for the simulated control mode (whether to accept a velocity
    /// or acceleration command)
    enum class ControlMode { Velocity, Acceleration };

    /// @brief Simulated control mode (whether to accept a velocity
    /// or acceleration command)
    DroneOptions::ControlMode controlMode = DroneOptions::ControlMode::Acceleration;

    /// @brief Initial position for the drone
    sf::Vector2f initialPosition = {0.0F, 0.0F};

    /// @brief Magnitude of the maximum "jerk" to the wind at any iteration. Current wind speed is calculated like
    /// currentWind += rand(-intensity, intensity)
    float windIntensity = 10.0F;

    /// @brief Maximum acceleration command the drone will accept
    float maxAcceleration = 500.0F;

    /// @brief Maximum velocity command the drone will accept
    float maxVelocity = 500.0F;

    /// @brief Drag constant in the linear regime (friction drag)
    float linearDragConstant = 0.000005F;

    /// @brief Drag constant in the quadratic regime (pressure drag)
    float quadraticDragConstant = 0.000005F;

    /// @brief Speed threshold at which to switch from linear to quadratic drag
    float quadraticDragThreshold = 100.0F;
};

/// @brief Simulates vehicle dynamics given control and environment forces
class Drone : public Renderable, public Resettable {
   public:
    /// @brief Simulates vehicle dynamics given control and environment forces
    /// @param node_name The name of the node to launch under
    /// @param options Simulation options struct
    Drone(std::string node_name, DroneOptions& options);

    /// @brief Destructor
    virtual ~Drone();

    /// @brief Takes a sprite by move to use as the visual representation of this drone
    /// @param sprite sprite instance to use
    void setSprite(ManagedSprite&& sprite);

    /// @brief Callback that sets current commanded acceleration (acceleration control mode)
    /// @param msg message
    void setAccelerationCommand(const geometry_msgs::msg::Vector3::SharedPtr msg);

    /// @brief Callback that sets current commanded velocity (velocity control mode)
    /// @param msg message
    void setVelocityCommand(const geometry_msgs::msg::Vector3::SharedPtr msg);

    /// @brief Advance the simulation
    /// @param dt the amount of time that has passed since the last call
    void tick(const sf::Time dt);

    /// @brief Publish simulation state
    void publish();

    /// @brief Reset the simulation state to initial conditions
    bool reset() override;

    std::vector<const sf::Drawable*> getDrawables() const override;

    sf::Vector2f currentWind;
    sf::Vector2f currentDrag;
    sf::Vector2f currentVelocity;
    sf::Vector2f currentPosition;

    DroneOptions options;

   private:
    ManagedSprite sprite;

    DroneOptions::ControlMode controlMode;

    sf::Vector2f accelerationCommand;

    std::random_device randomDevice;
    std::mt19937 randomGenerator;
    std::uniform_real_distribution<> dist;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> transformBroadcaster;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocity_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr accelerationCommandSubscriber;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr velocityCommandSubscriber;

    friend class DroneDebugInfo;
};

class DroneDebugInfo {
   public:
    explicit DroneDebugInfo(std::shared_ptr<Drone> drone) : drone(drone) {};

    std::vector<std::function<std::string(void)>> allDebugInfo() {
        std::vector<std::function<std::string(void)>> allDebugInfo;
        allDebugInfo.push_back(positionDebugInfo);
        allDebugInfo.push_back(velocityDebugInfo);
        allDebugInfo.push_back(windDebugInfo);
        allDebugInfo.push_back(dragDebugInfo);
        return allDebugInfo;
    }

    std::function<std::string(void)> positionDebugInfo = [this]() {
        if (auto drone = this->drone.lock()) {
            // clang-format off
            return fmt::format(
                "current position: ({:.2f}, {:.2f})",
                drone->currentPosition.x,
                drone->currentPosition.y
            );
            // clang-format on
        }
        return std::string();
    };

    std::function<std::string(void)> velocityDebugInfo = [this]() {
        if (auto drone = this->drone.lock()) {
            // clang-format off
            return fmt::format(
                "current velocity {:.0f} px/s: ({:.2f}, {:.2f})",
                simplesim::norm(drone->currentVelocity),
                drone->currentVelocity.x,
                drone->currentVelocity.y
            );
            // clang-format on
        }
        return std::string();
    };

    std::function<std::string(void)> windDebugInfo = [this]() {
        if (auto drone = this->drone.lock()) {
            // clang-format off
            return fmt::format(
                "wind: ({:.2f}, {:.2f})",
                drone->currentWind.x,
                drone->currentWind.y
            );
            // clang-format on
        }
        return std::string();
    };

    std::function<std::string(void)> dragDebugInfo = [this]() {
        if (auto drone = this->drone.lock()) {
            // clang-format off
            return fmt::format(
                "drag: ({:.2f}, {:.2f})",
                drone->currentDrag.x,
                drone->currentDrag.y
            );
            // clang-format on
        }
        return std::string();
    };

   private:
    std::weak_ptr<Drone> drone;
};