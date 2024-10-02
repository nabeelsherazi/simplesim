#pragma once

#include <filesystem>
#include <random>

#include <tf2_ros/transform_broadcaster.h>
#include <SFML/Graphics.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "simplesim/interfaces/renderable.hpp"
#include "simplesim/interfaces/resettable.hpp"
#include "simplesim/managed_sprite.hpp"
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
    sf::Vector2f initialPosition = {0.0f, 0.0f};

    /// @brief Magnitude of the maximum "jerk" to the wind at any iteration. Current wind speed is calculated like
    /// currentWind += rand(-intensity, intensity)
    float windIntensity = 0.0f;

    /// @brief Maximum acceleration command the drone will accept
    float maxAcceleration = 10.0f;

    /// @brief Drag constant in the linear regime (friction drag)
    float linearDragConstant = 0.0005f;

    /// @brief Drag constant in the quadratic regime (pressure drag)
    float quadraticDragConstant = 0.0005f;

    /// @brief Speed threshold at which to switch from linear to quadratic drag
    float quadraticDragThreshold = 500.0f;
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

    DroneOptions options;

   private:
    ManagedSprite sprite;

    DroneOptions::ControlMode controlMode;

    sf::Vector2f accelerationCommand;
    sf::Vector2f currentVelocity;
    sf::Vector2f currentPosition;

    std::random_device randomDevice;
    std::mt19937 randomGenerator;
    std::uniform_real_distribution<> dist;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> transformBroadcaster;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocity_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr accelerationCommandSubscriber;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr velocityCommandSubscriber;
};