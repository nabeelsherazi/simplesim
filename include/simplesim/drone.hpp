#pragma once

#include <SFML/Graphics.hpp>
#include <filesystem>
#include <geometry_msgs/msg/vector3.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>

#include "simplesim/managed_sprite.hpp"
#include "simplesim/renderable.hpp"
#include "simplesim/visuals.hpp"

using namespace std::placeholders;

struct DroneOptions {
    /// @brief Enum for the simulated control mode (whether to accept a velocity
    /// or acceleration command)
    enum class ControlMode { Velocity, Acceleration };

    sf::Vector2f initialPosition = {0.0f, 0.0f};
    DroneOptions::ControlMode controlMode = DroneOptions::ControlMode::Acceleration;
    float windIntensity = 0.0f;
};

class Drone : public rclcpp::Node, public Renderable {
   public:
    /// @brief Class that represents the "real world" drone
    /// @param node_name Name of the ROS2 node
    /// @param controlMode the control mode offered
    Drone(std::string node_name, DroneOptions& options);

    virtual ~Drone();

    void setSprite(ManagedSprite&& sprite);

    void setAccelerationCommand(const geometry_msgs::msg::Vector3::SharedPtr msg);

    void setVelocityCommand(const geometry_msgs::msg::Vector3::SharedPtr msg);

    /// @brief Advance the simulation
    /// @param dt the amount of time that has passed since the last call
    void tick(const sf::Time dt);

    void publish();

    void reset();

    std::vector<const sf::Drawable*> getDrawables() const override;

    sf::Vector2f currentWind;

   private:
    ManagedSprite sprite;

    DroneOptions::ControlMode controlMode;

    sf::Vector2f accelerationCommand;
    sf::Vector2f currentVelocity;
    sf::Vector2f currentPosition;

    std::random_device randomDevice;
    std::mt19937 randomGenerator;
    std::uniform_real_distribution<> dist;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr accelerationCommandSubscriber;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr velocityCommandSubscriber;
};