#pragma once

#include <filesystem>
#include <SFML/Graphics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>

using namespace std::placeholders;

class Drone : public rclcpp::Node {

public:

    /// @brief Enum for the simulated control mode (whether to accept a velocity or acceleration command)
    enum class ControlMode {
        Velocity,
        Acceleration
    };

    /// @brief Class that represents the "real world" drone
    /// @param node_name Name of the ROS2 node
    /// @param controlMode the control mode offered
    Drone(std::string node_name, Drone::ControlMode controlMode = ControlMode::Acceleration);

    /// @brief Loads sprite
    /// @param filename file to load sprite from
    /// @return whether the sprite was successfully loaded
    bool load(const std::filesystem::path& filename);

    /// @brief Scales sprite to given size in pixels
    /// @param size_x x size pixels
    /// @param size_y y size pixels
    void scaleToSize(float size_x, float size_y);

    /// @brief Scaled sprite to given size in pixels
    /// @param size size to scale both x and y to, pixels
    void scaleToSize(float size);

    void setAccelerationCommand(const geometry_msgs::msg::Vector3::SharedPtr msg);

    void setVelocityCommand(const geometry_msgs::msg::Vector3::SharedPtr msg);

    /// @brief Advance the simulation
    /// @param dt the amount of time that has passed since the last call
    void tick(const sf::Time dt);

    void publish();

    void reset();
    
    sf::Sprite sprite;

private:
    sf::Texture texture;
    sf::FloatRect localBounds;

    Drone::ControlMode controlMode;

    sf::Vector2f accelerationCommand;
    sf::Vector2f currentVelocity;
    sf::Vector2f currentPosition;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr accelerationCommandSubscriber;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr velocityCommandSubscriber;
};