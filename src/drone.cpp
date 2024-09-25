#include "simplesim/drone.hpp"
#include <filesystem>
#include <geometry_msgs/msg/vector3.hpp>
#include <utility>

Drone::Drone(std::string node_name, DroneOptions& options) : Node(node_name) {
    // Initial position
    this->currentPosition = options.initialPosition;
    this->position_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("/simplesim/drone/position", 10);
    // Control mode
    switch (options.controlMode) {
        case DroneOptions::ControlMode::Acceleration:
            this->accelerationCommandSubscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
                "/simplesim/drone/accel_cmd", 10, std::bind(&Drone::setAccelerationCommand, this, _1));
            break;
        case DroneOptions::ControlMode::Velocity:
            this->velocityCommandSubscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
                "/simplesim/drone/velocity_cmd", 10, std::bind(&Drone::setVelocityCommand, this, _1));
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown control mode requested");
            throw std::runtime_error("Unknown control mode requested");
    }
};

Drone::~Drone() {};

void Drone::setSprite(ManagedSprite&& sprite) {
    this->sprite = std::move(sprite);
}

std::vector<const sf::Drawable*> Drone::getDrawables() const {
    return std::vector<const sf::Drawable*>{&(this->sprite)};
}

void Drone::setAccelerationCommand(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    this->accelerationCommand = sf::Vector2f(msg->x, msg->y);
}

void Drone::setVelocityCommand(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    this->currentVelocity = sf::Vector2f(msg->x, msg->y);
}

void Drone::tick(const sf::Time dt) {
    if (this->controlMode == DroneOptions::ControlMode::Acceleration) {
        this->currentVelocity += dt.asSeconds() * this->accelerationCommand;
    }
    // Calculate ground truth position
    this->currentPosition += dt.asSeconds() * this->currentVelocity;
    // Update sprite
    this->sprite.setPosition(this->currentPosition);
    this->publish();
}

void Drone::publish() {
    geometry_msgs::msg::Vector3 msg;
    msg.x = this->currentPosition.x;
    msg.y = this->currentPosition.y;
    this->position_publisher->publish(msg);
}

void Drone::reset() {
    this->currentPosition = {0, 0};
    this->currentVelocity = {0, 0};
    this->accelerationCommand = {0, 0};
    RCLCPP_INFO(this->get_logger(), "Reset drone");
}