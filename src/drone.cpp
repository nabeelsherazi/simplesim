#include "simplesim/drone.hpp"

#include <filesystem>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include <SFML/Graphics.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "simplesim/interfaces/resettable.hpp"
#include "simplesim/utils.hpp"

extern template class rclcpp::Publisher<geometry_msgs::msg::Vector3>;
extern template class rclcpp::Subscription<geometry_msgs::msg::Vector3>;
extern template class rclcpp::Subscription<nav_msgs::msg::Odometry>;

Drone::Drone(std::string node_name, DroneOptions& options) : Resettable(node_name), rclcpp::Node(node_name) {
    // Options
    this->options = options;
    // Initial position
    this->currentPosition = options.initialPosition;
    this->position_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("drone/position", 10);
    this->velocity_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("drone/velocity", 10);
    this->odometryPublisher = this->create_publisher<nav_msgs::msg::Odometry>("drone/odometry", 10);
    this->transformBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // Control mode
    this->controlMode = options.controlMode;
    switch (options.controlMode) {
        case DroneOptions::ControlMode::Acceleration:
            this->accelerationCommandSubscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
                "drone/accel_cmd", 10,
                [=](const std::shared_ptr<geometry_msgs::msg::Vector3> msg) { this->setAccelerationCommand(msg); });
            break;
        case DroneOptions::ControlMode::Velocity:
            this->velocityCommandSubscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
                "drone/velocity_cmd", 10,
                [=](const std::shared_ptr<geometry_msgs::msg::Vector3> msg) { this->setVelocityCommand(msg); });
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown control mode requested");
            throw std::runtime_error("Unknown control mode requested");
    }
    // Noise generator
    this->randomGenerator = std::mt19937(randomDevice());
    this->dist = std::uniform_real_distribution<>(-options.windIntensity, options.windIntensity);
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
    // Clamp to maximum accepted command
    if (simplesim::norm(this->accelerationCommand) > this->options.maxAcceleration) {
        this->accelerationCommand =
            this->accelerationCommand / simplesim::norm(this->accelerationCommand) * this->options.maxAcceleration;
    }
}

void Drone::setVelocityCommand(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    this->currentVelocity = sf::Vector2f(msg->x, msg->y);
    // Clamp to maximum accepted command
    if (simplesim::norm(this->currentVelocity) > this->options.maxVelocity) {
        this->currentVelocity =
            this->currentVelocity / simplesim::norm(this->currentVelocity) * this->options.maxVelocity;
    }
}

void Drone::tick(const sf::Time dt) {
    // Update current wind speed as a random walk
    currentWind.x += dist(randomGenerator);
    currentWind.y += dist(randomGenerator);

    // Update current drag
    // Smoothly blend between linear and quadratic drag up to the quadratic threshold, then just use quadratic drag
    auto velocitySquared = simplesim::squareComponents(this->currentVelocity);
    auto linearDrag = -this->options.linearDragConstant * this->currentVelocity;
    auto quadraticDrag = -this->options.quadraticDragConstant * velocitySquared;
    auto alpha = simplesim::norm(this->currentVelocity) / this->options.quadraticDragThreshold;
    this->currentDrag = (1 - alpha) * linearDrag + alpha * quadraticDrag;

    // Integrate for new velocity
    this->currentVelocity += dt.asSeconds() * (this->currentWind + this->currentDrag);

    // In acceleration control mode, integrate for new velocity (otherwise receive new velocity directly)
    if (this->controlMode == DroneOptions::ControlMode::Acceleration) {
        this->currentVelocity += dt.asSeconds() * this->accelerationCommand;
    }
    // Tick kinematics
    this->currentPosition += dt.asSeconds() * this->currentVelocity;
    // Update sprite
    this->sprite.setPosition(this->currentPosition);
    this->publish();
}

void Drone::publish() {
    geometry_msgs::msg::Vector3 position_msg;
    position_msg.x = this->currentPosition.x;
    position_msg.y = this->currentPosition.y;
    geometry_msgs::msg::Vector3 velocity_msg;
    velocity_msg.x = this->currentVelocity.x;
    velocity_msg.y = this->currentVelocity.y;
    this->position_publisher->publish(position_msg);
    this->velocity_publisher->publish(velocity_msg);

    // Publish odometry message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = this->currentPosition.x;
    odom_msg.pose.pose.position.y = this->currentPosition.y;
    odom_msg.twist.twist.linear.x = this->currentVelocity.x;
    odom_msg.twist.twist.linear.y = this->currentVelocity.y;
    this->odometryPublisher->publish(odom_msg);

    // Publish transform
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = this->currentPosition.x;
    transformStamped.transform.translation.y = this->currentPosition.y;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformBroadcaster->sendTransform(transformStamped);
}

bool Drone::reset() {
    this->currentPosition = {0, 0};
    this->currentVelocity = {0, 0};
    this->accelerationCommand = {0, 0};
    RCLCPP_INFO(this->get_logger(), "Reset drone");
    return true;
}