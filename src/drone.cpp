#include <filesystem>
#include <geometry_msgs/msg/vector3.hpp>
#include "simplesim/drone.hpp"

Drone::Drone(std::string node_name, Drone::ControlMode controlMode) : Node(node_name) {
        this->position_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("/simplesim/drone/position", 10);
        switch (controlMode) {
            case Drone::ControlMode::Acceleration:
                this->accelerationCommandSubscriber = this->create_subscription<geometry_msgs::msg::Vector3>("/simplesim/drone/accel_cmd", 10, std::bind(&Drone::setAccelerationCommand, this, _1));
                break;
            case Drone::ControlMode::Velocity:
                this->velocityCommandSubscriber = this->create_subscription<geometry_msgs::msg::Vector3>("/simplesim/drone/velocity_cmd", 10, std::bind(&Drone::setVelocityCommand, this, _1));
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown control mode requested");
                throw std::runtime_error("Unknown control mode requested");
        }
    };

bool Drone::load(const std::filesystem::path& filename) {
        if (!this->texture.loadFromFile(filename))
        {
            return false;
        }
        this->texture.setSmooth(true);
        this->sprite.setTexture(this->texture);
        this->localBounds = this->sprite.getLocalBounds();
        // This sprite is a little bottom-heavy, set origin in middle horizontally and 2/3 the way down vertically
        this->sprite.setOrigin(this->localBounds.width / 2.0f, this->localBounds.height / 1.5f);
        return true;
    }


void Drone::scaleToSize(float size_x, float size_y) {
    this->sprite.setScale(size_x / this->localBounds.width, size_y / this->localBounds.height);
}

void Drone::scaleToSize(float size) {
    this->scaleToSize(size, size);
}

void Drone::setAccelerationCommand(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    this->accelerationCommand = sf::Vector2f(msg->x, msg->y);
    this->controlMode = ControlMode::Acceleration;
}

void Drone::setVelocityCommand(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    this->currentVelocity = sf::Vector2f(msg->x, msg->y);
    this->controlMode = ControlMode::Velocity;
}

void Drone::tick(const sf::Time dt) {
    if (this->controlMode == ControlMode::Acceleration) {
        this->currentVelocity += dt.asSeconds() * this->accelerationCommand;
    }
    this->sprite.move(dt.asSeconds() * this->currentVelocity);
    this->currentPosition = this->sprite.getPosition();
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
    this->sprite.setPosition(this->currentPosition);
}