#pragma once

#include <filesystem>
#include <SFML/Graphics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <functional>

using namespace std::placeholders;

class Drone : public rclcpp::Node {

public:
    /// @brief Class that represents the "real world" drone
    Drone(std::string node_name) : Node(node_name) {
        this->position_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("/simplesim/drone/position", 10);
        this->jerkCommandSubscriber = this->create_subscription<geometry_msgs::msg::Vector3>("/simplesim/drone/jerk_cmd", 10, std::bind(&Drone::setJerkCommand, this, _1));
    };

    /// @brief Load sprite
    /// @param filename file to load sprite from
    /// @return whether the sprite was successfully loaded
    bool load(const std::filesystem::path& filename) {
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

    void scaleToSize(float size_x, float size_y) {
        this->sprite.setScale(size_x / this->localBounds.width, size_y / this->localBounds.height);
    }

    void scaleToSize(float size) {
        this->scaleToSize(size, size);
    }

    void setJerkCommand(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        this->linearJerkCommand = sf::Vector2f(msg->x, msg->y);
        RCLCPP_INFO(this->get_logger(), "jerk command: %f, %f", msg->x, msg->y);
    }

    void tick(const sf::Time dt) {
        this->currentAcceleration += dt.asSeconds() * linearJerkCommand;
        this->currentVelocity += dt.asSeconds() * this->currentAcceleration;
        this->sprite.move(dt.asSeconds() * this->currentVelocity);
        this->currentPosition = this->sprite.getPosition();
        this->publish();
    }

    void publish() {
        geometry_msgs::msg::Vector3 msg;
        msg.x = this->currentPosition.x;
        msg.y = this->currentPosition.y;
        this->position_publisher->publish(msg);
    }
    
    sf::Sprite sprite;

private:
    sf::Texture texture;
    sf::FloatRect localBounds;

    sf::Vector2f linearJerkCommand;
    sf::Vector2f currentAcceleration;
    sf::Vector2f currentVelocity;
    sf::Vector2f currentPosition;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr jerkCommandSubscriber;
};