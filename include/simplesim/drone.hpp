#pragma once

#include <vector>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <SFML/Graphics.hpp>
#include <cmath>
#include "simplesim/utils.hpp"


class Drone {

public:
    Drone(sf::Vector2f initialPosition) :
    waypointEpsilon(10.0f),
    p(0.2f),
    d(0.1f),
    maxSpeed(5.0f)
    {};

    void setControllerP(float p) {
        this->p = p;
    }

    void setControllerD(float d) {
        this->d = d;
    }

    void setMaxSpeed(float maxSpeed) {
        this->maxSpeed = maxSpeed;
    }

    void setWaypointEpsilon(float eps) {
        this-> waypointEpsilon = eps;
    }

    bool load(const std::filesystem::path& filename) {
        if (!this->texture.loadFromFile(filename))
            return false;
        this->texture.setSmooth(true);
        this->sprite.setTexture(this->texture);
        auto localBounds = this->sprite.getLocalBounds();
        // This sprite is a little bottom-heavy, set origin in middle horizontally and 2/3 the way down vertically
        this->sprite.setOrigin(localBounds.width / 2.0f, localBounds.height / 1.5f);
        this->sprite.setScale(75.0f / localBounds.width, 75.0f / localBounds.height);
        this->currentPosition = this->sprite.getPosition();
        return true;
    }

    void addWaypoint(sf::Vector2f wpt) {
        this->waypointList.push_back(wpt);
    }

    void tick(sf::Time dt) {
        if (currentWaypointIndex < waypointList.size()) {
            // Calculate velocity command
            auto nextWaypointPosition = waypointList[currentWaypointIndex];
            auto error = nextWaypointPosition - currentPosition;
            auto commandedVelocity = p * error * maxSpeed;

            // Move based on commanded velocity
            this->sprite.move(dt.asSeconds() * commandedVelocity);
            this->currentPosition = sprite.getPosition();

            // Move to next waypoint if we're close enough to the current one
            if (norm(nextWaypointPosition - currentPosition) <= waypointEpsilon) {
                this->currentWaypointIndex++;
            }
        }
    }

    sf::Sprite sprite;

private:

    float waypointEpsilon;

    float p;
    float d;

    float maxSpeed;

    sf::Vector2f currentPosition;
    int currentWaypointIndex = 0;
    std::vector<sf::Vector2f> waypointList;

    sf::Texture texture;

};