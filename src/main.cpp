#include <iostream>
#include <filesystem>
#include <string>
#include <sstream>
#include <fmt/core.h>
#include <SFML/Graphics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include "simplesim/main.hpp"
#include "simplesim/text.hpp"

int main(int argc, char* argv[])
{
    // ROS node

    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("simplesim");

    geometry_msgs::msg::Pose2D desiredPose;

    auto poseSubscription = node->create_subscription<geometry_msgs::msg::Pose2D>("goal_pose", 10, [&desiredPose](geometry_msgs::msg::Pose2D::UniquePtr msg) {desiredPose = *msg;});

    float droneSpeed = 5.0;
    float yawSpeed = 5.0;
    float p = 0.2;

    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(640, 480), "simplesim!", sf::Style::Default, settings);
    window.setVerticalSyncEnabled(true);

    DebugTextConsole debugText;
    debugText.loadFont(OPEN_SANS_REGULAR);
    int fpsDisplayHandle = debugText.addFixedTextLine("0 fps");
    int errorDisplayHandle = debugText.addFixedTextLine("x error: 0, y error: 0");
    int velocityDisplayHandle = debugText.addFixedTextLine("x velocity: 0, y velocity: 0");

    ManagedSprite droneSprite;
    if (!droneSprite.load(executableLocation / "data/sprites/drone.png")) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to load drone sprite!");
        return 1;
    }

    droneSprite.scaleToSize(75.0, 75.0);
    droneSprite.setPosition(300.0, 300.0);

    sf::Clock clock;

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            // Handle window resizing
            if (event.type == sf::Event::Resized) {
                // Reset the view to the new size, but maintain the aspect ratio
                sf::FloatRect visibleArea(0, 0, event.size.width, event.size.height);
                window.setView(sf::View(visibleArea));
            }
        }

        auto dt = clock.restart();

        window.clear(sf::Color::White);
        
        // Drone
        auto currentPosition = droneSprite.getPosition();
        auto goalPosition = sf::Vector2f(desiredPose.x, desiredPose.y);
        auto error = goalPosition - currentPosition;
        auto commandedVelocity = p * error * droneSpeed;
        droneSprite.move(dt.asSeconds() * commandedVelocity);
        auto orientationError = desiredPose.theta - droneSprite.getRotation();
        auto commandedYaw = p * orientationError * yawSpeed;
        droneSprite.rotate(dt.asSeconds() * commandedYaw);
        window.draw(droneSprite);

        // Debug text
        debugText.updateFixedTextLine(fpsDisplayHandle, fmt::format("{:.1f} fps", 1 / dt.asSeconds()));
        debugText.updateFixedTextLine(errorDisplayHandle, fmt::format("x error: {}, y error: {}", error.x, error.y));
        debugText.updateFixedTextLine(velocityDisplayHandle, fmt::format("x velocity: {}, y velocity: {}", commandedVelocity.x, commandedVelocity.y));
        debugText.tick(dt);

        for (auto& textLine : debugText.drawables()) {
            window.draw(textLine);
        }
        
        window.display();
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}