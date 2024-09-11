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
#include "simplesim/drone.hpp"
#include "simplesim/shapes.hpp"

int main(int argc, char* argv[])
{
    // ROS node

    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("simplesim");

    geometry_msgs::msg::Pose2D desiredPose;

    auto poseSubscription = node->create_subscription<geometry_msgs::msg::Pose2D>("~/goal_pose", 10, [&desiredPose](geometry_msgs::msg::Pose2D::UniquePtr msg) {desiredPose = *msg;});

    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(640, 480), "simplesim!", sf::Style::Default, settings);
    window.setVerticalSyncEnabled(true);

    DebugTextConsole debugText;
    debugText.loadFont(OPEN_SANS_REGULAR);
    int fpsDisplayHandle = debugText.addFixedTextLine("0 fps");
    int errorDisplayHandle = debugText.addFixedTextLine("x error: 0, y error: 0");
    int velocityDisplayHandle = debugText.addFixedTextLine("x velocity: 0, y velocity: 0");

    Drone drone({300.0f, 300.0f});
    drone.load(executableLocation / "data/sprites/drone.png");
    drone.addWaypoint({300.f, 200.f});

    std::vector<XShape> waypointMarks {};

    sf::Clock clock;

    while (window.isOpen() && rclcpp::ok())
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
            // Clicks set new waypoints
            if (event.type == sf::Event::MouseButtonReleased) {
                if (event.mouseButton.button == sf::Mouse::Left)
                    {
                        drone.addWaypoint(sf::Vector2f(event.mouseButton.x, event.mouseButton.y));
                        waypointMarks.push_back(XShape(sf::Vector2f(event.mouseButton.x, event.mouseButton.y), 12.5f));
                    }
            }
        }

        auto dt = clock.restart();

        window.clear(sf::Color::White);
        
        // Drone
        drone.tick(dt);
        window.draw(drone.sprite);

        // Debug marks
        for (auto& waypointMark : waypointMarks) {
            window.draw(waypointMark);
        }

        // Debug text
        debugText.updateFixedTextLine(fpsDisplayHandle, fmt::format("{:.1f} fps", 1 / dt.asSeconds()));
        // debugText.updateFixedTextLine(errorDisplayHandle, fmt::format("x error: {:.2f}, y error: {:.2f}", error.x, error.y));
        // debugText.updateFixedTextLine(velocityDisplayHandle, fmt::format("x velocity: {:.2f}, y velocity: {:.2f}", commandedVelocity.x, commandedVelocity.y));
        debugText.tick(dt);

        for (auto& textLine : debugText.drawables()) {
            window.draw(textLine);
        }
        
        window.display();
        rclcpp::spin_some(node);
    }

    // In case we get here from rclcpp::ok -> false
    if (window.isOpen()) {
        window.close();
    }

    rclcpp::shutdown();
    return 0;
}