#include <iostream>
#include <filesystem>
#include <string>
#include <sstream>
#include <fmt/core.h>
#include <SFML/Graphics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "simplesim/text.hpp"
#include "simplesim/drone.hpp"
#include "simplesim/shapes.hpp"
#include "simplesim/controller.hpp"

static const auto executableLocation = std::filesystem::canonical("/proc/self/exe").parent_path();

int main(int argc, char* argv[])
{
    // ROS node

    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("simplesim");

    geometry_msgs::msg::Pose2D desiredPose;

    auto poseSubscription = node->create_subscription<geometry_msgs::msg::Pose2D>("~/goal_pose", 10, [&desiredPose](geometry_msgs::msg::Pose2D::UniquePtr msg) {desiredPose = *msg;});
    auto waypointPublisher = node->create_publisher<geometry_msgs::msg::Vector3>("/simplesim/drone/waypoint", 10);
    rclcpp::executors::StaticSingleThreadedExecutor executor;

    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(640, 480), "simplesim!", sf::Style::Default, settings);
    window.setVerticalSyncEnabled(true);

    DebugTextConsole debugText;
    debugText.loadFont(OPEN_SANS_REGULAR);
    int fpsDisplayHandle = debugText.addFixedTextLine("0 fps");
    int errorDisplayHandle = debugText.addFixedTextLine("x error: 0, y error: 0");
    int pidCommandDisplayHandle = debugText.addFixedTextLine("(kp) * error + (kd) * (error - lastError) / dt = command");

    std::shared_ptr<Drone> drone = std::make_shared<Drone>("drone_node", Drone::ControlMode::Velocity);
    drone->load(executableLocation / "data/sprites/drone.png");
    drone->scaleToSize(75.0f);

    std::shared_ptr<Controller> controller = std::make_shared<Controller>("controller_node", sf::Vector2f(300.f, 200.f));

    auto resetService = node->create_service<std_srvs::srv::Trigger>("~/reset", [&](__attribute__((unused))const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response) {
        drone->reset();
        controller->reset();
        response->success = true;
    });

    executor.add_node(node);
    executor.add_node(drone);
    executor.add_node(controller);

    std::vector<CrossShape> waypointMarks {};

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
                        // drone.addWaypoint(sf::Vector2f(event.mouseButton.x, event.mouseButton.y));
                        geometry_msgs::msg::Vector3 msg;
                        msg.x = event.mouseButton.x;
                        msg.y = event.mouseButton.y;
                        waypointPublisher->publish(msg);
                        waypointMarks.push_back(CrossShape(sf::Vector2f(event.mouseButton.x, event.mouseButton.y), 12.5f));
                    }
            }
        }

        auto dt = clock.restart();

        window.clear(sf::Color::White);
        
        // Controller
        drone->tick(dt);
        controller->tick(dt);
        window.draw(drone->sprite);

        // Debug marks
        for (auto& waypointMark : waypointMarks) {
            window.draw(waypointMark);
        }

        // Debug text
        debugText.updateFixedTextLine(fpsDisplayHandle, fmt::format("{:.1f} fps", 1 / dt.asSeconds()));
        debugText.updateFixedTextLine(errorDisplayHandle, fmt::format("x error: {:.2f}, y error: {:.2f}", controller->positionError.x, controller->positionError.y));
        debugText.updateFixedTextLine(pidCommandDisplayHandle, fmt::format("kp:={:.2f} * ({:.2f},{:.2f}) + kd:{:.2f} * ({:.2f},{:.2f}) = ({:.2f}, {:.2f})", controller->kp_position, controller->positionError.x, controller->positionError.y, controller->kd_position, controller->deltaError.x, controller->deltaError.y, controller->velocityCommand.x, controller->velocityCommand.y));
        // debugText.updateFixedTextLine(velocityDisplayHandle, fmt::format("x velocity: {:.2f}, y velocity: {:.2f}", commandedVelocity.x, commandedVelocity.y));
        debugText.tick(dt);

        for (auto& textLine : debugText.drawables()) {
            window.draw(textLine);
        }
        
        window.display();
        executor.spin_some();
    }

    // In case we get here from rclcpp::ok -> false
    if (window.isOpen()) {
        window.close();
    }

    rclcpp::shutdown();
    return 0;
}