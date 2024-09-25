#include <fmt/core.h>
#include <SFML/Graphics.hpp>
#include <filesystem>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "simplesim/controller.hpp"
#include "simplesim/drone.hpp"
#include "simplesim/managed_sprite.hpp"
#include "simplesim/shapes.hpp"
#include "simplesim/text.hpp"
#include "simplesim/visuals.hpp"

static const auto executableLocation = std::filesystem::canonical("/proc/self/exe").parent_path();

int main(int argc, char* argv[]) {
    // ROS node

    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("simplesim");

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
    int pidCommandDisplayHandle =
        debugText.addFixedTextLine("(kp) * error + (kd) * (error - lastError) / dt = command");
    int windDisplayHandle = debugText.addFixedTextLine("current wind: (0, 0)");

    DroneOptions droneOptions{.controlMode = DroneOptions::ControlMode::Velocity, .windIntensity = 50.0f};
    std::shared_ptr<Drone> drone = std::make_shared<Drone>("drone_node", droneOptions);
    {
        ManagedSprite droneSprite;
        droneSprite.load(executableLocation / "data/sprites/drone.png");
        droneSprite.scaleToSize(75.0f);
        droneSprite.setOriginRelative({0.5f, 0.75f});
        drone->setSprite(std::move(droneSprite));
    }

    ControllerOptions controllerOptions;
    std::shared_ptr<Controller> controller = std::make_shared<Controller>("controller_node", controllerOptions);

    Visuals visuals;

    auto resetService = node->create_service<std_srvs::srv::Trigger>(
        "~/reset", [&](__attribute__((unused)) const std_srvs::srv::Trigger::Request::SharedPtr request,
                       std_srvs::srv::Trigger::Response::SharedPtr response) {
            drone->reset();
            controller->reset();
            response->success = true;
        });

    executor.add_node(node);
    executor.add_node(drone);
    executor.add_node(controller);

    std::vector<const Renderable*> renderableEntities{drone.get(), controller.get()};

    sf::Clock clock;

    while (window.isOpen() && rclcpp::ok()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            // Handle window resizing
            if (event.type == sf::Event::Resized) {
                // Reset the view to the new size, but maintain the aspect ratio
                sf::FloatRect visibleArea(0, 0, event.size.width, event.size.height);
                window.setView(sf::View(visibleArea));
            }
            // Clicks set new waypoints
            if (event.type == sf::Event::MouseButtonReleased) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    // drone.addWaypoint(sf::Vector2f(event.mouseButton.x,
                    // event.mouseButton.y));
                    geometry_msgs::msg::Vector3 msg;
                    msg.x = event.mouseButton.x;
                    msg.y = event.mouseButton.y;
                    waypointPublisher->publish(msg);
                }
            }
        }

        auto dt = clock.restart();

        window.clear(sf::Color::White);

        // Controller
        drone->tick(dt);
        controller->tick(dt);
        visuals.update(renderableEntities);
        visuals.render(window);

        // Debug text
        debugText.updateFixedTextLine(fpsDisplayHandle, fmt::format("{:.1f} fps", 1 / dt.asSeconds()));
        debugText.updateFixedTextLine(
            errorDisplayHandle,
            fmt::format("x error: {:.2f}, y error: {:.2f}", controller->positionError.x, controller->positionError.y));
        debugText.updateFixedTextLine(
            pidCommandDisplayHandle,
            fmt::format("kp:={:.2f} * ({:.2f},{:.2f}) + kd:{:.2f} * "
                        "({:.2f},{:.2f}) = ({:.2f}, {:.2f})",
                        controller->kp_position, controller->positionError.x, controller->positionError.y,
                        controller->kd_position, controller->deltaError.x, controller->deltaError.y,
                        controller->velocityCommand.x, controller->velocityCommand.y));
        debugText.updateFixedTextLine(
            windDisplayHandle, fmt::format("wind: ({:.1f}, {:.1f})", drone->currentWind.x, drone->currentWind.y));
        // debugText.updateFixedTextLine(velocityDisplayHandle, fmt::format("x
        // velocity: {:.2f}, y velocity: {:.2f}", commandedVelocity.x,
        // commandedVelocity.y));
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