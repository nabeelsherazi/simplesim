#include <filesystem>
#include <memory>
#include <vector>

#include <fmt/core.h>
#include <SFML/Graphics.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "simplesim/controller.hpp"
#include "simplesim/drone.hpp"
#include "simplesim/managed_sprite.hpp"
#include "simplesim/shapes.hpp"
#include "simplesim/sim.hpp"
#include "simplesim/text.hpp"
#include "simplesim/visuals.hpp"

static const auto executableLocation = std::filesystem::canonical("/proc/self/exe").parent_path();

int main(int argc, char* argv[]) {
    // ROS
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    // SFML setup
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(640, 480), "simplesim!", sf::Style::Default, settings);
    window.setVerticalSyncEnabled(true);

    // Setup debug console
    DebugTextConsole debugText;
    debugText.loadFont(executableLocation / "data/fonts/OpenSans-Regular.ttf");
    int fpsDisplayHandle = debugText.addFixedTextLine("0 fps");
    int waypointDisplayHandle = debugText.addFixedTextLine("current waypoint: {}");
    int errorDisplayHandle = debugText.addFixedTextLine("x error: 0, y error: 0");
    int positionPidDisplayHandle =
        debugText.addFixedTextLine("(kp) * error + (kd) * (error - lastError) / dt = command");
    int velocityPidDisplayHandle =
        debugText.addFixedTextLine("(kp) * error + (kd) * (error - lastError) / dt = command");
    int windDisplayHandle = debugText.addFixedTextLine("current wind: (0, 0)");

    // Simulated drone model setup
    DroneOptions droneOptions{
        .controlMode = DroneOptions::ControlMode::Acceleration,
        .initialPosition = {static_cast<float>(window.getSize().x) / 2, static_cast<float>(window.getSize().y) / 2},
        .windIntensity = 0.0f};
    std::shared_ptr<Drone> drone = std::make_shared<Drone>("drone_node", droneOptions);

    // Sprite for drone model, limit scope because setSprite will move in its guts
    {
        ManagedSprite droneSprite;
        if (!droneSprite.load(executableLocation / "data/sprites/drone.png")) {
            return 1;
        }
        droneSprite.scaleToSize(75.0f);
        droneSprite.setOriginRelative({0.5f, 0.75f});
        drone->setSprite(std::move(droneSprite));
    }

    // Trajectory controller
    ControllerOptions controllerOptions;
    std::shared_ptr<Controller> controller = std::make_shared<Controller>("controller_node", controllerOptions);

    // Simulation main node (reset service, manual waypoint publisher)
    auto sim = std::make_shared<SimNode>("simplesim");
    sim->AddToResetChain({controller.get(), drone.get()});

    // Renderer
    Visuals visuals;
    std::vector<const Renderable*> renderableEntities{controller.get(), drone.get()};

    executor.add_node(sim);
    executor.add_node(drone);
    executor.add_node(controller);

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
                debugText.addTemporaryTextLine(
                    fmt::format("Resized window to {}x{}!", event.size.width, event.size.height), sf::seconds(2));
            }
            // Clicks set new waypoints
            if (event.type == sf::Event::MouseButtonReleased) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    sim->publishManualWaypoint(event.mouseButton);
                }
            }
        }

        auto dt = clock.restart();

        window.clear(sf::Color::White);

        // Advance simulation
        drone->tick(dt);
        controller->tick(dt);
        visuals.update(renderableEntities);
        visuals.render(window);

        // Debug text
#pragma region debugtext
        debugText.updateFixedTextLine(fpsDisplayHandle, fmt::format("{:.1f} fps", 1 / dt.asSeconds()));
        debugText.updateFixedTextLine(waypointDisplayHandle,
                                      fmt::format("current waypoint: {}", controller->currentWaypointIndex));
        debugText.updateFixedTextLine(
            errorDisplayHandle,
            fmt::format("x error: {:.2f}, y error: {:.2f}, norm: {:.2f}", controller->positionError.x,
                        controller->positionError.y, simplesim::norm(controller->positionError)));
        debugText.updateFixedTextLine(
            positionPidDisplayHandle,
            fmt::format("kp_p:={:.2f} * ({:.2f},{:.2f}) + kd_p:{:.2f} * "
                        "({:.2f},{:.2f}) = ({:.2f}, {:.2f})",
                        controller->kp_position, controller->positionError.x, controller->positionError.y,
                        controller->kd_position, controller->deltaPositionError.x, controller->deltaPositionError.y,
                        controller->velocityCommand.x, controller->velocityCommand.y));
        debugText.updateFixedTextLine(
            velocityPidDisplayHandle,
            fmt::format("kp_v:={:.2f} * ({:.2f},{:.2f}) + kd_v:{:.2f} * "
                        "({:.2f},{:.2f}) = ({:.2f}, {:.2f})",
                        controller->kp_velocity, controller->velocityError.x, controller->velocityError.y,
                        controller->kd_velocity, controller->deltaVelocityError.x, controller->deltaVelocityError.y,
                        controller->accelerationCommand.x, controller->accelerationCommand.y));
        debugText.updateFixedTextLine(
            windDisplayHandle, fmt::format("wind: ({:.1f}, {:.1f})", drone->currentWind.x, drone->currentWind.y));
        // debugText.updateFixedTextLine(velocityDisplayHandle, fmt::format("x
        // velocity: {:.2f}, y velocity: {:.2f}", commandedVelocity.x,
        // commandedVelocity.y));
#pragma endregion
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