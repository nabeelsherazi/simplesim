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
#include "simplesim/utils.hpp"
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

    // Simulated drone model setup
    DroneOptions droneOptions{
        .controlMode = DroneOptions::ControlMode::Velocity,
        .initialPosition = {static_cast<float>(window.getSize().x) / 2, static_cast<float>(window.getSize().y) / 2},
        .windIntensity = 0.0F,
        .linearDragConstant = 0.0F,
        .quadraticDragConstant = 0.0F};
    std::shared_ptr<Drone> drone = std::make_shared<Drone>("drone_node", droneOptions);
    DroneDebugInfo droneDebugInfo(drone);
    for (auto& debugInfo : droneDebugInfo.allDebugInfo()) {
        debugText.addSelfUpdatingTextLine(debugInfo);
    }

    // Sprite for drone model, limit scope because setSprite will move in its guts
    {
        ManagedSprite droneSprite;
        if (!droneSprite.load(executableLocation / "data/sprites/drone.png")) {
            return 1;
        }
        droneSprite.scaleToSize(75.0F);
        droneSprite.setOriginRelative({0.5F, 0.75F});
        drone->setSprite(std::move(droneSprite));
    }

    // Trajectory controller
    ControllerOptions controllerOptions;
    std::shared_ptr<Controller> controller = std::make_shared<Controller>("controller_node", controllerOptions);
    ControllerDebugInfo controllerDebugInfo(controller);
    for (auto& debugInfo : controllerDebugInfo.allDebugInfo()) {
        debugText.addSelfUpdatingTextLine(debugInfo);
    }

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
    sf::Clock interframeTimer;
    bool paused = false;

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
            if (event.type == sf::Event::LostFocus) {
                paused = true;
            }
            if (event.type == sf::Event::GainedFocus) {
                paused = false;
                interframeTimer.restart();
            }
        }

        auto dt = interframeTimer.restart();

        window.clear(sf::Color::White);

        // Advance simulation
        if (paused) {
            continue;
        }
        drone->tick(dt);
        controller->tick(dt);
        sim->publishClock(clock.getElapsedTime());
        visuals.update(renderableEntities);
        visuals.render(window);

        // Debug text
        debugText.updateFixedTextLine(fpsDisplayHandle, fmt::format("{:.1f} fps", 1 / dt.asSeconds()));
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