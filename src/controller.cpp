#include "simplesim/controller.hpp"

#include <SFML/Graphics.hpp>
#include <SFML/Graphics/Drawable.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>

#include "simplesim/interfaces/renderable.hpp"
#include "simplesim/interfaces/resettable.hpp"
#include "simplesim/utils.hpp"

Controller::Controller(const std::string& node_name, ControllerOptions& options)
    : Resettable(node_name), rclcpp::Node(node_name) {
    // Set options
    this->options = options;
    this->kp_position = options.positionControllerTune.kp;
    this->kd_position = options.positionControllerTune.kd;
    this->kp_velocity = options.velocityControllerTune.kp;
    this->kd_velocity = options.velocityControllerTune.kd;
    this->lookaheadDistance = options.lookaheadDistance;
    this->waypointEpsilon = options.waypointEpsilon;
    this->currentPosition = options.initialPosition;

    // World input subscribers
    this->positionSubscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/simplesim/drone/position", 10, [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
            this->currentPosition = sf::Vector2f(msg->x, msg->y);
            this->lookaheadCircle.setPosition(this->currentPosition);
        });

    this->velocitySubscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/simplesim/drone/velocity", 10,
        [this](geometry_msgs::msg::Vector3::SharedPtr msg) { this->currentVelocity = sf::Vector2f(msg->x, msg->y); });

    // Trajectory input subscribers
    this->waypointSubscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/simplesim/drone/waypoint", 10,
        [this](geometry_msgs::msg::Vector3::SharedPtr msg) { this->addWaypoint(sf::Vector2f(msg->x, msg->y)); });

    // Command output publishers
    this->accelerationCommandPublisher =
        this->create_publisher<geometry_msgs::msg::Vector3>("/simplesim/drone/accel_cmd", 10);
    this->velocityCommandPublisher =
        this->create_publisher<geometry_msgs::msg::Vector3>("/simplesim/drone/velocity_cmd", 10);

    // Parameter subscribers
    this->declare_parameter("kp_position", 0.0f);
    this->declare_parameter("kd_position", 0.0f);
    parameterSubscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
    parameterCallbackHandle =
        parameterSubscriber->add_parameter_callback("kp_position", std::bind(&Controller::parameterCallback, this, _1));
    parameterCallbackHandle =
        parameterSubscriber->add_parameter_callback("kd_position", std::bind(&Controller::parameterCallback, this, _1));

    // Configure drawables
    waypointPathLines.setPrimitiveType(sf::LinesStrip);
    lookaheadCircle.setOrigin({options.lookaheadDistance, options.lookaheadDistance});
    lookaheadCircle.setRadius(options.lookaheadDistance);
    lookaheadCircle.setPosition(options.initialPosition);
    lookaheadCircle.setFillColor(sf::Color::Transparent);
    lookaheadCircle.setOutlineThickness(1.0f);
    lookaheadCircle.setOutlineColor(sf::Color::Black);
    lookaheadPoint.setRadius(5.0f);
    lookaheadPoint.setOrigin({5.0f, 5.0f});
    lookaheadPoint.setFillColor(sf::Color::Red);
};

void Controller::parameterCallback(const rclcpp::Parameter& param) {
    if (param.get_name() == "kp_position") {
        this->kp_position = static_cast<float>(param.as_double());
    } else if (param.get_name() == "kd_position") {
        this->kd_position = static_cast<float>(param.as_double());
    }
    RCLCPP_INFO(this->get_logger(), "Parameter %s updated :)", param.get_name().c_str());
}

void Controller::addWaypoint(sf::Vector2f wpt) {
    this->waypointList.push_back(wpt);
    this->waypointMarks.addCross(wpt);
    this->waypointPathLines.append(sf::Vertex(wpt, sf::Color::Black));
}

void Controller::tick(sf::Time dt) {
    if (currentWaypointIndex < waypointList.size()) {
        // Setpoint is next waypoint
        currentSetpoint = waypointList[currentWaypointIndex];
        // If there's a waypoint after this one, use pure pursuit instead
        if (currentWaypointIndex + 1 < waypointList.size() && this->options.usePurePursuit) {
            bool exitedLookaheadRadius = false;
            // First interpolate from current position to next waypoint, after that between next waypoint and next+1
            // waypoint
            int i = 0;
            auto lastWaypoint = this->currentPosition;
            while (currentWaypointIndex + i < waypointList.size() && !exitedLookaheadRadius) {
                // Walk between the waypoints until we leave the lookahead circle
                for (float t = 0.0f; t <= 1.0f; t += 0.05f) {
                    currentSetpoint = lastWaypoint + (t * (waypointList[currentWaypointIndex + i] - lastWaypoint));
                    if (simplesim::norm(currentSetpoint - currentPosition) >= lookaheadDistance) {
                        exitedLookaheadRadius = true;
                        break;
                    }
                }
                lastWaypoint = waypointList[currentWaypointIndex + i];
                i += 1;
            }
        }
        lookaheadPoint.setPosition(currentSetpoint);
        // Cascade PID controller
        // Outer loop: position error -> desired velocity
        positionError = currentSetpoint - currentPosition;
        deltaPositionError = (positionError - lastPositionError) / dt.asSeconds();
        velocityCommand = kp_position * positionError + kd_position * deltaPositionError;
        lastPositionError = positionError;

        // Inner loop: velocity error -> desired acceleration
        velocityError = velocityCommand - currentVelocity;
        deltaVelocityError = (velocityError - lastVelocityError) / dt.asSeconds();
        accelerationCommand = kp_velocity * velocityError;
        lastVelocityError = velocityError;
        // Clamp acceleration
        auto accelerationNorm = simplesim::norm(accelerationCommand);
        // if (accelerationNorm > maxAcceleration) {
        //     accelerationCommand = accelerationCommand / accelerationNorm * maxAcceleration;
        // }

        // Move to next waypoint if we're close enough to the current one
        if (simplesim::norm(waypointList[currentWaypointIndex] - currentPosition) <= waypointEpsilon &&
            currentWaypointIndex != waypointList.size() - 1) {
            this->currentWaypointIndex++;
            RCLCPP_INFO(this->get_logger(), "Moving to waypoint %i", currentWaypointIndex);
        }
    }
    geometry_msgs::msg::Vector3 msg;
    msg.x = accelerationCommand.x;
    msg.y = accelerationCommand.y;
    this->accelerationCommandPublisher->publish(msg);
}

bool Controller::reset() {
    auto zero = sf::Vector2f(0.0f, 0.0f);
    this->currentWaypointIndex = 0;
    this->waypointList.clear();
    this->waypointMarks.clear();
    this->waypointPathLines.clear();
    this->lastPositionError = zero;
    this->lastVelocityError = zero;
    this->lookaheadPoint.setPosition(zero);
    RCLCPP_INFO(this->get_logger(), "Reset controller");
    return true;
}

std::vector<const sf::Drawable*> Controller::getDrawables() const {
    std::vector<const sf::Drawable*> drawables;
    drawables.push_back(&waypointMarks);
    drawables.push_back(&waypointPathLines);
    drawables.push_back(&lookaheadCircle);
    drawables.push_back(&lookaheadPoint);
    return drawables;
}