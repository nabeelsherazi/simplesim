#include "simplesim/controller.hpp"

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include <fmt/core.h>
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/Drawable.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>

#include "simplesim/interfaces/renderable.hpp"
#include "simplesim/interfaces/resettable.hpp"
#include "simplesim/utils.hpp"

extern template class rclcpp::Publisher<geometry_msgs::msg::Vector3>;
extern template class rclcpp::Subscription<geometry_msgs::msg::Vector3>;

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

    // Validation

    // If pure pursuit is enabled, set waypoint epsilon to lookahead distance
    if (options.usePurePursuit) {
        this->waypointEpsilon = this->lookaheadDistance;
    }

    // World input subscribers
    this->positionSubscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
        "drone/position", 10, [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
            this->currentPosition = sf::Vector2f(msg->x, msg->y);
            this->lookaheadCircle.setPosition(this->currentPosition);
        });

    this->velocitySubscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
        "drone/velocity", 10,
        [this](geometry_msgs::msg::Vector3::SharedPtr msg) { this->currentVelocity = sf::Vector2f(msg->x, msg->y); });

    // Trajectory input subscribers
    this->waypointSubscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
        "drone/waypoint", 10,
        [this](geometry_msgs::msg::Vector3::SharedPtr msg) { this->addWaypoint(sf::Vector2f(msg->x, msg->y)); });

    // Command output publishers
    this->accelerationCommandPublisher = this->create_publisher<geometry_msgs::msg::Vector3>("drone/accel_cmd", 10);
    this->velocityCommandPublisher = this->create_publisher<geometry_msgs::msg::Vector3>("drone/velocity_cmd", 10);

    // Parameter subscribers
    this->declare_parameter("kp_position", this->options.positionControllerTune.kp);
    this->declare_parameter("kd_position", this->options.positionControllerTune.kd);
    this->declare_parameter("kp_velocity", this->options.velocityControllerTune.kp);
    this->declare_parameter("kd_velocity", this->options.velocityControllerTune.kd);
    this->declare_parameter("lookahead_distance", this->options.lookaheadDistance);
    this->declare_parameter("waypoint_epsilon", this->options.waypointEpsilon);
    this->declare_parameter("max_acceleration", this->options.maxAcceleration);
    this->declare_parameter("use_pure_pursuit", this->options.usePurePursuit);

    parameterSubscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
    parameterCallbackHandles.push_back(parameterSubscriber->add_parameter_callback(
        "kp_position", [this](auto param) { this->parameterCallback(param); }));
    parameterCallbackHandles.push_back(parameterSubscriber->add_parameter_callback(
        "kd_position", [this](auto param) { this->parameterCallback(param); }));

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
    }

    else if (param.get_name() == "kd_position") {
        this->kd_position = static_cast<float>(param.as_double());
    }

    else if (param.get_name() == "kp_velocity") {
        this->kp_velocity = static_cast<float>(param.as_double());
    }

    else if (param.get_name() == "kd_velocity") {
        this->kd_velocity = static_cast<float>(param.as_double());
    }

    else if (param.get_name() == "lookahead_distance") {
        this->lookaheadDistance = static_cast<float>(param.as_double());
        this->lookaheadCircle.setRadius(this->lookaheadDistance);
        this->lookaheadCircle.setOrigin({this->lookaheadDistance, this->lookaheadDistance});
    }

    else if (param.get_name() == "waypoint_epsilon") {
        this->waypointEpsilon = static_cast<float>(param.as_double());
    }

    else if (param.get_name() == "max_acceleration") {
        this->options.maxAcceleration = static_cast<float>(param.as_double());
    }

    else if (param.get_name() == "use_pure_pursuit") {
        this->options.usePurePursuit = param.as_bool();
        if (this->options.usePurePursuit) {
            this->waypointEpsilon = this->options.lookaheadDistance;
        } else {
            this->waypointEpsilon = this->options.waypointEpsilon;
        }
    }
    RCLCPP_INFO(this->get_logger(), "Parameter %s updated :)", param.get_name().c_str());
}

void Controller::addWaypoint(sf::Vector2f wpt) {
    RCLCPP_INFO(this->get_logger(),
                fmt::format("Added waypoint {} ({:.0f},{:.0f})", this->waypointList.size(), wpt.x, wpt.y).c_str());
    this->waypointList.push_back(wpt);
    this->waypointMarks.addCross(wpt);
    this->waypointPathLines.append(sf::Vertex(wpt, sf::Color::Black));
}

void Controller::tick(sf::Time dt) {
    if (currentWaypointIndex < waypointList.size()) {
        // Setpoint is next waypoint
        currentSetpoint = waypointList[currentWaypointIndex];

        // If there's a waypoint after this one, use pure pursuit instead (if enabled)
        if (currentWaypointIndex + 1 < waypointList.size() && this->options.usePurePursuit) {
            // Define lookahead distance (L_d)
            lookaheadDistance = this->options.lookaheadDistance;

            // If adaptive lookahead is enabled, lookahead distance is increased as velocity increases
            if (this->options.useAdaptiveLookahead) {
                lookaheadDistance =
                    simplesim::clamp(this->options.minLookaheadDistance +
                                         this->options.adaptiveLookaheadGain * simplesim::norm(currentVelocity),
                                     this->options.minLookaheadDistance, this->options.maxLookaheadDistance);
            }

            // Initialize best lookahead point as the point on the current path segment closest to the current position

            // Vector from p1 to p2
            auto p1 = waypointList[currentWaypointIndex];
            auto p2 = waypointList[currentWaypointIndex + 1];

            sf::Vector2f segment = p2 - p1;
            sf::Vector2f currentToP1 = currentPosition - p1;
            float segmentLength = simplesim::norm(segment);
            // Find the projection point
            float t = simplesim::dot(currentToP1, segment) / (segmentLength * segmentLength);

            // Clamp t to [0, 1] to stay within segment bounds
            t = simplesim::clamp(t, 0.0F, 1.0F);

            // Initial best lookahead point is the projection point
            currentSetpoint = p1 + t * segment;

            // Find the lookahead point along the path
            // If this isn't the first waypoint, the current segment is from the previous waypoint to the current
            // waypoint Otherwise, it's from the first waypoint to the second waypoint
            auto segmentStartIndex = currentWaypointIndex == 0 ? 0 : currentWaypointIndex - 1;

            for (int i = segmentStartIndex; i < waypointList.size(); i++) {
                p1 = waypointList[i];
                p2 = waypointList[i + 1];

                // After the current segment, we only check following segments if their start is within the lookahead
                // distance
                if (i > segmentStartIndex && !simplesim::pointWithinCircle(currentPosition, lookaheadDistance, p1)) {
                    break;
                }

                auto intersections =
                    simplesim::intersectCircleAndLineParameterized(currentPosition, lookaheadDistance, p1, p2);

                if (!intersections.empty()) {
                    auto furthestIntersection = std::max_element(intersections.begin(), intersections.end());
                    currentSetpoint = p1 + *furthestIntersection * (p2 - p1);
                }
            }
        }

        // Update drawables
        lookaheadPoint.setPosition(currentSetpoint);
        lookaheadCircle.setOrigin({lookaheadDistance, lookaheadDistance});
        lookaheadCircle.setRadius(lookaheadDistance);

        // Cascade PID controller
        // Outer loop: position error -> desired velocity
        // Outer loop should be slower than inner loop
        if (tickCount % 2 == 0) {
            positionError = currentSetpoint - currentPosition;
            deltaPositionError = (positionError - lastPositionError) / dt.asSeconds();
            velocityCommand = kp_position * positionError + kd_position * deltaPositionError;
            lastPositionError = positionError;
        }

        // Inner loop: velocity error -> desired acceleration
        velocityError = velocityCommand - currentVelocity;
        deltaVelocityError = (velocityError - lastVelocityError) / dt.asSeconds();
        accelerationCommand = kp_velocity * velocityError + kd_velocity * deltaVelocityError;
        lastVelocityError = velocityError;

        // Clamp acceleration
        float accelerationNorm = simplesim::norm(accelerationCommand);
        if (accelerationNorm > this->options.maxAcceleration) {
            accelerationCommand = accelerationCommand / accelerationNorm * this->options.maxAcceleration;
        }

        // Move to next waypoint if close enough
        if (simplesim::norm(waypointList[currentWaypointIndex] - currentPosition) <= waypointEpsilon &&
            currentWaypointIndex < waypointList.size() - 1) {
            this->currentWaypointIndex++;
            RCLCPP_INFO(this->get_logger(), "Moving to waypoint %i", currentWaypointIndex);
        }
    }

    geometry_msgs::msg::Vector3 velocityCommandMsg;
    velocityCommandMsg.x = velocityCommand.x;
    velocityCommandMsg.y = velocityCommand.y;
    this->velocityCommandPublisher->publish(velocityCommandMsg);

    geometry_msgs::msg::Vector3 accelerationCommandMsg;
    accelerationCommandMsg.x = accelerationCommand.x;
    accelerationCommandMsg.y = accelerationCommand.y;
    this->accelerationCommandPublisher->publish(accelerationCommandMsg);
    tickCount++;
}

bool Controller::reset() {
    auto zero = sf::Vector2f(0.0F, 0.0F);
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
