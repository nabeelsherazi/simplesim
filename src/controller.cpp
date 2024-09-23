#include <SFML/Graphics.hpp>
#include <rclcpp/rclcpp.hpp>
#include "simplesim/controller.hpp"
#include "simplesim/utils.hpp"

Controller::Controller(std::string node_name, sf::Vector2f initialPosition) :
    Node(node_name),
    waypointEpsilon(50.0f),
    kp_position(2.0f),
    kd_position(-0.05f),
    kp_velocity(0.5f),
    kd_velocity(-0.05f),
    maxAcceleration(10.0f) {
    this->positionSubscriber = this->create_subscription<geometry_msgs::msg::Vector3>("/simplesim/drone/position", 10, [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
        this->currentPosition = sf::Vector2f(msg->x, msg->y);
    });
    this->waypointSubscriber = this->create_subscription<geometry_msgs::msg::Vector3>("/simplesim/drone/waypoint", 10, [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
        this->addWaypoint(sf::Vector2f(msg->x, msg->y));
    });
    this->accelerationCommandPublisher = this->create_publisher<geometry_msgs::msg::Vector3>("/simplesim/drone/accel_cmd", 10);
    this->velocityCommandPublisher = this->create_publisher<geometry_msgs::msg::Vector3>("/simplesim/drone/velocity_cmd", 10);
    this->currentPosition = initialPosition;

    this->declare_parameter("kp_position", 0.0f);
    parameterSubscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
    parameterCallbackHandle = parameterSubscriber->add_parameter_callback("kp_position", std::bind(&Controller::parameterCallback, this, _1));
    
};

void Controller::parameterCallback(const rclcpp::Parameter& param) {
    switch (hash(param.get_name().c_str())) {
        case hash("kp_position"):
            this->kp_position = static_cast<float>(param.as_double());
            break;
        case hash("position_kd"):
            break;
        default:
            break;
    };

    RCLCPP_INFO(this->get_logger(), "Parameter %s updated", param.get_name());
}

void Controller::addWaypoint(sf::Vector2f wpt) {
    this->waypointList.push_back(wpt);
}

void Controller::tick(sf::Time dt) {
    if (currentWaypointIndex < waypointList.size()) {
        // Calculate velocity command
        auto nextWaypointPosition = waypointList[currentWaypointIndex];
        // Cascade PID controller
        // Outer loop: position error -> desired velocity
        positionError = nextWaypointPosition - currentPosition;
        deltaError = (positionError - lastPositionError) / dt.asSeconds();
        velocityCommand = kp_position * positionError + kd_position * deltaError;
        lastPositionError = positionError;

        // Inner loop: velocity error -> desired acceleration
        // auto velocityError = velocityCommand - currentVelocity;
        // auto commandedAcceleration = kp_velocity * velocityError + kd_velocity * (velocityError - lastVelocityError) / dt.asSeconds();
        // // Clamp acceleration
        // auto accelerationNorm = norm(commandedAcceleration);
        // if (accelerationNorm > maxAcceleration) {
        //     commandedAcceleration = commandedAcceleration / accelerationNorm * maxAcceleration;
        // }

        // // Move based on commanded acceleration
        // geometry_msgs::msg::Vector3 msg;
        // msg.x = commandedAcceleration.x;
        // msg.y = commandedAcceleration.y;
        // this->accelerationCommandPublisher->publish(msg);

        geometry_msgs::msg::Vector3 msg;
        msg.x = velocityCommand.x;
        msg.y = velocityCommand.y;
        this->velocityCommandPublisher->publish(msg);


        // Move to next waypoint if we're close enough to the current one
        if (norm(nextWaypointPosition - currentPosition) <= waypointEpsilon && currentWaypointIndex != waypointList.size() -1 ) {
            this->currentWaypointIndex++;
        }
    }
}

void Controller::reset() {
    this->currentWaypointIndex = 0;
    this->waypointList.clear();
    this->lastPositionError = sf::Vector2f(0.0f, 0.0f);
    this->lastVelocityError = sf::Vector2f(0.0f, 0.0f);
}