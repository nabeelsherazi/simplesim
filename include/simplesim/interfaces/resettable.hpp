#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

/// @brief Interface for ROS nodes that can be reset to their initial state with a ROS service call
class Resettable : virtual public rclcpp::Node {
   public:
    /// @brief Constructor
    Resettable(const std::string& name) : Node(name) {
        this->resetService = this->create_service<std_srvs::srv::Trigger>(
            "~/reset", [&](__attribute__((unused)) const std_srvs::srv::Trigger::Request::SharedPtr request,
                           std_srvs::srv::Trigger::Response::SharedPtr response) {
                auto allSucceeded = this->reset();
                for (const auto& resettable : resetChain) {
                    allSucceeded &= resettable->reset();
                };
                response->success = allSucceeded;
            });
    };

    /// @brief Add a resettable to the reset chain
    void AddToResetChain(Resettable* resettable) { resetChain.push_back(resettable); }

    /// @brief Add a vector of resettables to the reset chain
    void AddToResetChain(std::vector<Resettable*> resettables) {
        resetChain.insert(resetChain.end(), resettables.begin(), resettables.end());
    }

    /// @brief Reset the node to its initial state
    /// @return true if the reset was successful
    virtual bool reset() = 0;

    /// @brief Destructor
    virtual ~Resettable() = default;

   private:
    std::vector<Resettable*> resetChain;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetService;
};