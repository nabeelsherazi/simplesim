#pragma once

#include <filesystem>
#include <SFML/Graphics.hpp>
#include <rclcpp/rclcpp.hpp>

// Executable location
static const auto executableLocation = std::filesystem::canonical("/proc/self/exe").parent_path();