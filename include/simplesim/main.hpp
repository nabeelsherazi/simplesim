#pragma once

#include <filesystem>
#include <SFML/Graphics.hpp>
#include <rclcpp/rclcpp.hpp>

// Executable location
static const auto executableLocation = std::filesystem::canonical("/proc/self/exe").parent_path();

class ManagedSprite : public sf::Sprite {

public:
    ManagedSprite() {};

    bool load(const std::filesystem::path& filename) {
        if (!this->texture.loadFromFile(filename))
        {
            RCLCPP_ERROR(rclcpp::get_logger(PROJECT_NAME), "Failed to load sprite %s", filename.string().c_str());
            return false;
        }
        this->texture.setSmooth(true);
        this->setTexture(this->texture);
        this->localBounds = this->getLocalBounds();
        this->setOrigin(this->localBounds.width / 2, this->localBounds.height / 1.5);
        return true;
    }

    void scaleToSize(float size_x, float size_y) {
        this->setScale(size_x / this->localBounds.width, size_y / this->localBounds.height);
    }

    void scaleToSize (float size) {
        this->scaleToSize(size, size);
    }
    

private:
    sf::Texture texture;
    sf::FloatRect localBounds;
};