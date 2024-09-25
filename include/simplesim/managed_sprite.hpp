#pragma once

#include <SFML/Graphics.hpp>
#include <filesystem>

class ManagedSprite : public sf::Sprite {
   public:
    ManagedSprite() = default;

    /// @brief Move constructor
    /// @param other
    ManagedSprite(ManagedSprite&& other) noexcept;

    /// @brief Move assignment operator
    /// @param other
    /// @return
    ManagedSprite& operator=(ManagedSprite&& other) noexcept;

    ManagedSprite(const ManagedSprite&) = delete;
    ManagedSprite& operator=(const ManagedSprite&) = delete;

    bool load(const std::filesystem::path& filename);

    void scaleToSize(const sf::Vector2f& size);

    void scaleToSize(float size);

    void setOriginRelative(const sf::Vector2f& relativeOrigin);

    sf::Texture texture;
    sf::Vector2f scaledSize = {1.0f, 1.0f};
    sf::Vector2f relativeOrigin = {0.0f, 0.0f};
};