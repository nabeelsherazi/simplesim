#include "simplesim/managed_sprite.hpp"

#include <filesystem>
#include <utility>

#include <SFML/Graphics.hpp>

ManagedSprite::ManagedSprite(ManagedSprite&& other) noexcept
    : sf::Sprite(std::move(other)),
      texture(std::move(other.texture)),
      scaledSize(other.scaledSize),
      relativeOrigin(other.relativeOrigin) {}

ManagedSprite& ManagedSprite::operator=(ManagedSprite&& other) noexcept {
    if (this != &other) {
        sf::Sprite::operator=(std::move(other));
        texture = std::move(other.texture);
        scaledSize = other.scaledSize;
        relativeOrigin = other.relativeOrigin;
        this->setTexture(this->texture);
    }
    return *this;
}

bool ManagedSprite::load(const std::filesystem::path& filename) {
    if (!this->texture.loadFromFile(filename)) {
        return false;
    }
    this->texture.setSmooth(true);
    this->setTexture(this->texture);
    return true;
}

void ManagedSprite::scaleToSize(const sf::Vector2f& size) {
    this->scaledSize = size;
    auto localBounds = this->getLocalBounds();
    this->setScale(size.x / localBounds.width, size.y / localBounds.height);
}

void ManagedSprite::scaleToSize(float size) {
    this->scaleToSize({size, size});
}

void ManagedSprite::setOriginRelative(const sf::Vector2f& relativeOrigin) {
    this->relativeOrigin = relativeOrigin;
    auto localBounds = this->getLocalBounds();
    this->setOrigin(localBounds.width * relativeOrigin.x, localBounds.height * relativeOrigin.y);
}