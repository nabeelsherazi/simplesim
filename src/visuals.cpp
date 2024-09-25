#include "simplesim/visuals.hpp"

void Visuals::update(const std::vector<const Renderable*>& renderables) {
    this->drawables.clear();
    for (const auto& renderable : renderables) {
        auto entityDrawables = renderable->getDrawables();
        this->drawables.insert(drawables.end(), entityDrawables.begin(), entityDrawables.end());
    }
}

void Visuals::render(sf::RenderWindow& window) {
    for (auto drawable : drawables) {
        window.draw(*drawable);
    }
}