#pragma once

#include <vector>

#include <SFML/Graphics.hpp>

class Renderable {
   public:
    virtual std::vector<const sf::Drawable*> getDrawables() const = 0;
    virtual ~Renderable() = default;
};
