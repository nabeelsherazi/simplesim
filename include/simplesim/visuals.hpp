#pragma once

#include <vector>

#include <SFML/Graphics.hpp>

#include "simplesim/renderable.hpp"

/// @brief Renderer
class Visuals {
   public:
    void update(const std::vector<const Renderable*>& renderables);

    void render(sf::RenderWindow& window);

   private:
    std::vector<const sf::Drawable*> drawables;
};