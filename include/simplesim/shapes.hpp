#pragma once

#include <SFML/Graphics.hpp>

/// @brief Wraps an sf::VertexArray to make adding cross shapes easier
class CrossShapeArray : public sf::Drawable {
   public:
    CrossShapeArray() { vertices.setPrimitiveType(sf::Lines); }

    void addCross(sf::Vector2f position, float size = 12.5f, sf::Color color = sf::Color::Black) {
        // Top left to bottom right
        vertices.append(sf::Vertex({position.x - size / 2, position.y - size / 2}, color));
        vertices.append(sf::Vertex({position.x + size / 2, position.y + size / 2}, color));

        // Bottom left to top right
        vertices.append(sf::Vertex({position.x - size / 2, position.y + size / 2}, color));
        vertices.append(sf::Vertex({position.x + size / 2, position.y - size / 2}, color));
    }

    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const override {
        target.draw(vertices, states);
    }

    void clear() {
        this->vertices.clear();
    }

   private:
    sf::VertexArray vertices;
};