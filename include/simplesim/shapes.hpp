#pragma once

#include <SFML/Graphics.hpp>

class CrossShape : public sf::Drawable {
public:
    // Constructor takes the position (center of the "X") and size (length of the arms)
    CrossShape(sf::Vector2f position, float size)
        : position(position), size(size)
    {
        vertices.setPrimitiveType(sf::Lines);
        // 4 vertices for two lines
        vertices.resize(4);

        // Top left to bottom right
        vertices[0].position = sf::Vector2f(position.x - size / 2, position.y - size / 2);
        vertices[0].color = sf::Color::Black;
        vertices[1].position = sf::Vector2f(position.x + size / 2, position.y + size / 2);
        vertices[1].color = sf::Color::Black;

        // Bottom left to top right
        vertices[2].position = sf::Vector2f(position.x - size / 2, position.y + size / 2);
        vertices[2].color = sf::Color::Black;
        vertices[3].position = sf::Vector2f(position.x + size / 2, position.y - size / 2);
        vertices[3].color = sf::Color::Black;
    }

    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const override
    {
        target.draw(vertices, states);
    }

private:
    sf::Vector2f position;
    float size;
    sf::VertexArray vertices;
};