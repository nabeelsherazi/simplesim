#pragma once

#include <SFML/Graphics.hpp>

class XShape : public sf::Drawable {
public:
    // Constructor takes the position (center of the "X") and size (length of the arms)
    XShape(sf::Vector2f position, float size)
        : m_position(position), m_size(size)
    {
        createXShape();
    }

private:
    sf::Vector2f m_position;  // Center of the "X"
    float m_size;             // Length of the arms of the "X"
    sf::VertexArray m_vertices; // VertexArray to hold the two lines

    // This function will be called to draw the "X" shape
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const override
    {
        target.draw(m_vertices, states);  // Draw the lines
    }

    // Create the two diagonal lines that form the "X"
    void createXShape()
    {
        m_vertices.setPrimitiveType(sf::Lines);
        m_vertices.resize(4);  // 4 vertices for two lines

        // First diagonal: top-left to bottom-right
        m_vertices[0].position = sf::Vector2f(m_position.x - m_size / 2, m_position.y - m_size / 2);
        m_vertices[0].color = sf::Color::Black;
        m_vertices[1].position = sf::Vector2f(m_position.x + m_size / 2, m_position.y + m_size / 2);
        m_vertices[1].color = sf::Color::Black;

        // Second diagonal: bottom-left to top-right
        m_vertices[2].position = sf::Vector2f(m_position.x - m_size / 2, m_position.y + m_size / 2);
        m_vertices[2].color = sf::Color::Black;
        m_vertices[3].position = sf::Vector2f(m_position.x + m_size / 2, m_position.y - m_size / 2);
        m_vertices[3].color = sf::Color::Black;
    }
};
