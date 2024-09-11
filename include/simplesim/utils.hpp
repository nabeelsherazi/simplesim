#pragma once

#include <SFML/Graphics.hpp>
#include <cmath>

float norm(sf::Vector2f vec) {
    return powf(vec.x * vec.x + vec.y * vec.y, 0.5f);
}