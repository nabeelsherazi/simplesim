#pragma once

#include <SFML/Graphics.hpp>
#include <cmath>
#include <type_traits>
#include <vector>

namespace simplesim {

inline float norm(sf::Vector2f vec) {
    return std::sqrt(vec.x * vec.x + vec.y * vec.y);
}

inline float clamp(float val, float min, float max) {
    return std::max(min, std::min(val, max));
}

inline constexpr unsigned int hash(const char* s, int off = 0) {
    return !s[off] ? 5381 : (hash(s, off + 1) * 33) ^ s[off];
}

inline sf::Vector2f squareComponents(sf::Vector2f& vec) {
    return {vec.x * vec.x, vec.y * vec.y};
}

}