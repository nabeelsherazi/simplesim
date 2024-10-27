#pragma once

#include <cmath>
#include <vector>

#include <SFML/System/Vector2.hpp>

namespace simplesim {

inline float norm(sf::Vector2f vec) {
    return std::sqrt(vec.x * vec.x + vec.y * vec.y);
}

inline sf::Vector2f normalize(sf::Vector2f vec) {
    return vec / norm(vec);
}

inline float dot(sf::Vector2f v1, sf::Vector2f v2) {
    return v1.x * v2.x + v1.y * v2.y;
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

}  // namespace simplesim