#pragma once

#include <SFML/Graphics.hpp>
#include <cmath>
#include <type_traits>
#include <vector>

inline float norm(sf::Vector2f vec) {
    return powf(vec.x * vec.x + vec.y * vec.y, 0.5f);
}

inline float clamp(float val, float min, float max) {
    return std::max(min, std::min(val, max));
}

inline constexpr unsigned int hash(const char* s, int off = 0) {
    return !s[off] ? 5381 : (hash(s, off + 1) * 33) ^ s[off];
}