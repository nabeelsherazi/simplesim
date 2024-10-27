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

// Function to compute intersections between a circle and a line segment
inline bool getCircleSegmentIntersections(const sf::Vector2f& center, float radius,
                                               const sf::Vector2f& p1, const sf::Vector2f& p2,
                                               sf::Vector2f& intersection1, sf::Vector2f& intersection2,
                                               int& numIntersections) {
    // Compute the quadratic equation coefficients
    sf::Vector2f d = p2 - p1;
    sf::Vector2f f = p1 - center;

    float a = simplesim::dot(d, d);
    float b = 2 * simplesim::dot(f, d);
    float c = simplesim::dot(f, f) - radius * radius;

    float discriminant = b * b - 4 * a * c;

    if (discriminant < 0) {
        // No intersection
        numIntersections = 0;
        return false;
    } else {
        discriminant = sqrt(discriminant);

        float t1 = (-b - discriminant) / (2 * a);
        float t2 = (-b + discriminant) / (2 * a);

        numIntersections = 0;

        if (t1 >= 0 && t1 <= 1) {
            intersection1 = p1 + t1 * d;
            numIntersections++;
        }
        if (t2 >= 0 && t2 <= 1) {
            if (numIntersections == 0) {
                intersection1 = p1 + t2 * d;
            } else {
                intersection2 = p1 + t2 * d;
            }
            numIntersections++;
        }

        return numIntersections > 0;
    }
}

}  // namespace simplesim