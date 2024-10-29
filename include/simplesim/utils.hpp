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

/// @brief Computes the intersection points between a circle and a line segment, returning the intersections as
/// parameters along the segment
/// @param center center of the circle
/// @param radius radius of the circle
/// @param p1 first point of the line segment
/// @param p2 second point of the line segment
/// @return vector containing exactly 0, 1 or 2 intersection points, as floats in [0, 1] representing where the
/// intersection is along the segment
inline std::vector<float> intersectCircleAndLineParameterized(const sf::Vector2f& center,
                                                              float radius,
                                                              const sf::Vector2f& p1,
                                                              const sf::Vector2f& p2) {
    // Let d be the vector from p1 to p2
    // Let f be the vector from p1 to the circle center c
    // The equation solving for the intersection between a point on p1->p2 and the circle is:
    // (p1 + t * d - c)^2 = r^2
    // This simplifies to a quadratic equation in t:
    // t^2 (d . d) + 2t (f . d) + (f . f - r^2) = 0
    // Below we let a, b, and c be the coefficients of the quadratic equation

    // Compute the quadratic equation coefficients
    sf::Vector2f d = p2 - p1;
    sf::Vector2f f = p1 - center;

    float a = dot(d, d);
    float b = 2 * dot(f, d);
    float c = dot(f, f) - radius * radius;

    float discriminant = b * b - 4 * a * c;

    std::vector<float> result(3);

    if (discriminant >= 0) {
        // If discriminant is close enough to zero, there's only one solution
        if (std::abs(discriminant) < 1e-6) {
            float t = -b / (2 * a);
            result.push_back(t);
        } else {
            // Compute the two solutions
            discriminant = sqrt(discriminant);

            // Positive solution
            float t1 = (-b - discriminant) / (2 * a);
            // Negative solution
            float t2 = (-b + discriminant) / (2 * a);

            // Even though the solutions are for the line, we need to check if they're within the segment
            // Thus, even a line that intersects the circle may not have any intersection points if the segment doesn't
            // contain them
            if (t1 >= 0 && t1 <= 1) {
                result.push_back(t1);
            }
            if (t2 >= 0 && t2 <= 1) {
                result.push_back(t2);
            }
        }
    }
    return result;
}

inline bool pointWithinCircle(const sf::Vector2f& center, float radius, const sf::Vector2f& point) {
    return norm(center - point) <= radius;
}

}  // namespace simplesim