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

inline constexpr unsigned int hash(const char *s, int off = 0) {                        
    return !s[off] ? 5381 : (hash(s, off+1)*33) ^ s[off];                           
}

template <typename T>
class JoinedVectors {
    std::vector<const std::vector<T>*> vecs;

public:
    // Templated constructor to accept a parameter pack of vectors
    template<typename... Vecs>
    JoinedVectors(const Vecs&... vectors) : vecs{ &vectors... } {
        static_assert((std::is_same_v<Vecs, std::vector<T>> && ...), "All arguments must be std::vector<T>");
    }

    class iterator {
        const std::vector<const std::vector<T>*>& vecs;
        size_t vec_index;
        typename std::vector<T>::const_iterator current_iter;
        typename std::vector<T>::const_iterator current_end;

    public:
        iterator(const std::vector<const std::vector<T>*>& vecs_, bool is_end = false)
            : vecs(vecs_), vec_index(0)
        {
            if (is_end || vecs.empty()) {
                vec_index = vecs.size();
            } else {
                current_iter = vecs[vec_index]->begin();
                current_end = vecs[vec_index]->end();
                advance_past_empty_vectors();
            }
        }

        void advance_past_empty_vectors() {
            while (current_iter == current_end) {
                ++vec_index;
                if (vec_index >= vecs.size()) break;
                current_iter = vecs[vec_index]->begin();
                current_end = vecs[vec_index]->end();
            }
        }

        iterator& operator++() {
            if (vec_index >= vecs.size()) return *this;
            ++current_iter;
            if (current_iter == current_end) {
                ++vec_index;
                if (vec_index < vecs.size()) {
                    current_iter = vecs[vec_index]->begin();
                    current_end = vecs[vec_index]->end();
                    advance_past_empty_vectors();
                }
            }
            return *this;
        }

        const T& operator*() const {
            return *current_iter;
        }

        bool operator==(const iterator& other) const {
            if (vec_index != other.vec_index) return false;
            if (vec_index == vecs.size()) return true;
            return current_iter == other.current_iter;
        }

        bool operator!=(const iterator& other) const {
            return !(*this == other);
        }
    };

    iterator begin() const {
        return iterator(vecs, false);
    }

    iterator end() const {
        return iterator(vecs, true);
    }
};

template<typename T, typename... Vecs>
JoinedVectors<T> join(const Vecs&... vecs) {
    static_assert((std::is_same_v<Vecs, std::vector<T>> && ...), "All arguments must be std::vector<T>");
    return JoinedVectors<T>(vecs...);
}