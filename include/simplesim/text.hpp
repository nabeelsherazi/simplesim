#pragma once

#include <SFML/Graphics.hpp>
#include <algorithm>
#include <filesystem>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/transform.hpp>
#include <utility>
#include <vector>

#define OPEN_SANS_REGULAR "/usr/share/fonts/truetype/open-sans/OpenSans-Regular.ttf"

class DebugTextConsole {
   public:
    DebugTextConsole(float textSizePixels = 12.0,
                     float padding = 3.0,
                     sf::Time updateGranularity = sf::milliseconds(60));

    bool loadFont(std::filesystem::path filename);

    void setConsoleTextSize(float pixels, float padding = 0.0);

    int addFixedTextLine(std::string initialText);

    void updateFixedTextLine(int index, std::string newText, sf::Color newColor = sf::Color::Black);
    /// @brief Add a text line to the console that will disappear
    /// @param text The text to display
    /// @param duration How long to display it for
    /// @param color The color to display the text
    void addTemporaryTextLine(std::string text, sf::Time duration, sf::Color color = sf::Color::Black);

    /// @brief Update vertical position of temporary console entries
    void updateTextLinePositions();

    void tick(sf::Time dt);

    /// @brief Get an iterable view of all sf::Text objects
    /// @return ranges::view of all sf::Text objects
    auto drawables() {
        return ranges::views::concat(
            fixedTextLines,
            temporaryTextLines | ranges::views::transform([](auto& p) -> sf::Text& { return p.first; }));
    }

   private:
    sf::Font font;
    float textSizePixels;
    float endOfFixedTextLineHeightPixels;
    float padding;
    sf::Time updateGranularity;
    sf::Time timeSinceLastUpdate;
    std::vector<sf::Text> fixedTextLines;
    std::vector<std::pair<sf::Text, sf::Time>> temporaryTextLines;
};