#pragma once

#include <vector>
#include <utility>
#include <filesystem>
#include <algorithm>
#include <range/v3/view/join.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/concat.hpp>
#include <SFML/Graphics.hpp>

#define OPEN_SANS_REGULAR "/usr/share/fonts/truetype/open-sans/OpenSans-Regular.ttf"

class DebugTextConsole {
public:
    DebugTextConsole(float textSizePixels = 12.0, float padding = 3.0, sf::Time updateGranularity = sf::milliseconds(60)) : textSizePixels(textSizePixels), padding(padding), updateGranularity(updateGranularity) {};

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

    auto drawables();

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