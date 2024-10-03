#pragma once

#include <algorithm>
#include <filesystem>
#include <utility>
#include <vector>

#include <SFML/Graphics.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/transform.hpp>

class DebugTextConsole {
   public:
    DebugTextConsole(float textSizePixels = 12.0,
                     float padding = 3.0,
                     sf::Time updateGranularity = sf::milliseconds(60));

    /// @brief Load a font from a file
    /// @param filename the path to the font file
    /// @return true if the font was loaded successfully
    bool loadFont(std::filesystem::path filename);

    /// @brief Set the size of the text in the console
    /// @param pixels The size of the text in pixels
    /// @param padding The padding between lines in pixels
    void setConsoleTextSize(float pixels, float padding = 0.0);

    /// @brief Add a persistent text line to the console
    /// @param initialText The initial text to display
    /// @return A handle that can be used to update the text line
    /// @see updateFixedTextLine
    int addFixedTextLine(std::string initialText);

    /// @brief Update a fixed text line
    /// @param index The handle of a text line to update
    /// @param newText The new text to display
    /// @param newColor The new color to display the text
    void updateFixedTextLine(int index, std::string newText, sf::Color newColor = sf::Color::Black);

    /// @brief Add a text line to the console that updates itself
    /// @param updater A function that returns the text to display
    /// @param color The color to display the text
    void addSelfUpdatingTextLine(std::function<std::string()> updater, sf::Color color = sf::Color::Black);

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
    std::vector<std::function<std::string()>> selfUpdatingTextLines;
    std::vector<std::pair<sf::Text, sf::Time>> temporaryTextLines;
};
