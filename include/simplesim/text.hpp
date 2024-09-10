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

    bool loadFont(std::filesystem::path filename) {
        return this->font.loadFromFile(filename);
    }

    void setConsoleTextSize(float pixels, float padding = 0.0) {
        this->textSizePixels = pixels;
        this->padding = padding;
    }

    int addFixedTextLine(std::string initialText) {
        sf::Text textLine;
        textLine.setFont(this->font);
        textLine.setCharacterSize(this->textSizePixels);
        textLine.setFillColor(sf::Color::Black);
        textLine.setString(initialText);

        int insertedToIndex = this->fixedTextLines.size();
        this->fixedTextLines.push_back(textLine);
        this->endOfFixedTextLineHeightPixels = this->textSizePixels + this->padding;

        return insertedToIndex;
    }

    void updateFixedTextLine(int index, std::string newText, sf::Color newColor = sf::Color::Black) {
        this->fixedTextLines[index].setString(newText);
        this->fixedTextLines[index].setFillColor(newColor);
    }

    /// @brief Add a text line to the console that will disappear
    /// @param text The text to display
    /// @param duration How long to display it for
    /// @param color The color to display the text
    void addTemporaryTextLine(std::string text, sf::Time duration, sf::Color color = sf::Color::Black) {
        sf::Text textLine;
        textLine.setFont(this->font);
        textLine.setCharacterSize(this->textSizePixels);
        textLine.setFillColor(color);
        textLine.setString(text);

        this->temporaryTextLines.push_back(std::make_pair(textLine, duration));
    }

    /// @brief Update vertical position of temporary console entries
    void updateTextLinePositions() {
        for (size_t i = 0; i < this->fixedTextLines.size(); i++) {
            this->fixedTextLines[i].setPosition(0, i * (textSizePixels + padding));
        }
        for (size_t i = 0; i < this->temporaryTextLines.size(); i++) {
            this->temporaryTextLines[i].first.setPosition(0, endOfFixedTextLineHeightPixels + padding + (i * (textSizePixels + padding)));
        }
    }

    void tick(sf::Time dt) {

        this->timeSinceLastUpdate += dt;

        if (timeSinceLastUpdate >= updateGranularity) {
            this->temporaryTextLines.erase(
                std::remove_if(temporaryTextLines.begin(), temporaryTextLines.end(), [dt](std::pair<sf::Text, sf::Time>& temporaryLine) {
                temporaryLine.second -= dt;
                return temporaryLine.second <= sf::Time::Zero;  
                }),
                temporaryTextLines.end()
            );

            this->updateTextLinePositions();

            this->timeSinceLastUpdate = sf::Time::Zero;
        }

    }

    auto drawables() {
        // Create a range for temporaryTextLines that extracts sf::Text from each pair
        auto tempTextRange = temporaryTextLines | ranges::views::transform([](auto& pair) -> sf::Text& {
            return pair.first;  // Ensure returning by reference
        });

        // Join fixedTextLines and tempTextRange into a single range
        return ranges::views::concat(fixedTextLines, tempTextRange);
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