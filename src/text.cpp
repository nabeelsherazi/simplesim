#include <vector>
#include <utility>
#include <filesystem>
#include <algorithm>
#include <range/v3/view/join.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/concat.hpp>
#include <SFML/Graphics.hpp>

#include "simplesim/text.hpp"

DebugTextConsole::DebugTextConsole(float textSizePixels, float padding, sf::Time updateGranularity) : textSizePixels(textSizePixels), padding(padding), updateGranularity(updateGranularity) {};

bool DebugTextConsole::loadFont(std::filesystem::path filename) {
    return this->font.loadFromFile(filename);
}

void DebugTextConsole::setConsoleTextSize(float pixels, float padding = 0.0) {
    this->textSizePixels = pixels;
    this->padding = padding;
}

int DebugTextConsole::addFixedTextLine(std::string initialText) {
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

void DebugTextConsole::updateFixedTextLine(int index, std::string newText, sf::Color newColor = sf::Color::Black) {
    this->fixedTextLines[index].setString(newText);
    this->fixedTextLines[index].setFillColor(newColor);
}

void DebugTextConsole::addTemporaryTextLine(std::string text, sf::Time duration, sf::Color color = sf::Color::Black) {
    sf::Text textLine;
    textLine.setFont(this->font);
    textLine.setCharacterSize(this->textSizePixels);
    textLine.setFillColor(color);
    textLine.setString(text);

    this->temporaryTextLines.push_back(std::make_pair(textLine, duration));
}

void DebugTextConsole::updateTextLinePositions() {
    for (size_t i = 0; i < this->fixedTextLines.size(); i++) {
        this->fixedTextLines[i].setPosition(0, i * (textSizePixels + padding));
    }
    for (size_t i = 0; i < this->temporaryTextLines.size(); i++) {
        this->temporaryTextLines[i].first.setPosition(0, endOfFixedTextLineHeightPixels + padding + (i * (textSizePixels + padding)));
    }
}

void DebugTextConsole::tick(sf::Time dt) {

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

auto DebugTextConsole::drawables() {
    // Create a range for temporaryTextLines that extracts sf::Text from each pair
    auto tempTextRange = temporaryTextLines | ranges::views::transform([](auto& pair) -> sf::Text& {return pair.first;});

    // Join fixedTextLines and tempTextRange into a single range
    return ranges::views::concat(fixedTextLines, tempTextRange);
}