#include "simplesim/text.hpp"

#include <algorithm>
#include <filesystem>
#include <utility>
#include <vector>
#include <string>

#include <SFML/Graphics.hpp>

DebugTextConsole::DebugTextConsole(float textSizePixels, float padding, sf::Time updateGranularity)
    : textSizePixels(textSizePixels), padding(padding), updateGranularity(updateGranularity) {};

bool DebugTextConsole::loadFont(std::filesystem::path filename) {
    return this->font.loadFromFile(filename);
}

void DebugTextConsole::setConsoleTextSize(float pixels, float padding) {
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
    this->endOfFixedTextLineHeightPixels += textLine.getLocalBounds().height + this->padding;

    return insertedToIndex;
}

void DebugTextConsole::updateFixedTextLine(int index, std::string newText, sf::Color newColor) {
    this->fixedTextLines[index].setString(newText);
    this->fixedTextLines[index].setFillColor(newColor);
}

void DebugTextConsole::addTemporaryTextLine(std::string text, sf::Time duration, sf::Color color) {
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
        this->temporaryTextLines[i].first.setPosition(
            0, endOfFixedTextLineHeightPixels + padding + (i * (textSizePixels + padding)));
    }
}

void DebugTextConsole::tick(sf::Time dt) {
    this->timeSinceLastUpdate += dt;

    if (timeSinceLastUpdate >= updateGranularity) {
        this->temporaryTextLines.erase(std::remove_if(temporaryTextLines.begin(), temporaryTextLines.end(),
                                                      [this](std::pair<sf::Text, sf::Time>& temporaryLine) {
                                                          temporaryLine.second -= this->timeSinceLastUpdate;
                                                          return temporaryLine.second <= sf::Time::Zero;
                                                      }),
                                       temporaryTextLines.end());

        this->updateTextLinePositions();

        this->timeSinceLastUpdate = sf::Time::Zero;
    }
}