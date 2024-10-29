#include "simplesim/text.hpp"

#include <algorithm>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include <SFML/Graphics.hpp>

DebugTextConsole::DebugTextConsole(float textSizePixels, float padding, unsigned int updateFrequencyHz)
    : textSizePixels(textSizePixels), padding(padding), updateGranularity(sf::seconds(1 / updateFrequencyHz)) {};

bool DebugTextConsole::loadFont(const std::filesystem::path& filename) {
    return this->font.loadFromFile(filename);
}

void DebugTextConsole::setConsoleTextSize(float pixels, float padding) {
    this->textSizePixels = pixels;
    this->padding = padding;
}

int DebugTextConsole::addFixedTextLine(std::string initialText, sf::Color color) {
    sf::Text textLine;
    textLine.setFont(this->font);
    textLine.setCharacterSize(this->textSizePixels);
    textLine.setFillColor(color);
    textLine.setString(initialText);

    int insertedToIndex = this->fixedTextLines.size();
    this->fixedTextLines.push_back(textLine);
    this->endOfFixedTextLineHeightPixels += textLine.getLocalBounds().height + this->padding;

    return insertedToIndex;
}

void DebugTextConsole::addSelfUpdatingTextLine(const std::function<std::string()>& updater, sf::Color color) {
    auto initialText = updater();
    int handle = this->addFixedTextLine(initialText);
    this->selfUpdatingTextLines.emplace_back(handle, updater);
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

    this->temporaryTextLines.emplace_back(textLine, duration);
}

void DebugTextConsole::updateTextLinePositions() {
    for (int i = 0; i < this->fixedTextLines.size(); i++) {
        this->fixedTextLines[i].setPosition(0, i * (textSizePixels + padding));
    }
    for (int i = 0; i < this->temporaryTextLines.size(); i++) {
        this->temporaryTextLines[i].first.setPosition(
            0, endOfFixedTextLineHeightPixels + padding + (i * (textSizePixels + padding)));
    }
}

void DebugTextConsole::tick(sf::Time dt) {
    this->timeSinceLastUpdate += dt;

    if (timeSinceLastUpdate >= updateGranularity) {
        // Remove expired temporary text lines
        this->temporaryTextLines.erase(std::remove_if(temporaryTextLines.begin(), temporaryTextLines.end(),
                                                      [this](std::pair<sf::Text, sf::Time>& temporaryLine) {
                                                          temporaryLine.second -= this->timeSinceLastUpdate;
                                                          return temporaryLine.second <= sf::Time::Zero;
                                                      }),
                                       temporaryTextLines.end());

        // Recalculate the vertical position of all text lines
        this->updateTextLinePositions();

        // Update self-updating text lines
        for (auto& [handle, updater] : this->selfUpdatingTextLines) {
            this->updateFixedTextLine(handle, updater());
        }

        this->timeSinceLastUpdate = sf::Time::Zero;
    }
}