#ifndef INPUT_DIALOG_H
#define INPUT_DIALOG_H

#include <SFML/Graphics.hpp>
#include <vector>
#include <string>
#include "SimConfig.h"

/**
 * @brief Modal startup dialog that lets the user enter all five
 *        SimConfig parameters before the main GUI launches.
 *
 * Features:
 *   - Five text input fields (Speed, TurnRate, Duration, GPS Noise, Mass)
 *   - TAB / arrow keys to switch fields, click to focus
 *   - Inline validation: invalid values display in red with a tooltip
 *   - START button to confirm and close
 *   - CANCEL button to abort
 *
 * Returns true if the user clicked START with valid values, false if
 * the user closed the window or clicked CANCEL.
 */
class InputDialog {
public:
    InputDialog();

    /**
     * Shows the dialog modally. Blocks until the user starts or cancels.
     * On success, fills @p outConfig with the entered values.
     * Returns true on START, false on CANCEL/close.
     */
    bool show(SimConfig& outConfig);

private:
    struct Field {
        std::string label;
        std::string suffix;       // " m/s", " kg", etc.
        std::string text;         // current text content
        double      minVal;
        double      maxVal;
        std::string error;        // empty = valid
        sf::FloatRect rect;       // input box rect (for hit testing)
    };

    sf::RenderWindow window;
    sf::Font         font;
    bool             fontLoaded;

    std::vector<Field> fields;
    int focusedField;
    std::string globalError;

    sf::FloatRect btnStartRect;
    sf::FloatRect btnCancelRect;

    bool startClicked;
    bool cancelClicked;

    static const unsigned WIN_W = 560;
    static const unsigned WIN_H = 540;

    void buildFields();
    void layoutWidgets();
    void drawAll();
    void drawText(const std::string& s, float x, float y, unsigned size,
                  sf::Color color);
    void drawField(int idx);
    void drawButton(const sf::FloatRect& r, const std::string& label,
                    sf::Color base);
    bool pointInRect(float x, float y, const sf::FloatRect& r) const;
    void onClick(float x, float y);
    void onTextEntered(char32_t ch);
    void onKeyPressed(sf::Keyboard::Key key);
    bool validateField(int idx);
    bool validateAll(SimConfig& outConfig);
};

#endif // INPUT_DIALOG_H
