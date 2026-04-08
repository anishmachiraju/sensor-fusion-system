#include "InputDialog.h"
#include <sstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <algorithm>

InputDialog::InputDialog()
    : window(sf::VideoMode({WIN_W, WIN_H}),
             "Sensor Fusion - Configure Simulation",
             sf::Style::Titlebar | sf::Style::Close),
      fontLoaded(false),
      focusedField(0),
      startClicked(false),
      cancelClicked(false)
{
    window.setFramerateLimit(60);

    const char* fontPaths[] = {
        "/System/Library/Fonts/Helvetica.ttc",
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/Library/Fonts/Arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "C:/Windows/Fonts/arial.ttf",
        "arial.ttf",
        "DejaVuSans.ttf"
    };
    for (const char* p : fontPaths) {
        if (font.openFromFile(p)) {
            fontLoaded = true;
            break;
        }
    }

    buildFields();
    layoutWidgets();
}

void InputDialog::buildFields() {
    fields.clear();

    Field f;

    f.label  = "Vehicle Speed";
    f.suffix = " m/s";
    f.text   = "5.0";
    f.minVal = SimConfig::MIN_SPEED;
    f.maxVal = SimConfig::MAX_SPEED;
    fields.push_back(f);

    f.label  = "Turn Rate";
    f.suffix = " deg/s";
    f.text   = "10.0";
    f.minVal = SimConfig::MIN_TURN;
    f.maxVal = SimConfig::MAX_TURN;
    fields.push_back(f);

    f.label  = "Duration (0 = infinite)";
    f.suffix = " s";
    f.text   = "0";
    f.minVal = SimConfig::MIN_DURATION;
    f.maxVal = SimConfig::MAX_DURATION;
    fields.push_back(f);

    f.label  = "GPS Noise Sigma";
    f.suffix = " m";
    f.text   = "1.5";
    f.minVal = SimConfig::MIN_GPS_NOISE;
    f.maxVal = SimConfig::MAX_GPS_NOISE;
    fields.push_back(f);

    f.label  = "Vehicle Mass";
    f.suffix = " kg";
    f.text   = "1000";
    f.minVal = SimConfig::MIN_MASS;
    f.maxVal = SimConfig::MAX_MASS;
    fields.push_back(f);
}

void InputDialog::layoutWidgets() {
    const float startY  = 90.f;
    const float fieldH  = 60.f;
    const float boxX    = 230.f;
    const float boxW    = 280.f;
    const float boxH    = 32.f;

    for (size_t i = 0; i < fields.size(); ++i) {
        float y = startY + i * fieldH;
        fields[i].rect = sf::FloatRect({boxX, y}, {boxW, boxH});
    }

    // Buttons at the bottom
    const float btnY = WIN_H - 70.f;
    const float btnW = 130.f;
    const float btnH = 40.f;
    btnStartRect  = sf::FloatRect({WIN_W * 0.5f - btnW - 10.f, btnY}, {btnW, btnH});
    btnCancelRect = sf::FloatRect({WIN_W * 0.5f + 10.f,        btnY}, {btnW, btnH});
}

bool InputDialog::show(SimConfig& outConfig) {
    while (window.isOpen()) {
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
                return false;
            }
            else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
                onKeyPressed(keyPressed->code);
            }
            else if (const auto* textEntered = event->getIf<sf::Event::TextEntered>()) {
                onTextEntered(static_cast<char32_t>(textEntered->unicode));
            }
            else if (const auto* mouseBtn = event->getIf<sf::Event::MouseButtonPressed>()) {
                if (mouseBtn->button == sf::Mouse::Button::Left) {
                    onClick(static_cast<float>(mouseBtn->position.x),
                            static_cast<float>(mouseBtn->position.y));
                }
            }
        }

        if (startClicked) {
            if (validateAll(outConfig)) {
                window.close();
                return true;
            }
            startClicked = false;  // re-enable trying again
        }
        if (cancelClicked) {
            window.close();
            return false;
        }

        drawAll();
    }
    return false;
}

void InputDialog::onKeyPressed(sf::Keyboard::Key key) {
    if (key == sf::Keyboard::Key::Escape) {
        cancelClicked = true;
        return;
    }
    if (key == sf::Keyboard::Key::Enter) {
        startClicked = true;
        return;
    }
    if (key == sf::Keyboard::Key::Tab) {
        focusedField = (focusedField + 1) % static_cast<int>(fields.size());
        return;
    }
    if (key == sf::Keyboard::Key::Up) {
        focusedField = (focusedField - 1 + static_cast<int>(fields.size())) %
                       static_cast<int>(fields.size());
        return;
    }
    if (key == sf::Keyboard::Key::Down) {
        focusedField = (focusedField + 1) % static_cast<int>(fields.size());
        return;
    }
    if (key == sf::Keyboard::Key::Backspace) {
        if (focusedField >= 0 && focusedField < (int)fields.size()) {
            auto& t = fields[focusedField].text;
            if (!t.empty()) t.pop_back();
            validateField(focusedField);
        }
    }
}

void InputDialog::onTextEntered(char32_t ch) {
    if (focusedField < 0 || focusedField >= (int)fields.size()) return;

    // Accept digits, decimal point, minus sign
    if ((ch >= '0' && ch <= '9') || ch == '.' || ch == '-') {
        // Don't allow more than one decimal point or misplaced minus sign
        auto& t = fields[focusedField].text;
        if (ch == '.' && t.find('.') != std::string::npos) return;
        if (ch == '-' && !t.empty()) return;
        if (t.size() < 12) {
            t.push_back(static_cast<char>(ch));
            validateField(focusedField);
        }
    }
}

void InputDialog::onClick(float x, float y) {
    // Click on a field?
    for (size_t i = 0; i < fields.size(); ++i) {
        if (pointInRect(x, y, fields[i].rect)) {
            focusedField = static_cast<int>(i);
            return;
        }
    }
    if (pointInRect(x, y, btnStartRect))  { startClicked  = true; return; }
    if (pointInRect(x, y, btnCancelRect)) { cancelClicked = true; return; }
}

bool InputDialog::pointInRect(float x, float y, const sf::FloatRect& r) const {
    return x >= r.position.x && x <= r.position.x + r.size.x &&
           y >= r.position.y && y <= r.position.y + r.size.y;
}

bool InputDialog::validateField(int idx) {
    Field& f = fields[idx];
    f.error.clear();

    if (f.text.empty() || f.text == "-" || f.text == ".") {
        f.error = "required";
        return false;
    }

    try {
        double v = std::stod(f.text);
        if (v < f.minVal || v > f.maxVal) {
            std::ostringstream oss;
            oss << "must be " << f.minVal << " - " << f.maxVal;
            f.error = oss.str();
            return false;
        }
    } catch (const std::exception&) {
        f.error = "not a number";
        return false;
    }
    return true;
}

bool InputDialog::validateAll(SimConfig& outConfig) {
    bool allOk = true;
    for (size_t i = 0; i < fields.size(); ++i) {
        if (!validateField(static_cast<int>(i))) allOk = false;
    }
    if (!allOk) {
        globalError = "Please fix the highlighted fields.";
        return false;
    }

    SimConfig cfg;
    cfg.speed     = std::stod(fields[0].text);
    cfg.turnRate  = std::stod(fields[1].text);
    cfg.duration  = std::stod(fields[2].text);
    cfg.gpsNoise  = std::stod(fields[3].text);
    cfg.mass      = std::stod(fields[4].text);

    try {
        cfg.validate();
    } catch (const std::exception& e) {
        globalError = e.what();
        return false;
    }

    outConfig = cfg;
    globalError.clear();
    return true;
}

void InputDialog::drawAll() {
    window.clear(sf::Color(28, 32, 44));

    // Title
    drawText("Configure Simulation", 30, 20, 22, sf::Color(220, 235, 255));
    drawText("Enter parameters and click START.", 30, 50, 13,
             sf::Color(150, 165, 190));

    // Each field
    for (size_t i = 0; i < fields.size(); ++i) {
        drawField(static_cast<int>(i));
    }

    // Global error if any
    if (!globalError.empty()) {
        drawText(globalError, 30, WIN_H - 110, 13, sf::Color(255, 100, 100));
    }

    drawButton(btnStartRect,  "START",  sf::Color(60, 180, 90));
    drawButton(btnCancelRect, "CANCEL", sf::Color(150, 80, 80));

    window.display();
}

void InputDialog::drawField(int idx) {
    const Field& f = fields[idx];
    bool focused   = (idx == focusedField);
    bool hasError  = !f.error.empty();

    // Label
    drawText(f.label,
             30, f.rect.position.y + 6, 14,
             sf::Color(200, 215, 235));

    // Range hint
    {
        std::ostringstream oss;
        oss << "(" << f.minVal << " - " << f.maxVal << ")";
        drawText(oss.str(), 30, f.rect.position.y + 24, 11,
                 sf::Color(120, 135, 160));
    }

    // Box background
    sf::RectangleShape box(f.rect.size);
    box.setPosition(f.rect.position);
    box.setFillColor(sf::Color(45, 52, 70));
    if (hasError)      box.setOutlineColor(sf::Color(255, 100, 100));
    else if (focused)  box.setOutlineColor(sf::Color(120, 180, 255));
    else               box.setOutlineColor(sf::Color(80, 90, 110));
    box.setOutlineThickness(2);
    window.draw(box);

    // Text content
    sf::Color txtCol = hasError ? sf::Color(255, 200, 200)
                                : sf::Color(245, 250, 255);
    std::string display = f.text + f.suffix;
    if (focused) {
        // Insert a simple cursor before the suffix
        display = f.text + "|" + f.suffix;
    }
    drawText(display, f.rect.position.x + 8, f.rect.position.y + 7, 16, txtCol);

    // Error message
    if (hasError) {
        drawText(f.error,
                 f.rect.position.x + f.rect.size.x + 8,
                 f.rect.position.y + 8, 12,
                 sf::Color(255, 120, 120));
    }
}

void InputDialog::drawButton(const sf::FloatRect& r, const std::string& label,
                             sf::Color base) {
    sf::RectangleShape btn(r.size);
    btn.setPosition(r.position);
    btn.setFillColor(base);
    btn.setOutlineColor(sf::Color::White);
    btn.setOutlineThickness(2);
    window.draw(btn);

    float cx = r.position.x + r.size.x * 0.5f - label.size() * 5.0f;
    float cy = r.position.y + r.size.y * 0.5f - 11.f;
    drawText(label, cx, cy, 16, sf::Color::White);
}

void InputDialog::drawText(const std::string& s, float x, float y,
                           unsigned size, sf::Color color) {
    if (!fontLoaded) return;
    sf::Text t(font, s, size);
    t.setFillColor(color);
    t.setPosition({x, y});
    window.draw(t);
}
