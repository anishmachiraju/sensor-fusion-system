#include "Visualizer.h"
#include <sstream>
#include <iomanip>
#include <cmath>
#include <iostream>
#include <optional>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---- Construction ----

Visualizer::Visualizer()
    : window(sf::VideoMode({WINDOW_WIDTH, WINDOW_HEIGHT}),
             "Sensor Fusion System - Autonomous Vehicle Simulation",
             sf::Style::Titlebar | sf::Style::Close),
      fontLoaded(false),
      mapScale(12.0f),
      mapOriginX(MAP_WIDTH * 0.25f),
      mapOriginY(MAP_HEIGHT * 0.75f),
      simState(SimState::Stopped),
      resetFlag(false)
{
    window.setFramerateLimit(60);

    // Button layout: bottom-center of the map area
    const float btnW = 110.f;
    const float btnH = 36.f;
    const float btnY = MAP_HEIGHT - 60.f;
    const float gap  = 12.f;
    const float groupW = 3 * btnW + 2 * gap;
    const float startX = (MAP_WIDTH - groupW) * 0.5f;

    btnStartRect = sf::FloatRect({startX,                       btnY}, {btnW, btnH});
    btnPauseRect = sf::FloatRect({startX + (btnW + gap),        btnY}, {btnW, btnH});
    btnResetRect = sf::FloatRect({startX + 2 * (btnW + gap),    btnY}, {btnW, btnH});

    // Try loading a font from common system locations
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
    for (const char* path : fontPaths) {
        if (font.openFromFile(path)) {
            fontLoaded = true;
            break;
        }
    }
    if (!fontLoaded) {
        std::cerr << "[Visualizer] Warning: no font found - text will not render\n";
    }
}

// ---- Public API ----

bool Visualizer::isOpen()    const { return window.isOpen(); }
bool Visualizer::isRunning() const { return simState == SimState::Running; }
bool Visualizer::isPaused()  const { return simState == SimState::Paused; }
Visualizer::SimState Visualizer::getState() const { return simState; }

bool Visualizer::consumeResetRequest() {
    if (resetFlag) {
        resetFlag = false;
        return true;
    }
    return false;
}

void Visualizer::clearForReset() {
    trajectory.clear();
    currentState   = StateEstimate{};
    imuReading     = StateEstimate{};
    gpsReading     = StateEstimate{};
    tempReading    = StateEstimate{};
    encoderReading = StateEstimate{};
    fusedState     = StateEstimate{};
}

void Visualizer::handleEvents() {
    while (const std::optional event = window.pollEvent()) {
        if (event->is<sf::Event::Closed>()) {
            window.close();
        }
        else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
            if (keyPressed->code == sf::Keyboard::Key::Escape) {
                window.close();
            }
            else if (keyPressed->code == sf::Keyboard::Key::Space) {
                // Toggle between Running and Paused (from Stopped, Space starts it)
                if (simState == SimState::Running)      simState = SimState::Paused;
                else if (simState == SimState::Paused)  simState = SimState::Running;
                else if (simState == SimState::Stopped) simState = SimState::Running;
            }
            else if (keyPressed->code == sf::Keyboard::Key::R) {
                resetFlag = true;
                simState  = SimState::Stopped;
            }
        }
        else if (const auto* mouseBtn = event->getIf<sf::Event::MouseButtonPressed>()) {
            if (mouseBtn->button == sf::Mouse::Button::Left) {
                handleButtonClick(static_cast<float>(mouseBtn->position.x),
                                  static_cast<float>(mouseBtn->position.y));
            }
        }
    }
}

void Visualizer::handleButtonClick(float mx, float my) {
    if (pointInRect(mx, my, btnStartRect)) {
        // START acts as Start (from Stopped) or Resume (from Paused)
        if (simState != SimState::Running) simState = SimState::Running;
    }
    else if (pointInRect(mx, my, btnPauseRect)) {
        if (simState == SimState::Running) simState = SimState::Paused;
    }
    else if (pointInRect(mx, my, btnResetRect)) {
        resetFlag = true;
        simState  = SimState::Stopped;
    }
}

bool Visualizer::pointInRect(float x, float y, const sf::FloatRect& r) const {
    return x >= r.position.x && x <= r.position.x + r.size.x &&
           y >= r.position.y && y <= r.position.y + r.size.y;
}

void Visualizer::update(const StateEstimate& current,
                        const StateEstimate& imu,
                        const StateEstimate& gps,
                        const StateEstimate& temp,
                        const StateEstimate& encoder,
                        const StateEstimate& fused) {
    currentState   = current;
    imuReading     = imu;
    gpsReading     = gps;
    tempReading    = temp;
    encoderReading = encoder;
    fusedState     = fused;

    sf::Vector2f pt = worldToScreen(
        static_cast<float>(current.positionX),
        static_cast<float>(current.positionY));
    trajectory.push_back(pt);
}

void Visualizer::render() {
    window.clear(sf::Color(18, 22, 30));

    drawBackground();
    drawGrid();
    drawTrajectory();
    drawVehicle();
    drawTitleBar();
    drawLegend();
    drawButtons();

    drawSensorPanel(0, "IMU Sensor",         imuReading,
                    sf::Color(255, 128, 64));
    drawSensorPanel(1, "GPS Sensor",         gpsReading,
                    sf::Color(64, 180, 255));
    drawSensorPanel(2, "Temperature Sensor", tempReading,
                    sf::Color(255, 80, 80));
    drawSensorPanel(3, "Wheel Encoder",      encoderReading,
                    sf::Color(160, 255, 100));
    drawFusedPanel();

    window.display();
}

// ---- Drawing helpers ----

void Visualizer::drawBackground() {
    sf::RectangleShape map(sf::Vector2f(MAP_WIDTH, MAP_HEIGHT));
    map.setPosition({0.f, 0.f});
    map.setFillColor(sf::Color(28, 34, 46));
    window.draw(map);

    sf::RectangleShape panelBg(
        sf::Vector2f(WINDOW_WIDTH - MAP_WIDTH, WINDOW_HEIGHT));
    panelBg.setPosition({static_cast<float>(MAP_WIDTH), 0.f});
    panelBg.setFillColor(sf::Color(22, 26, 36));
    window.draw(panelBg);

    sf::RectangleShape sep(sf::Vector2f(2.f, WINDOW_HEIGHT));
    sep.setPosition({static_cast<float>(MAP_WIDTH - 1), 0.f});
    sep.setFillColor(sf::Color(60, 70, 90));
    window.draw(sep);
}

void Visualizer::drawGrid() {
    sf::Color gridColor(50, 60, 80);
    sf::Color axisColor(110, 130, 160);

    const float spacing = 5.0f;
    float step = spacing * mapScale;

    for (float x = mapOriginX; x < MAP_WIDTH; x += step) {
        sf::Vertex line[] = {
            sf::Vertex{sf::Vector2f(x, 0),            gridColor},
            sf::Vertex{sf::Vector2f(x, MAP_HEIGHT),   gridColor}
        };
        window.draw(line, 2, sf::PrimitiveType::Lines);
    }
    for (float x = mapOriginX - step; x > 0; x -= step) {
        sf::Vertex line[] = {
            sf::Vertex{sf::Vector2f(x, 0),            gridColor},
            sf::Vertex{sf::Vector2f(x, MAP_HEIGHT),   gridColor}
        };
        window.draw(line, 2, sf::PrimitiveType::Lines);
    }
    for (float y = mapOriginY; y < MAP_HEIGHT; y += step) {
        sf::Vertex line[] = {
            sf::Vertex{sf::Vector2f(0, y),            gridColor},
            sf::Vertex{sf::Vector2f(MAP_WIDTH, y),    gridColor}
        };
        window.draw(line, 2, sf::PrimitiveType::Lines);
    }
    for (float y = mapOriginY - step; y > 0; y -= step) {
        sf::Vertex line[] = {
            sf::Vertex{sf::Vector2f(0, y),            gridColor},
            sf::Vertex{sf::Vector2f(MAP_WIDTH, y),    gridColor}
        };
        window.draw(line, 2, sf::PrimitiveType::Lines);
    }

    sf::Vertex xAxis[] = {
        sf::Vertex{sf::Vector2f(0, mapOriginY),           axisColor},
        sf::Vertex{sf::Vector2f(MAP_WIDTH, mapOriginY),   axisColor}
    };
    sf::Vertex yAxis[] = {
        sf::Vertex{sf::Vector2f(mapOriginX, 0),           axisColor},
        sf::Vertex{sf::Vector2f(mapOriginX, MAP_HEIGHT),  axisColor}
    };
    window.draw(xAxis, 2, sf::PrimitiveType::Lines);
    window.draw(yAxis, 2, sf::PrimitiveType::Lines);

    drawText("X (m)", MAP_WIDTH - 55, mapOriginY + 5, 14, axisColor);
    drawText("Y (m)", mapOriginX + 5, 5,              14, axisColor);
    drawText("(0,0)", mapOriginX + 4, mapOriginY + 4, 12, axisColor);
}

void Visualizer::drawTrajectory() {
    if (trajectory.size() < 2) return;

    for (size_t i = 1; i < trajectory.size(); ++i) {
        float t = static_cast<float>(i) / trajectory.size();
        std::uint8_t alpha = static_cast<std::uint8_t>(60 + 180 * t);
        sf::Color col(100, 220, 255, alpha);
        sf::Vertex line[] = {
            sf::Vertex{trajectory[i - 1], col},
            sf::Vertex{trajectory[i],     col}
        };
        window.draw(line, 2, sf::PrimitiveType::Lines);
    }
}

void Visualizer::drawVehicle() {
    // Don't draw the vehicle until simulation has actually started
    if (simState == SimState::Stopped && trajectory.empty()) return;

    sf::Vector2f pos = worldToScreen(
        static_cast<float>(currentState.positionX),
        static_cast<float>(currentState.positionY));

    sf::CircleShape glow(16.f);
    glow.setOrigin({16.f, 16.f});
    glow.setPosition(pos);
    glow.setFillColor(sf::Color(100, 220, 255, 40));
    window.draw(glow);

    sf::ConvexShape body;
    body.setPointCount(3);
    body.setPoint(0, sf::Vector2f( 14.f,  0.f));
    body.setPoint(1, sf::Vector2f(-10.f,  8.f));
    body.setPoint(2, sf::Vector2f(-10.f, -8.f));
    body.setFillColor(sf::Color(255, 230, 100));
    body.setOutlineColor(sf::Color(255, 180, 0));
    body.setOutlineThickness(2);
    body.setPosition(pos);
    body.setRotation(sf::degrees(-static_cast<float>(currentState.heading)));
    window.draw(body);

    std::ostringstream oss;
    oss << "(" << fmt(currentState.positionX, 1) << ", "
        << fmt(currentState.positionY, 1) << ")";
    drawText(oss.str(), pos.x + 18, pos.y - 8, 13,
             sf::Color(255, 230, 100));
}

void Visualizer::drawSensorPanel(int slot, const std::string& name,
                                 const StateEstimate& reading,
                                 sf::Color headerColor) {
    const int panelH = 140;
    const int margin = 10;
    int y = 50 + slot * (panelH + 8);

    sf::RectangleShape bg(sf::Vector2f(PANEL_WIDTH, panelH));
    bg.setPosition({static_cast<float>(PANEL_X), static_cast<float>(y)});
    bg.setFillColor(sf::Color(32, 38, 52));
    bg.setOutlineColor(sf::Color(60, 70, 90));
    bg.setOutlineThickness(1);
    window.draw(bg);

    sf::RectangleShape header(sf::Vector2f(PANEL_WIDTH, 24));
    header.setPosition({static_cast<float>(PANEL_X), static_cast<float>(y)});
    header.setFillColor(headerColor);
    window.draw(header);

    drawText(name, PANEL_X + margin, y + 4, 14, sf::Color::Black);

    int row = y + 32;
    int rowH = 18;
    sf::Color labelCol(160, 170, 190);
    sf::Color valueCol(240, 245, 255);

    auto drawRow = [&](const std::string& label, const std::string& value) {
        drawText(label, PANEL_X + margin,       row, 13, labelCol);
        drawText(value, PANEL_X + margin + 150, row, 13, valueCol);
        row += rowH;
    };

    drawRow("Position X:", fmt(reading.positionX, 2) + " m");
    drawRow("Position Y:", fmt(reading.positionY, 2) + " m");
    drawRow("Velocity:",   "(" + fmt(reading.velocityX, 2) + ", "
                               + fmt(reading.velocityY, 2) + ") m/s");
    drawRow("Heading:",    fmt(reading.heading, 2) + " deg");
    drawRow("Temperature:",fmt(reading.temperature, 2) + " C");
}

void Visualizer::drawFusedPanel() {
    const int panelH = 130;
    int y = WINDOW_HEIGHT - panelH - 10;

    sf::RectangleShape bg(sf::Vector2f(PANEL_WIDTH, panelH));
    bg.setPosition({static_cast<float>(PANEL_X), static_cast<float>(y)});
    bg.setFillColor(sf::Color(45, 55, 75));
    bg.setOutlineColor(sf::Color(120, 180, 255));
    bg.setOutlineThickness(2);
    window.draw(bg);

    sf::RectangleShape header(sf::Vector2f(PANEL_WIDTH, 26));
    header.setPosition({static_cast<float>(PANEL_X), static_cast<float>(y)});
    header.setFillColor(sf::Color(120, 180, 255));
    window.draw(header);

    drawText("FUSED STATE ESTIMATE", PANEL_X + 10, y + 5, 14,
             sf::Color::Black);

    int row = y + 34;
    int rowH = 18;
    sf::Color labelCol(170, 200, 230);
    sf::Color valueCol(255, 255, 255);

    auto drawRow = [&](const std::string& label, const std::string& value) {
        drawText(label, PANEL_X + 10,  row, 13, labelCol);
        drawText(value, PANEL_X + 160, row, 13, valueCol);
        row += rowH;
    };

    drawRow("Position:",    "(" + fmt(fusedState.positionX, 2) + ", "
                                + fmt(fusedState.positionY, 2) + ") m");
    drawRow("Velocity:",    "(" + fmt(fusedState.velocityX, 2) + ", "
                                + fmt(fusedState.velocityY, 2) + ") m/s");
    drawRow("Heading:",     fmt(fusedState.heading, 2) + " deg");
    drawRow("Temperature:", fmt(fusedState.temperature, 2) + " C");
    drawRow("Time:",        fmt(fusedState.timestamp, 2) + " s");
}

void Visualizer::drawTitleBar() {
    sf::RectangleShape bar(sf::Vector2f(MAP_WIDTH, 40));
    bar.setPosition({0.f, 0.f});
    bar.setFillColor(sf::Color(12, 15, 22, 220));
    window.draw(bar);

    drawText("Sensor Fusion System - Live Vehicle Tracking",
             12, 10, 18, sf::Color(220, 235, 255));

    std::string status;
    sf::Color statusCol;
    switch (simState) {
        case SimState::Running:
            status = "RUNNING";
            statusCol = sf::Color(120, 255, 160);
            break;
        case SimState::Paused:
            status = "PAUSED";
            statusCol = sf::Color(255, 180, 80);
            break;
        case SimState::Stopped:
            status = "READY - press START";
            statusCol = sf::Color(180, 200, 230);
            break;
    }
    drawText(status, MAP_WIDTH - 220, 12, 14, statusCol);
}

void Visualizer::drawLegend() {
    int x = MAP_WIDTH - 200;
    int y = 50;

    sf::RectangleShape bg(sf::Vector2f(190, 60));
    bg.setPosition({static_cast<float>(x), static_cast<float>(y)});
    bg.setFillColor(sf::Color(20, 25, 35, 200));
    bg.setOutlineColor(sf::Color(60, 70, 90));
    bg.setOutlineThickness(1);
    window.draw(bg);

    sf::ConvexShape tri;
    tri.setPointCount(3);
    tri.setPoint(0, sf::Vector2f( 7.f,  0.f));
    tri.setPoint(1, sf::Vector2f(-5.f,  5.f));
    tri.setPoint(2, sf::Vector2f(-5.f, -5.f));
    tri.setFillColor(sf::Color(255, 230, 100));
    tri.setPosition({static_cast<float>(x + 20), static_cast<float>(y + 18)});
    window.draw(tri);
    drawText("Vehicle", x + 38, y + 10, 13, sf::Color(220, 230, 240));

    sf::Vertex trail[] = {
        sf::Vertex{sf::Vector2f(x + 10, y + 40), sf::Color(100, 220, 255)},
        sf::Vertex{sf::Vector2f(x + 30, y + 40), sf::Color(100, 220, 255)}
    };
    window.draw(trail, 2, sf::PrimitiveType::Lines);
    drawText("Trajectory", x + 38, y + 33, 13, sf::Color(220, 230, 240));
}

void Visualizer::drawButtons() {
    bool startEnabled = (simState != SimState::Running);
    bool pauseEnabled = (simState == SimState::Running);
    bool resetEnabled = true;

    // Highlight the button that matches current state
    bool startHighlight = (simState == SimState::Running);
    bool pauseHighlight = (simState == SimState::Paused);

    drawButton(btnStartRect,
               simState == SimState::Paused ? "RESUME" : "START",
               sf::Color(60, 180, 90), startEnabled, startHighlight);

    drawButton(btnPauseRect, "PAUSE",
               sf::Color(220, 160, 50), pauseEnabled, pauseHighlight);

    drawButton(btnResetRect, "RESET",
               sf::Color(200, 70, 70), resetEnabled, false);
}

void Visualizer::drawButton(const sf::FloatRect& rect, const std::string& label,
                            sf::Color baseColor, bool enabled, bool highlighted) {
    sf::RectangleShape btn(rect.size);
    btn.setPosition(rect.position);

    if (!enabled) {
        btn.setFillColor(sf::Color(60, 65, 75));
        btn.setOutlineColor(sf::Color(80, 85, 95));
    } else if (highlighted) {
        // Brighter fill when this action represents current state
        btn.setFillColor(sf::Color(
            std::min(255, baseColor.r + 40),
            std::min(255, baseColor.g + 40),
            std::min(255, baseColor.b + 40)));
        btn.setOutlineColor(sf::Color::White);
    } else {
        btn.setFillColor(baseColor);
        btn.setOutlineColor(sf::Color(
            std::min(255, baseColor.r + 30),
            std::min(255, baseColor.g + 30),
            std::min(255, baseColor.b + 30)));
    }
    btn.setOutlineThickness(2);
    window.draw(btn);

    // Label (roughly centered). Without measuring text width we just
    // nudge by a fixed amount based on label length.
    sf::Color textCol = enabled ? sf::Color::White : sf::Color(130, 135, 145);
    float cx = rect.position.x + rect.size.x * 0.5f - label.size() * 4.5f;
    float cy = rect.position.y + rect.size.y * 0.5f - 10.f;
    drawText(label, cx, cy, 15, textCol);
}

void Visualizer::drawText(const std::string& str, float x, float y,
                          unsigned size, sf::Color color) {
    if (!fontLoaded) return;
    sf::Text text(font, str, size);
    text.setFillColor(color);
    text.setPosition({x, y});
    window.draw(text);
}

// ---- Coordinate transform ----

sf::Vector2f Visualizer::worldToScreen(float wx, float wy) const {
    return sf::Vector2f(mapOriginX + wx * mapScale,
                        mapOriginY - wy * mapScale);
}

// ---- Formatting ----

std::string Visualizer::fmt(double val, int precision) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << val;
    return oss.str();
}
