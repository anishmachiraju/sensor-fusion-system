#include "Visualizer.h"
#include <sstream>
#include <iomanip>
#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---- Construction ----

Visualizer::Visualizer()
    : window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT),
             "Sensor Fusion System - Autonomous Vehicle Simulation",
             sf::Style::Titlebar | sf::Style::Close),
      fontLoaded(false),
      mapScale(12.0f),
      mapOriginX(MAP_WIDTH * 0.25f),
      mapOriginY(MAP_HEIGHT * 0.75f),
      paused(false)
{
    window.setFramerateLimit(60);

    // Try loading a font from common system locations
    const char* fontPaths[] = {
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
        "C:/Windows/Fonts/arial.ttf",
        "arial.ttf",
        "DejaVuSans.ttf"
    };
    for (const char* path : fontPaths) {
        if (font.loadFromFile(path)) {
            fontLoaded = true;
            break;
        }
    }
    if (!fontLoaded) {
        std::cerr << "[Visualizer] Warning: no font found — text will not render\n";
    }
}

// ---- Public API ----

bool Visualizer::isOpen()   const { return window.isOpen(); }
bool Visualizer::isPaused() const { return paused; }

void Visualizer::handleEvents() {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window.close();
        }
        if (event.type == sf::Event::KeyPressed) {
            if (event.key.code == sf::Keyboard::Escape) {
                window.close();
            }
            if (event.key.code == sf::Keyboard::Space) {
                paused = !paused;
            }
            if (event.key.code == sf::Keyboard::R) {
                trajectory.clear();
            }
        }
    }
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

    // Record to trajectory
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

    // Right-side data panels
    drawSensorPanel(0, "IMU Sensor",          imuReading,
                    sf::Color(255, 128, 64));
    drawSensorPanel(1, "GPS Sensor",          gpsReading,
                    sf::Color(64, 180, 255));
    drawSensorPanel(2, "Temperature Sensor",  tempReading,
                    sf::Color(255, 80, 80));
    drawSensorPanel(3, "Wheel Encoder",       encoderReading,
                    sf::Color(160, 255, 100));
    drawFusedPanel();

    window.display();
}

// ---- Drawing helpers ----

void Visualizer::drawBackground() {
    // Map area background
    sf::RectangleShape map(sf::Vector2f(MAP_WIDTH, MAP_HEIGHT));
    map.setPosition(0, 0);
    map.setFillColor(sf::Color(28, 34, 46));
    window.draw(map);

    // Panel area background
    sf::RectangleShape panelBg(
        sf::Vector2f(WINDOW_WIDTH - MAP_WIDTH, WINDOW_HEIGHT));
    panelBg.setPosition(MAP_WIDTH, 0);
    panelBg.setFillColor(sf::Color(22, 26, 36));
    window.draw(panelBg);

    // Separator line
    sf::RectangleShape sep(sf::Vector2f(2, WINDOW_HEIGHT));
    sep.setPosition(MAP_WIDTH - 1, 0);
    sep.setFillColor(sf::Color(60, 70, 90));
    window.draw(sep);
}

void Visualizer::drawGrid() {
    sf::Color gridColor(50, 60, 80);
    sf::Color axisColor(110, 130, 160);

    // Grid lines every 5 meters
    const float spacing = 5.0f;
    float pxPerMeter = mapScale;
    float step = spacing * pxPerMeter;

    // Vertical grid lines
    for (float x = mapOriginX; x < MAP_WIDTH; x += step) {
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(x, 0),        gridColor),
            sf::Vertex(sf::Vector2f(x, MAP_HEIGHT), gridColor)
        };
        window.draw(line, 2, sf::Lines);
    }
    for (float x = mapOriginX - step; x > 0; x -= step) {
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(x, 0),        gridColor),
            sf::Vertex(sf::Vector2f(x, MAP_HEIGHT), gridColor)
        };
        window.draw(line, 2, sf::Lines);
    }

    // Horizontal grid lines
    for (float y = mapOriginY; y < MAP_HEIGHT; y += step) {
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(0,        y), gridColor),
            sf::Vertex(sf::Vector2f(MAP_WIDTH, y), gridColor)
        };
        window.draw(line, 2, sf::Lines);
    }
    for (float y = mapOriginY - step; y > 0; y -= step) {
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(0,        y), gridColor),
            sf::Vertex(sf::Vector2f(MAP_WIDTH, y), gridColor)
        };
        window.draw(line, 2, sf::Lines);
    }

    // Main axes through origin
    sf::Vertex xAxis[] = {
        sf::Vertex(sf::Vector2f(0,        mapOriginY), axisColor),
        sf::Vertex(sf::Vector2f(MAP_WIDTH, mapOriginY), axisColor)
    };
    sf::Vertex yAxis[] = {
        sf::Vertex(sf::Vector2f(mapOriginX, 0),         axisColor),
        sf::Vertex(sf::Vector2f(mapOriginX, MAP_HEIGHT), axisColor)
    };
    window.draw(xAxis, 2, sf::Lines);
    window.draw(yAxis, 2, sf::Lines);

    // Axis labels
    drawText("X (m)", MAP_WIDTH - 55, mapOriginY + 5, 14, axisColor);
    drawText("Y (m)", mapOriginX + 5, 5,              14, axisColor);
    drawText("(0,0)", mapOriginX + 4, mapOriginY + 4, 12, axisColor);
}

void Visualizer::drawTrajectory() {
    if (trajectory.size() < 2) return;

    // Fading line: older points more transparent
    for (size_t i = 1; i < trajectory.size(); ++i) {
        float t = static_cast<float>(i) / trajectory.size();
        sf::Uint8 alpha = static_cast<sf::Uint8>(60 + 180 * t);
        sf::Color col(100, 220, 255, alpha);
        sf::Vertex line[] = {
            sf::Vertex(trajectory[i - 1], col),
            sf::Vertex(trajectory[i],     col)
        };
        window.draw(line, 2, sf::Lines);
    }
}

void Visualizer::drawVehicle() {
    sf::Vector2f pos = worldToScreen(
        static_cast<float>(currentState.positionX),
        static_cast<float>(currentState.positionY));

    // Outer glow
    sf::CircleShape glow(16);
    glow.setOrigin(16, 16);
    glow.setPosition(pos);
    glow.setFillColor(sf::Color(100, 220, 255, 40));
    window.draw(glow);

    // Vehicle body — triangle pointing in heading direction
    float headingRad = static_cast<float>(currentState.heading) * M_PI / 180.0f;
    // In our world, Y points up but SFML Y points down — flip sin
    sf::ConvexShape body;
    body.setPointCount(3);
    body.setPoint(0, sf::Vector2f( 14,  0));   // nose
    body.setPoint(1, sf::Vector2f(-10,  8));   // rear right
    body.setPoint(2, sf::Vector2f(-10, -8));   // rear left
    body.setFillColor(sf::Color(255, 230, 100));
    body.setOutlineColor(sf::Color(255, 180, 0));
    body.setOutlineThickness(2);
    body.setPosition(pos);
    // SFML rotates clockwise (because Y is flipped), so use -heading
    body.setRotation(-static_cast<float>(currentState.heading));
    window.draw(body);

    // Position label
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

    // Panel background
    sf::RectangleShape bg(sf::Vector2f(PANEL_WIDTH, panelH));
    bg.setPosition(PANEL_X, y);
    bg.setFillColor(sf::Color(32, 38, 52));
    bg.setOutlineColor(sf::Color(60, 70, 90));
    bg.setOutlineThickness(1);
    window.draw(bg);

    // Header strip
    sf::RectangleShape header(sf::Vector2f(PANEL_WIDTH, 24));
    header.setPosition(PANEL_X, y);
    header.setFillColor(headerColor);
    window.draw(header);

    // Title
    drawText(name, PANEL_X + margin, y + 4, 14, sf::Color::Black);

    // Data rows
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
    // Bottom panel spanning full panel width
    const int panelH = 130;
    int y = WINDOW_HEIGHT - panelH - 10;

    sf::RectangleShape bg(sf::Vector2f(PANEL_WIDTH, panelH));
    bg.setPosition(PANEL_X, y);
    bg.setFillColor(sf::Color(45, 55, 75));
    bg.setOutlineColor(sf::Color(120, 180, 255));
    bg.setOutlineThickness(2);
    window.draw(bg);

    sf::RectangleShape header(sf::Vector2f(PANEL_WIDTH, 26));
    header.setPosition(PANEL_X, y);
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
    bar.setPosition(0, 0);
    bar.setFillColor(sf::Color(12, 15, 22, 220));
    window.draw(bar);

    drawText("Sensor Fusion System - Live Vehicle Tracking",
             12, 10, 18, sf::Color(220, 235, 255));

    std::string status = paused ? "PAUSED (press SPACE)" : "RUNNING";
    sf::Color statusCol = paused ? sf::Color(255, 180, 80)
                                 : sf::Color(120, 255, 160);
    drawText(status, MAP_WIDTH - 220, 12, 14, statusCol);

    drawText("Controls: SPACE=pause  R=clear trail  ESC=quit",
             12, MAP_HEIGHT - 22, 12, sf::Color(120, 140, 170));
}

void Visualizer::drawLegend() {
    int x = MAP_WIDTH - 200;
    int y = 50;

    sf::RectangleShape bg(sf::Vector2f(190, 60));
    bg.setPosition(x, y);
    bg.setFillColor(sf::Color(20, 25, 35, 200));
    bg.setOutlineColor(sf::Color(60, 70, 90));
    bg.setOutlineThickness(1);
    window.draw(bg);

    // Vehicle swatch
    sf::ConvexShape tri;
    tri.setPointCount(3);
    tri.setPoint(0, sf::Vector2f( 7, 0));
    tri.setPoint(1, sf::Vector2f(-5, 5));
    tri.setPoint(2, sf::Vector2f(-5, -5));
    tri.setFillColor(sf::Color(255, 230, 100));
    tri.setPosition(x + 20, y + 18);
    window.draw(tri);
    drawText("Vehicle", x + 38, y + 10, 13, sf::Color(220, 230, 240));

    // Trail swatch
    sf::Vertex trail[] = {
        sf::Vertex(sf::Vector2f(x + 10, y + 40), sf::Color(100, 220, 255)),
        sf::Vertex(sf::Vector2f(x + 30, y + 40), sf::Color(100, 220, 255))
    };
    window.draw(trail, 2, sf::Lines);
    drawText("Trajectory", x + 38, y + 33, 13, sf::Color(220, 230, 240));
}

void Visualizer::drawText(const std::string& str, float x, float y,
                          unsigned size, sf::Color color) {
    if (!fontLoaded) return;
    sf::Text text;
    text.setFont(font);
    text.setString(str);
    text.setCharacterSize(size);
    text.setFillColor(color);
    text.setPosition(x, y);
    window.draw(text);
}

// ---- Coordinate transform ----

sf::Vector2f Visualizer::worldToScreen(float wx, float wy) const {
    // World Y points "up" but SFML screen Y points "down", so we negate Y
    return sf::Vector2f(mapOriginX + wx * mapScale,
                        mapOriginY - wy * mapScale);
}

// ---- Formatting ----

std::string Visualizer::fmt(double val, int precision) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << val;
    return oss.str();
}
