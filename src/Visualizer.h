#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <SFML/Graphics.hpp>
#include <vector>
#include <string>
#include "StateEstimate.h"

/**
 * @brief Real-time SFML dashboard for the sensor fusion simulation.
 *
 * Displays:
 *   - A 2D map with the vehicle's live position, heading arrow, and trajectory
 *   - Live data panels for IMU, GPS, Temperature, and WheelEncoder sensors
 *   - The fused state estimate panel
 *   - Grid, axes, and on-screen title
 *
 * Controls:
 *   - SPACE: pause/resume
 *   - R:     reset view
 *   - ESC:   quit
 */
class Visualizer {
private:
    sf::RenderWindow window;
    sf::Font         font;
    bool             fontLoaded;

    // Layout constants
    static const unsigned WINDOW_WIDTH  = 1400;
    static const unsigned WINDOW_HEIGHT = 800;
    static const int MAP_WIDTH     = 1000;
    static const int MAP_HEIGHT    = 800;
    static const int PANEL_X       = 1010;
    static const int PANEL_WIDTH   = 380;

    // Map state
    float mapScale;           // pixels per meter
    float mapOriginX;         // world origin in screen pixels
    float mapOriginY;

    // Trajectory + paused state
    std::vector<sf::Vector2f> trajectory;
    bool paused;

    // Latest snapshots
    StateEstimate currentState;
    StateEstimate imuReading;
    StateEstimate gpsReading;
    StateEstimate tempReading;
    StateEstimate encoderReading;
    StateEstimate fusedState;

public:
    Visualizer();
    ~Visualizer() = default;

    bool isOpen() const;
    bool isPaused() const;
    void handleEvents();

    void update(const StateEstimate& current,
                const StateEstimate& imu,
                const StateEstimate& gps,
                const StateEstimate& temp,
                const StateEstimate& encoder,
                const StateEstimate& fused);

    void render();

private:
    // Drawing helpers
    void drawBackground();
    void drawGrid();
    void drawTrajectory();
    void drawVehicle();
    void drawSensorPanel(int slot, const std::string& name,
                         const StateEstimate& reading, sf::Color headerColor);
    void drawFusedPanel();
    void drawTitleBar();
    void drawLegend();
    void drawText(const std::string& str, float x, float y,
                  unsigned size, sf::Color color);

    // Coordinate transform
    sf::Vector2f worldToScreen(float wx, float wy) const;

    // Formatting
    static std::string fmt(double val, int precision = 2);
};

#endif // VISUALIZER_H
