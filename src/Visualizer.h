#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <SFML/Graphics.hpp>
#include <vector>
#include <string>
#include "StateEstimate.h"

/**
 * @brief Real-time SFML dashboard for the sensor fusion simulation.
 *
 * Features:
 *   - Live 2D map with the vehicle, heading arrow, and trajectory trail
 *   - Data panels for IMU, GPS, Temperature, and WheelEncoder sensors
 *   - Fused state estimate panel
 *   - On-screen START / PAUSE / RESET buttons (clickable)
 *
 * Keyboard shortcuts:
 *   - SPACE:  pause / resume
 *   - R:      reset simulation
 *   - ESC:    quit
 */
class Visualizer {
public:
    enum class SimState { Stopped, Running, Paused };

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
    float mapScale;
    float mapOriginX;
    float mapOriginY;

    // Trajectory + simulation state
    std::vector<sf::Vector2f> trajectory;
    SimState  simState;
    bool      resetFlag;  // set when Reset button pressed; main.cpp clears it

    // Button rectangles (for hit testing)
    sf::FloatRect btnStartRect;
    sf::FloatRect btnPauseRect;
    sf::FloatRect btnResetRect;

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

    bool isOpen()    const;
    bool isRunning() const;   // true if SimState::Running
    bool isPaused()  const;
    SimState getState() const;

    // Returns true once when reset was requested; caller clears it by calling.
    bool consumeResetRequest();

    void handleEvents();

    void update(const StateEstimate& current,
                const StateEstimate& imu,
                const StateEstimate& gps,
                const StateEstimate& temp,
                const StateEstimate& encoder,
                const StateEstimate& fused);

    // Fully clear state (called by main.cpp after it rebuilds the vehicle)
    void clearForReset();

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
    void drawButtons();
    void drawButton(const sf::FloatRect& rect, const std::string& label,
                    sf::Color baseColor, bool enabled, bool highlighted);
    void drawText(const std::string& str, float x, float y,
                  unsigned size, sf::Color color);

    // Mouse handling
    bool pointInRect(float x, float y, const sf::FloatRect& r) const;
    void handleButtonClick(float mx, float my);

    // Coordinate transform
    sf::Vector2f worldToScreen(float wx, float wy) const;

    // Formatting
    static std::string fmt(double val, int precision = 2);
};

#endif // VISUALIZER_H
