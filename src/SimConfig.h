#ifndef SIM_CONFIG_H
#define SIM_CONFIG_H

#include <string>
#include <stdexcept>
#include <sstream>

/**
 * @brief All user-configurable simulation parameters.
 *
 * Used by both terminal and GUI run modes. validate() throws an
 * std::invalid_argument with a descriptive message if any value is
 * out of range.
 */
struct SimConfig {
    double speed       = 5.0;    // m/s        — vehicle scalar speed
    double turnRate    = 10.0;   // deg/s      — heading change rate (positive = left)
    double duration    = 0.0;    // seconds    — 0 means run forever (until window closed)
    double gpsNoise    = 1.5;    // meters     — GPS position noise sigma
    double mass        = 1000.0; // kg         — vehicle mass (affects turning agility)

    // Hard limits
    static constexpr double MIN_SPEED      =  0.1;
    static constexpr double MAX_SPEED      = 50.0;
    static constexpr double MIN_TURN       = -90.0;
    static constexpr double MAX_TURN       =  90.0;
    static constexpr double MIN_DURATION   =   0.0;     // 0 = infinite
    static constexpr double MAX_DURATION   = 3600.0;
    static constexpr double MIN_GPS_NOISE  =   0.0;
    static constexpr double MAX_GPS_NOISE  =  20.0;
    static constexpr double MIN_MASS       =   100.0;   // 100 kg (motorcycle-ish)
    static constexpr double MAX_MASS       = 20000.0;   // 20000 kg (semi truck)

    /**
     * Throws std::invalid_argument with a descriptive message if any
     * field is out of its allowed range.
     */
    void validate() const {
        if (speed < MIN_SPEED || speed > MAX_SPEED) {
            std::ostringstream oss;
            oss << "Speed must be between " << MIN_SPEED << " and " << MAX_SPEED
                << " m/s (got " << speed << ")";
            throw std::invalid_argument(oss.str());
        }
        if (turnRate < MIN_TURN || turnRate > MAX_TURN) {
            std::ostringstream oss;
            oss << "Turn rate must be between " << MIN_TURN << " and " << MAX_TURN
                << " deg/s (got " << turnRate << ")";
            throw std::invalid_argument(oss.str());
        }
        if (duration < MIN_DURATION || duration > MAX_DURATION) {
            std::ostringstream oss;
            oss << "Duration must be between " << MIN_DURATION << " and " << MAX_DURATION
                << " seconds (got " << duration << ")";
            throw std::invalid_argument(oss.str());
        }
        if (gpsNoise < MIN_GPS_NOISE || gpsNoise > MAX_GPS_NOISE) {
            std::ostringstream oss;
            oss << "GPS noise must be between " << MIN_GPS_NOISE << " and " << MAX_GPS_NOISE
                << " m (got " << gpsNoise << ")";
            throw std::invalid_argument(oss.str());
        }
        if (mass < MIN_MASS || mass > MAX_MASS) {
            std::ostringstream oss;
            oss << "Mass must be between " << MIN_MASS << " and " << MAX_MASS
                << " kg (got " << mass << ")";
            throw std::invalid_argument(oss.str());
        }
    }
};

#endif // SIM_CONFIG_H
