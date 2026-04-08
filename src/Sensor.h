#ifndef SENSOR_H
#define SENSOR_H

#include "StateEstimate.h"
#include <string>
#include <random>
#include <stdexcept>

/**
 * @brief Base class for all sensor types.
 *
 * Provides the common interface (generateReading, calibrate, shouldUpdate)
 * that every sensor subclass must implement or inherit.
 * David (Jeongheon Han) — primary author.
 */
class Sensor {
private:
    std::string  sensorType;
    [[maybe_unused]] double noiseSigma;
    double       biasOffset;
    StateEstimate lastReading;
    bool         isCalibrated;
    double       calibrationOffset;
    double       calibrationScale;

protected:
    double updateRate;      // Hz
    double lastUpdateTime;
    std::mt19937 rng;
    std::normal_distribution<double> noiseDist;

public:
    Sensor(const std::string& type, double sigma, double bias, double rate);
    virtual ~Sensor() = default;

    virtual StateEstimate generateReading(const StateEstimate& trueState);
    virtual void calibrate(double offset, double scale);
    virtual std::string getType() const;

    bool shouldUpdate(double currentTime) const;
    StateEstimate getLastReading() const;

protected:
    // Helpers available to subclasses
    double sampleNoise();
    double sampleNoise(double sigma);
    void   setLastReading(const StateEstimate& reading);
    bool   getIsCalibrated() const;
    double getCalibrationOffset() const;
    double getCalibrationScale() const;
};

#endif // SENSOR_H
