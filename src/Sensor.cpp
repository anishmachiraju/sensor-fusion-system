#include "Sensor.h"
#include <chrono>

Sensor::Sensor(const std::string& type, double sigma, double bias, double rate)
    : sensorType(type),
      noiseSigma(sigma),
      biasOffset(bias),
      isCalibrated(false),
      calibrationOffset(0.0),
      calibrationScale(1.0),
      updateRate(rate),
      lastUpdateTime(-1.0),
      rng(static_cast<unsigned>(
          std::chrono::steady_clock::now().time_since_epoch().count())),
      noiseDist(0.0, sigma)
{
    if (rate <= 0.0) {
        throw std::invalid_argument("Sensor update rate must be > 0");
    }
}

StateEstimate Sensor::generateReading(const StateEstimate& trueState) {
    if (trueState.timestamp < 0.0) {
        throw std::invalid_argument("True state timestamp must be >= 0");
    }

    StateEstimate reading = trueState;

    // Add noise + bias to all position/velocity fields (base behavior)
    reading.positionX += sampleNoise() + biasOffset;
    reading.positionY += sampleNoise() + biasOffset;
    reading.velocityX += sampleNoise() + biasOffset;
    reading.velocityY += sampleNoise() + biasOffset;
    reading.heading   += sampleNoise() + biasOffset;

    // Apply calibration if set
    if (isCalibrated) {
        reading.positionX = (reading.positionX - calibrationOffset) * calibrationScale;
        reading.positionY = (reading.positionY - calibrationOffset) * calibrationScale;
        reading.velocityX = (reading.velocityX - calibrationOffset) * calibrationScale;
        reading.velocityY = (reading.velocityY - calibrationOffset) * calibrationScale;
        reading.heading   = (reading.heading   - calibrationOffset) * calibrationScale;
    }

    lastReading    = reading;
    lastUpdateTime = trueState.timestamp;
    return reading;
}

void Sensor::calibrate(double offset, double scale) {
    if (scale == 0.0) {
        throw std::invalid_argument("Calibration scale must not be zero");
    }
    calibrationOffset = offset;
    calibrationScale  = scale;
    isCalibrated      = true;
}

std::string Sensor::getType() const {
    return sensorType;
}

bool Sensor::shouldUpdate(double currentTime) const {
    if (lastUpdateTime < 0.0) return true;          // never updated yet
    double interval = 1.0 / updateRate;
    return (currentTime - lastUpdateTime) >= interval;
}

StateEstimate Sensor::getLastReading() const {
    return lastReading;
}

// ---------- protected helpers ----------

double Sensor::sampleNoise() {
    return noiseDist(rng);
}

double Sensor::sampleNoise(double sigma) {
    std::normal_distribution<double> d(0.0, sigma);
    return d(rng);
}

void Sensor::setLastReading(const StateEstimate& reading) {
    lastReading = reading;
}

bool   Sensor::getIsCalibrated()      const { return isCalibrated; }
double Sensor::getCalibrationOffset() const { return calibrationOffset; }
double Sensor::getCalibrationScale()  const { return calibrationScale; }
