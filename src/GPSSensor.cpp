#include "GPSSensor.h"

GPSSensor::GPSSensor(double posSigma, double posBias, double dropout, double rate)
    : Sensor("GPS", posSigma, posBias, rate),
      positionNoiseSigma(posSigma),
      positionBias(posBias),
      dropoutRate(dropout),
      uniformDist(0.0, 1.0)
{
}

StateEstimate GPSSensor::generateReading(const StateEstimate& trueState) {
    // Simulate signal dropout
    if (uniformDist(rng) < dropoutRate) {
        return getLastReading();  // return stale reading
    }

    StateEstimate reading = trueState;

    // Perturb position only
    reading.positionX += sampleNoise(positionNoiseSigma) + positionBias;
    reading.positionY += sampleNoise(positionNoiseSigma) + positionBias;

    // Apply calibration if set
    if (getIsCalibrated()) {
        double off = getCalibrationOffset();
        double scl = getCalibrationScale();
        reading.positionX = (reading.positionX - off) * scl;
        reading.positionY = (reading.positionY - off) * scl;
    }

    setLastReading(reading);
    return reading;
}

void GPSSensor::calibrate(double offset, double scale) {
    Sensor::calibrate(offset, scale);
}

std::string GPSSensor::getType() const {
    return "GPS";
}
