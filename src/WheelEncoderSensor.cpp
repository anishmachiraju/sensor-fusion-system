#include "WheelEncoderSensor.h"
#include <cmath>

WheelEncoderSensor::WheelEncoderSensor(double sigma, double bias,
                                       double radius, int ticks, double rate)
    : Sensor("WheelEncoder", sigma, bias, rate),
      tickNoiseSigma(sigma),
      tickBias(bias),
      wheelRadius(radius),
      ticksPerRevolution(ticks)
{
    if (radius <= 0.0) {
        throw std::invalid_argument("Wheel radius must be > 0");
    }
    if (ticks <= 0) {
        throw std::invalid_argument("Ticks per revolution must be > 0");
    }
}

StateEstimate WheelEncoderSensor::generateReading(const StateEstimate& trueState) {
    StateEstimate reading = trueState;

    // Compute true speed from velocity components
    double trueSpeed = std::sqrt(trueState.velocityX * trueState.velocityX +
                                 trueState.velocityY * trueState.velocityY);

    // Convert speed to encoder ticks, add noise, convert back
    double circumference   = 2.0 * M_PI * wheelRadius;
    double revsPerSecond   = trueSpeed / circumference;
    double trueTicks       = revsPerSecond * ticksPerRevolution;
    double noisyTicks      = trueTicks + sampleNoise(tickNoiseSigma) + tickBias;

    // Convert noisy ticks back to speed
    double noisySpeed = (noisyTicks / ticksPerRevolution) * circumference;
    if (noisySpeed < 0.0) noisySpeed = 0.0;  // speed can't be negative

    // Decompose noisy speed back into X/Y using true heading
    double headingRad = trueState.heading * M_PI / 180.0;
    reading.velocityX = noisySpeed * std::cos(headingRad);
    reading.velocityY = noisySpeed * std::sin(headingRad);

    // Apply calibration if set
    if (getIsCalibrated()) {
        double off = getCalibrationOffset();
        double scl = getCalibrationScale();
        reading.velocityX = (reading.velocityX - off) * scl;
        reading.velocityY = (reading.velocityY - off) * scl;
    }

    setLastReading(reading);
    return reading;
}

void WheelEncoderSensor::calibrate(double offset, double scale) {
    Sensor::calibrate(offset, scale);
}

std::string WheelEncoderSensor::getType() const {
    return "WheelEncoder";
}
