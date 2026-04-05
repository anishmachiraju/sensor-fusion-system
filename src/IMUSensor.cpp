#include "IMUSensor.h"

IMUSensor::IMUSensor(double accelSigma, double gyroSigma,
                     double aBias, double gBias, double rate)
    : Sensor("IMU", accelSigma, aBias, rate),
      accelNoiseSigma(accelSigma),
      gyroNoiseSigma(gyroSigma),
      accelBias(aBias),
      gyroBias(gBias)
{
}

StateEstimate IMUSensor::generateReading(const StateEstimate& trueState) {
    StateEstimate reading = trueState;

    // Perturb velocity (acceleration proxy) with IMU-specific noise
    reading.velocityX += sampleNoise(accelNoiseSigma) + accelBias;
    reading.velocityY += sampleNoise(accelNoiseSigma) + accelBias;

    // Perturb heading (gyro)
    reading.heading += sampleNoise(gyroNoiseSigma) + gyroBias;

    // Apply calibration if set
    if (getIsCalibrated()) {
        double off = getCalibrationOffset();
        double scl = getCalibrationScale();
        reading.velocityX = (reading.velocityX - off) * scl;
        reading.velocityY = (reading.velocityY - off) * scl;
        reading.heading   = (reading.heading   - off) * scl;
    }

    setLastReading(reading);
    return reading;
}

void IMUSensor::calibrate(double offset, double scale) {
    Sensor::calibrate(offset, scale);
}

std::string IMUSensor::getType() const {
    return "IMU";
}
