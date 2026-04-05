#include "TemperatureSensor.h"

TemperatureSensor::TemperatureSensor(double sigma, double bias,
                                     double delay, double rate)
    : Sensor("Temperature", sigma, bias, rate),
      tempNoiseSigma(sigma),
      tempBias(bias),
      responseDelay(delay),
      prevTemp(20.0)
{
}

StateEstimate TemperatureSensor::generateReading(const StateEstimate& trueState) {
    StateEstimate reading = trueState;

    // Simulate slow thermal response: blend previous reading with true value
    double rawTemp = trueState.temperature;
    double delayedTemp = prevTemp + (1.0 - responseDelay) * (rawTemp - prevTemp);

    // Add noise and bias
    reading.temperature = delayedTemp + sampleNoise(tempNoiseSigma) + tempBias;

    // Apply calibration if set
    if (getIsCalibrated()) {
        double off = getCalibrationOffset();
        double scl = getCalibrationScale();
        reading.temperature = (reading.temperature - off) * scl;
    }

    prevTemp = delayedTemp;
    setLastReading(reading);
    return reading;
}

void TemperatureSensor::calibrate(double offset, double scale) {
    Sensor::calibrate(offset, scale);
}

std::string TemperatureSensor::getType() const {
    return "Temperature";
}
