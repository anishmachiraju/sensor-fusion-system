#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include "Sensor.h"

/**
 * @brief Temperature sensor subclass — generates noisy temperature readings.
 *
 * Overrides generateReading() to perturb only the temperature field.
 * Includes a simple response-delay model. David (Jeongheon Han) — primary author.
 */
class TemperatureSensor : public Sensor {
private:
    double tempNoiseSigma;
    double tempBias;
    double responseDelay;   // lag factor in [0, 1]
    double prevTemp;        // tracks delayed response

public:
    TemperatureSensor(double sigma, double bias, double delay, double rate);

    StateEstimate generateReading(const StateEstimate& trueState) override;
    void calibrate(double offset, double scale) override;
    std::string getType() const override;
};

#endif // TEMPERATURE_SENSOR_H
