#ifndef WHEEL_ENCODER_SENSOR_H
#define WHEEL_ENCODER_SENSOR_H

#include "Sensor.h"

/**
 * @brief Wheel encoder sensor — generates noisy velocity from wheel rotation.
 *
 * Uses wheel radius and ticks-per-revolution to convert rotation into
 * a velocity estimate. David (Jeongheon Han) — primary author.
 */
class WheelEncoderSensor : public Sensor {
private:
    double tickNoiseSigma;
    double tickBias;
    double wheelRadius;
    int    ticksPerRevolution;

public:
    WheelEncoderSensor(double sigma, double bias,
                       double radius, int ticks, double rate);

    StateEstimate generateReading(const StateEstimate& trueState) override;
    void calibrate(double offset, double scale) override;
    std::string getType() const override;
};

#endif // WHEEL_ENCODER_SENSOR_H
