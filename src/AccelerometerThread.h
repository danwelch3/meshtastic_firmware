#pragma once
#include "PowerFSM.h"
#include "concurrency/OSThread.h"
#include "configuration.h"
#include "main.h"
#include "power.h"

#include <Adafruit_LIS3DH.h>
#include <Adafruit_MPU6050.h>
#ifndef BMX160
#include <modules/Telemetry/Sensor/BMX160.h>
#endif
#ifndef MPU9250
#include <modules/Telemetry/Sensor/MPU9250.h>
#endif

#include <Arduino.h>
#include <Wire.h>
#include <bma.h>

#define ACCELEROMETER_CHECK_INTERVAL_MS 100
#define ACCELEROMETER_CLICK_THRESHOLD 40

uint16_t readRegister(uint8_t address, uint8_t reg, uint8_t *data, uint16_t len);

uint16_t writeRegister(uint8_t address, uint8_t reg, uint8_t *data, uint16_t len);

namespace concurrency
{
class AccelerometerThread : public concurrency::OSThread
{
  public:
    AccelerometerThread(ScanI2C::DeviceType type = ScanI2C::DeviceType::NONE);

  protected:
    int32_t runOnce() override;

  private:
    void wakeScreen();

    void buttonPress();

    ScanI2C::DeviceType accelerometer_type;
    Adafruit_MPU6050 mpu;
    Adafruit_LIS3DH lis;
    BMX160 bmx160;
    MPU9250 mpu9250;
    BMA423 bmaSensor;
    bool BMA_IRQ = false;
};

} // namespace concurrency
