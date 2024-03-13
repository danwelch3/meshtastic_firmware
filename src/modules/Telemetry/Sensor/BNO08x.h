#pragma once
#ifndef BNO080_H
#define BNO080_H

#include "BNO080/Adafruit_BNO08x.h"
#include <Arduino.h>
#include <Wire.h>

class BNO08x
{
  public:
    BNO08x();
    BNO08x(uint8_t address);

    bool initialize();
    bool initialize(uint8_t address, TwoWire &wirePort = Wire);

    uint8_t getDeviceID();
    bool testConnection();
    void calibrateMag();

    void update();
    float getHeading();
    float getTiltHeading();

    int16_t getTemperature();

    // float Mxyz[3] = {0};
    float heading = 0;

  protected:
    TwoWire *_wire; // Allows for use of various I2C ports

  private:
    uint8_t devAddr;
    uint8_t buffer[14];

    float mx_centre;
    float my_centre;
    float mz_centre;

    Adafruit_BNO08x  bno080;
    bool did_init = false;

    sh2_SensorValue_t sensorValue;
};

#endif // BNO08x_H