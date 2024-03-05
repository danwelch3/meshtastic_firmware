#pragma once
#ifndef BNO080_H
#define BNO080_H

#include "BNO080/SparkFun_BNO080_Arduino_Library.h"
#include <Arduino.h>
#include <Wire.h>

class BNO08x
{
  public:
    BNO08x();
    BNO08x(uint8_t address);

    void initialize();
    void initialize(uint8_t address, TwoWire &wirePort = Wire);

    uint8_t getDeviceID();
    bool testConnection();
    void calibrateMag();

    void update();
    float getHeading();
    float getTiltHeading();

    int16_t getTemperature();

    // void loadCalibration();
    // void saveCalibration();

    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;

  protected:
    TwoWire *_wire; // Allows for use of various I2C ports

  private:
    uint8_t devAddr;
    uint8_t buffer[14];
    // uint8_t buffer_m[6];

    float Axyz[3];
    float Gxyz[3];
    float Mxyz[3];

    float mx_centre;
    float my_centre;
    float mz_centre;

    // Orange
    // float mx_centre = 249.0;
    // float my_centre = 229.0;
    // float mz_centre = -280.0;

    // volatile int mx_max = 0;
    // volatile int my_max = 0;
    // volatile int mz_max = 0;

    // volatile int mx_min = 0;
    // volatile int my_min = 0;
    // volatile int mz_min = 0;

    BNO080 bno080;
    bool did_init = false;
};

#endif // BNO08x_H