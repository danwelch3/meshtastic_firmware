#pragma once
#ifndef BMX160_H
#define BMX160_H

#include "BMX160/DFRobot_BMX160.h"
#include <Arduino.h>
#include <Wire.h>

class BMX160
{
  public:
    BMX160();
    BMX160(uint8_t address);

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
    uint8_t buffer_m[6];

    sBmx160SensorData_t Omagn, Ogyro, Oaccel;

    float Axyz[3];
    float Gxyz[3];
    float Mxyz[3];

    // float mx_centre;
    // float my_centre;
    // float mz_centre;

    // // Orange
    float mx_centre = 249.0;
    float my_centre = 229.0;
    float mz_centre = -280.0;

    volatile int mx_max = 0;
    volatile int my_max = 0;
    volatile int mz_max = 0;

    volatile int mx_min = 0;
    volatile int my_min = 0;
    volatile int mz_min = 0;

    DFRobot_BMX160 bmx160;
    bool did_init = false;

    //     static constexpr uint8_t MAG_MODE {0x06};  // 0x02 for 8 Hz, 0x06 for 100 Hz continuous magnetometer data read
    //     float acc_resolution {0.f};                // scale resolutions per LSB for the sensors
    //     float gyro_resolution {0.f};               // scale resolutions per LSB for the sensors
    //     float mag_resolution {0.f};                // scale resolutions per LSB for the sensors

    //     // Calibration Parameters
    //     float acc_bias[3] {0., 0., 0.};   // acc calibration value in ACCEL_FS_SEL: 2g
    //     float gyro_bias[3] {0., 0., 0.};  // gyro calibration value in GYRO_FS_SEL: 250dps
    //     float mag_bias_factory[3] {0., 0., 0.};
    //     float mag_bias[3] {0., 0., 0.};  // mag calibration value in MAG_OUTPUT_BITS: 16BITS
    //     float mag_scale[3] {1., 1., 1.};
    //     float magnetic_declination = -7.51;  // Japan, 24th June

    //     // Temperature
    //     int16_t temperature_count {0};  // temperature raw count output
    //     float temperature {0.f};        // Stores the real internal chip temperature in degrees Celsius

    //     // Self Test
    //     float self_test_result[6] {0.f};  // holds results of gyro and accelerometer self test

    //     // IMU Data
    //     float a[3] {0.f, 0.f, 0.f};
    //     float g[3] {0.f, 0.f, 0.f};
    //     float m[3] {0.f, 0.f, 0.f};
    //     float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // vector to hold quaternion
    //     float rpy[3] {0.f, 0.f, 0.f};
    //     float lin_acc[3] {0.f, 0.f, 0.f};  // linear acceleration (acceleration with gravity component subtracted)
    //     // QuaternionFilter quat_filter;
    //     size_t n_filter_iter {1};

    //     // Other settings
    //     bool has_connected {false};
    //     bool b_ahrs {true};
    //     bool b_verbose {false};

    //     // I2C
    //     WireType* wire;
    //     uint8_t i2c_err_;

    // public:
    //     static constexpr uint16_t CALIB_GYRO_SENSITIVITY {131};     // LSB/degrees/sec
    //     static constexpr uint16_t CALIB_ACCEL_SENSITIVITY {16384};  // LSB/g

    //     bool setup(const uint8_t addr) {
    //         /* Status of api are returned to this variable */
    //         int8_t rslt;

    //         /* Sensor initialization configuration. */
    //         struct bmm150_dev dev;
    //         struct bmm150_settings settings;

    //         dev.intf = BMM150_I2C_INTF;

    //         dev_addr = BMM150_DEFAULT_I2C_ADDRESS;
    //         // dev.read = bmm150_user_i2c_reg_read;
    //         // dev.write = bmm150_user_i2c_reg_write;

    //         /* Assign device address to interface pointer */
    //         dev.int_status = &dev_addr;

    //         /* Configure delay in microseconds */
    //         // dev.delay_us = bmm150_user_delay_us;

    //         settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    //         rslt = bmm150_set_op_mode(&settings, &dev);
    //         Serial.print("BMM150 pwr_mode = ");
    //         Serial.println(rslt);

    //         settings.preset_mode = BMM150_PRESETMODE_LOWPOWER;
    //         rslt = bmm150_set_presetmode(&settings, &dev);
    //         Serial.print("BMM150 preset_mode = ");
    //         Serial.println(rslt);

    //         settings.int_settings.drdy_pin_en = 0x01;
    //         rslt = bmm150_set_sensor_settings(BMM150_SEL_DRDY_PIN_EN, &settings, &dev);
    //         Serial.print("BMM150 drdy_pin_en = ");
    //         Serial.println(rslt);

    //         return true;
    //     };

    //     // WHO_AM_I register
    //         uint8_t getDeviceID();
    //         void setDeviceID(uint8_t id);
    //     getDeviceID() {
    //     // I2Cdev::readBits(devAddr, MPU9250_RA_WHO_AM_I, MPU9250_WHO_AM_I_BIT, MPU9250_WHO_AM_I_LENGTH, buffer);
    //     I2Cdev::readByte(devAddr, MPU9250_RA_WHO_AM_I, buffer);
    //     return buffer[0];
    // }

    //     bool testConnection() {
    //         uint8_t deviceId = getDeviceID();
    //         if (deviceId == 0x71) {
    //             return true;
    //         } else {
    //             Serial.print("MPU9250 ID Failed: ");
    //             Serial.println(deviceId);

    //             return false;
    //         }
    //         // return getDeviceID() == 0x71;
    //     }

    //     bool testConnection() {
    //         has_connected = isConnectedBMI160() && isConnectedBMM150();
    //         return has_connected;
    //     }

    //     bool isConnectedBMI160() {
    //         byte c = read_byte(mpu_i2c_addr, WHO_AM_I_MPU9250);
    //         if (b_verbose) {
    //             Serial.print("MPU9250 WHO AM I = ");
    //             Serial.println(c, HEX);
    //         }
    //         bool b = (c == MPU9250_WHOAMI_DEFAULT_VALUE);
    //         b |= (c == MPU9255_WHOAMI_DEFAULT_VALUE);
    //         b |= (c == MPU6500_WHOAMI_DEFAULT_VALUE);
    //         return b;
    //     }

    //     bool isConnectedBMM150() {
    //         byte c = read_byte(AK8963_ADDRESS, AK8963_WHO_AM_I);
    //         if (b_verbose) {
    //             Serial.print("AK8963 WHO AM I = ");
    //             Serial.println(c, HEX);
    //         }
    //         return (c == AK8963_WHOAMI_DEFAULT_VALUE);
    //     }

    // bool available() {
    //     return has_connected && (read_byte(mpu_i2c_addr, INT_STATUS) & 0x01);
    // }

    // bool update() {
    //     if (!available()) return false;

    //     update_accel_gyro();
    //     update_mag();

    //     // Madgwick function needs to be fed North, East, and Down direction like
    //     // (AN, AE, AD, GN, GE, GD, MN, ME, MD)
    //     // Accel and Gyro direction is Right-Hand, X-Forward, Z-Up
    //     // Magneto direction is Right-Hand, Y-Forward, Z-Down
    //     // So to adopt to the general Aircraft coordinate system (Right-Hand, X-Forward, Z-Down),
    //     // we need to feed (ax, -ay, -az, gx, -gy, -gz, my, -mx, mz)
    //     // but we pass (-ax, ay, az, gx, -gy, -gz, my, -mx, mz)
    //     // because gravity is by convention positive down, we need to ivnert the accel data

    //     // get quaternion based on aircraft coordinate (Right-Hand, X-Forward, Z-Down)
    //     // acc[mg], gyro[deg/s], mag [mG]
    //     // gyro will be convert from [deg/s] to [rad/s] inside of this function
    //     // quat_filter.update(-a[0], a[1], a[2], g[0] * DEG_TO_RAD, -g[1] * DEG_TO_RAD, -g[2] * DEG_TO_RAD, m[1], -m[0], m[2],
    //     q);

    //     float an = -a[0];
    //     float ae = +a[1];
    //     float ad = +a[2];
    //     float gn = +g[0] * DEG_TO_RAD;
    //     float ge = -g[1] * DEG_TO_RAD;
    //     float gd = -g[2] * DEG_TO_RAD;
    //     float mn = +m[1];
    //     float me = -m[0];
    //     float md = +m[2];

    //     for (size_t i = 0; i < n_filter_iter; ++i) {
    //         quat_filter.update(an, ae, ad, gn, ge, gd, mn, me, md, q);
    //     }

    //     if (!b_ahrs) {
    //         temperature_count = read_temperature_data();               // Read the adc values
    //         temperature = ((float)temperature_count) / 333.87 + 21.0;  // Temperature in degrees Centigrade
    //     } else {
    //         update_rpy(q[0], q[1], q[2], q[3]);
    //     }
    //     return true;
    // }

    // float getRoll() const { return rpy[0]; }
    // float getPitch() const { return rpy[1]; }
    // float getYaw() const { return rpy[2]; }

    // float getEulerX() const { return rpy[0]; }
    // float getEulerY() const { return -rpy[1]; }
    // float getEulerZ() const { return -rpy[2]; }

    // float getQuaternionX() const { return q[1]; }
    // float getQuaternionY() const { return q[2]; }
    // float getQuaternionZ() const { return q[3]; }
    // float getQuaternionW() const { return q[0]; }

    // float getAcc(const uint8_t i) const { return (i < 3) ? a[i] : 0.f; }
    // float getGyro(const uint8_t i) const { return (i < 3) ? g[i] : 0.f; }
    // float getMag(const uint8_t i) const { return (i < 3) ? m[i] : 0.f; }
    // float getLinearAcc(const uint8_t i) const { return (i < 3) ? lin_acc[i] : 0.f; }

    // float getAccX() const { return a[0]; }
    // float getAccY() const { return a[1]; }
    // float getAccZ() const { return a[2]; }
    // float getGyroX() const { return g[0]; }
    // float getGyroY() const { return g[1]; }
    // float getGyroZ() const { return g[2]; }
    // float getMagX() const { return m[0]; }
    // float getMagY() const { return m[1]; }
    // float getMagZ() const { return m[2]; }
    // float getLinearAccX() const { return lin_acc[0]; }
    // float getLinearAccY() const { return lin_acc[1]; }
    // float getLinearAccZ() const { return lin_acc[2]; }

    // float getAccBias(const uint8_t i) const { return (i < 3) ? acc_bias[i] : 0.f; }
    // float getGyroBias(const uint8_t i) const { return (i < 3) ? gyro_bias[i] : 0.f; }
    // float getMagBias(const uint8_t i) const { return (i < 3) ? mag_bias[i] : 0.f; }
    // float getMagScale(const uint8_t i) const { return (i < 3) ? mag_scale[i] : 0.f; }

    // float getAccBiasX() const { return acc_bias[0]; }
    // float getAccBiasY() const { return acc_bias[1]; }
    // float getAccBiasZ() const { return acc_bias[2]; }
    // float getGyroBiasX() const { return gyro_bias[0]; }
    // float getGyroBiasY() const { return gyro_bias[1]; }
    // float getGyroBiasZ() const { return gyro_bias[2]; }
    // float getMagBiasX() const { return mag_bias[0]; }
    // float getMagBiasY() const { return mag_bias[1]; }
    // float getMagBiasZ() const { return mag_bias[2]; }
    // float getMagScaleX() const { return mag_scale[0]; }
    // float getMagScaleY() const { return mag_scale[1]; }
    // float getMagScaleZ() const { return mag_scale[2]; }

    // float getTemperature() const { return temperature; }

    // void setAccBias(const float x, const float y, const float z) {
    //     acc_bias[0] = x;
    //     acc_bias[1] = y;
    //     acc_bias[2] = z;
    //     write_accel_offset();
    // }
    // void setGyroBias(const float x, const float y, const float z) {
    //     gyro_bias[0] = x;
    //     gyro_bias[1] = y;
    //     gyro_bias[2] = z;
    //     write_gyro_offset();
    // }
    // void setMagBias(const float x, const float y, const float z) {
    //     mag_bias[0] = x;
    //     mag_bias[1] = y;
    //     mag_bias[2] = z;
    // }
    // void setMagScale(const float x, const float y, const float z) {
    //     mag_scale[0] = x;
    //     mag_scale[1] = y;
    //     mag_scale[2] = z;
    // }
    // void setMagneticDeclination(const float d) { magnetic_declination = d; }
};

#endif // BMX160_H