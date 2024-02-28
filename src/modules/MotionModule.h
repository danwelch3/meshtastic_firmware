#ifndef BMX160
#include <modules/Telemetry/Sensor/BMX160.h>
#endif
#ifndef MPU9250
#include <modules/Telemetry/Sensor/MPU9250.h>
#endif
#ifndef BNO08x
#include <modules/Telemetry/Sensor/BNO08x.h>
#endif
#include <modules/Telemetry/Sensor/quaternionFilters.h>

#include "detect/ScanI2C.h"
#include "detect/ScanI2CTwoWire.h"
#include <Arduino.h>
#include <Wire.h>

/**
 * @struct sensorData
 * @brief sensor data structure with x, y, z and time
 */
typedef struct {
    float x;             /**< X-axis sensor data */
    float y;             /**< Y-axis sensor data */
    float z;             /**< Z-axis sensor data */
    uint32_t sensortime; /**< sensor time */
} sensorData;

/**
 * @struct allSensorData
 * @brief sensor data structure with x, y, z and time
 */
typedef struct {
    sensorData magData;   /**< magnetometer data struct */
    sensorData gyroData;  /**< gyroscope data struct */
    sensorData accelData; /**< accelerometer data struct */
    float temp;
} allSensorData;

/**
 * @struct positionData
 * @brief sensor data structure with pitch, yaw, and roll
 */
typedef struct {
    float pitch; /**< pitch data */
    float yaw;   /**< yaw sensor data */
    float roll;  /**< roll sensor data */
} pyrData;

class MotionModule
{
  public:
    MotionModule();

    void init();
    void updateData();
    float getHeading();
    void calibrate();
    // float getPitch();
    // float getYaw();
    // float getRoll();
    // sensorData getMagDeg();

    ScanI2C::DeviceType accelerometer_type;
    allSensorData currentData;
    pyrData pyr;
    float heading = 0.0;
    bool hasCompass;

    float pitch, yaw, roll, temperature;    

  protected:
    BMX160 bmx160;
    MPU9250 mpu9250;
    BNO08x bno08x;
    float minMag[3];
    float maxMag[3];
};

extern MotionModule *motionModule;