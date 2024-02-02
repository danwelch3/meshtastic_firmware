#include "MotionModule.h"
#include "configuration.h"

#include <algorithm>

// uint16_t readRegister(uint8_t address, uint8_t reg, uint8_t *data, uint16_t len)
// {
//     Wire.beginTransmission(address);
//     Wire.write(reg);
//     Wire.endTransmission();
//     Wire.requestFrom((uint8_t)address, (uint8_t)len);
//     uint8_t i = 0;
//     while (Wire.available()) {
//         data[i++] = Wire.read();
//     }
//     return 0; // Pass
// }

// uint16_t writeRegister(uint8_t address, uint8_t reg, uint8_t *data, uint16_t len)
// {
//     Wire.beginTransmission(address);
//     Wire.write(reg);
//     Wire.write(data, len);
//     return (0 != Wire.endTransmission());
// }

MotionModule *motionModule;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

ScanI2C::DeviceAddress use_accelerometer = ScanI2C::ADDRESS_NONE;
// ScanI2C::DeviceType accelerometer_type = ScanI2C::DeviceType::NONE;

MotionModule::MotionModule() {}

void MotionModule::init()
{
    hasCompass = false;
    auto i2cScanner = std::unique_ptr<ScanI2CTwoWire>(new ScanI2CTwoWire());
#ifdef I2C_SDA1
    Wire1.begin(I2C_SDA1, I2C_SCL1);
    i2cScanner->scanPort(ScanI2C::I2CPort::WIRE1);
#endif

#ifdef I2C_SDA
    Wire.begin(I2C_SDA, I2C_SCL);
    i2cScanner->scanPort(ScanI2C::I2CPort::WIRE);
#elif HAS_WIRE
    i2cScanner->scanPort(ScanI2C::I2CPort::WIRE);
#endif

    auto acc_info = i2cScanner->firstAccelerometer();
    accelerometer_type = acc_info.type;
    use_accelerometer = acc_info.type != ScanI2C::DeviceType::NONE ? acc_info.address : use_accelerometer;

    if (use_accelerometer.port == ScanI2C::I2CPort::NO_I2C) {
        LOG_DEBUG("MotionModule disabling due to no sensors found\n");
        // disable();
        return;
    }

    LOG_DEBUG("MotionModule initializing\n");

    if (accelerometer_type == ScanI2C::DeviceType::BMX160) {
        LOG_DEBUG("BMX160 initializing\n");
        bmx160.initialize(use_accelerometer.address);
        if (bmx160.testConnection()) {
            hasCompass = true;
            LOG_DEBUG("BMX160 sucessfully initialized\n");
            // bmx160.calibrateMag();
        } else {
            LOG_DEBUG("BMX160 failed to initialize\n");
        }
    } else if (accelerometer_type == ScanI2C::DeviceType::MPU9250) {
        LOG_DEBUG("MPU9250 initializing\n");
        mpu9250.initialize();
        if (mpu9250.testConnection()) {
            hasCompass = true;
            LOG_DEBUG("MPU9250 sucessfully initialized\n");
            // mpu9250.calibrateMag();
        } else {
            LOG_DEBUG("MPU9250 failed to initialize\n");
        }
    }
}

void MotionModule::updateData()
{
    // LOG_DEBUG("UPDATING MAG DATA\n");
    if (accelerometer_type == ScanI2C::DeviceType::BMX160) {
        bmx160.update();
        MotionModule::heading = bmx160.getHeading();
        // LOG_DEBUG("MAG CENTER (%.1f, %.1f, %.1f)\n", bmx160.mx_centre, bmx160.my_centre, bmx160.mz_centre);
    } else if (accelerometer_type == ScanI2C::DeviceType::MPU9250) {
        mpu9250.update();
        MotionModule::heading = mpu9250.getHeading();
        // LOG_DEBUG("MAG CENTER (%.1f, %.1f, %.1f)\n", mpu9250.mx_centre, mpu9250.my_centre, mpu9250.mz_centre);
    }
}

float MotionModule::getHeading()
{
    if (accelerometer_type == ScanI2C::DeviceType::BMX160) {
        bmx160.update();
        return bmx160.getHeading();
    } else if (accelerometer_type == ScanI2C::DeviceType::MPU9250) {
        mpu9250.update();
        return mpu9250.getHeading();
    }

    return 0.0;
}

// float MotionModule::getPitch() {
//     if (accelerometer_type == ScanI2C::DeviceType::BMX160) {
//         // TO Do
//     } else if (accelerometer_type == ScanI2C::DeviceType::MPU9250) {
//         mpu9250.update();
//         return mpu9250.getPitch();
//     }

//     return 0.0;
// }

// float MotionModule::getYaw() {
//     if (accelerometer_type == ScanI2C::DeviceType::BMX160) {
//         // TO Do
//     } else if (accelerometer_type == ScanI2C::DeviceType::MPU9250) {
//         mpu9250.update();
//         return mpu9250.getYaw();
//     }

//     return 0.0;
// }

// float MotionModule::getRoll() {
//     if (accelerometer_type == ScanI2C::DeviceType::BMX160) {
//         // TO Do
//     } else if (accelerometer_type == ScanI2C::DeviceType::MPU9250) {
//         mpu9250.update();
//         return mpu9250.getRoll();
//     }

//     return 0.0;
// }

// sensorData MotionModule::getMagDeg() {
//     float spanX = maxMag[0] - minMag[0];
//     float spanY = maxMag[1] - minMag[1];
//     float spanZ = maxMag[2] - minMag[2];

//     sensorData data;
//     data.x = (int)(360 * ((currentData.magData.x - minMag[0]) / spanX));
//     data.y = (int)(360 * ((currentData.magData.y - minMag[1]) / spanY));
//     data.z = (int)(360 * ((currentData.magData.z - minMag[2]) / spanZ));

//     return data;
// }