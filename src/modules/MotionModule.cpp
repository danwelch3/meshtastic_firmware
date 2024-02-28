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
    } else if (accelerometer_type == ScanI2C::DeviceType::BNO08x) {
        LOG_DEBUG("BNO08x initializing\n");
        bno08x.initialize(use_accelerometer.address);
        if (bno08x.testConnection()) {
            hasCompass = true;
            LOG_DEBUG("BNO08x sucessfully initialized\n");
        } else {
            LOG_DEBUG("BNO08x failed to initialize");
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
        MahonyQuaternionUpdate(mpu9250.Axyz[0], mpu9250.Axyz[1], mpu9250.Axyz[2],
                               mpu9250.Gxyz[0] * DEG_TO_RAD, mpu9250.Gxyz[1] * DEG_TO_RAD, mpu9250.Gxyz[2] * DEG_TO_RAD,
                               mpu9250.Mxyz[0], mpu9250.Mxyz[1], mpu9250.Mxyz[2],
                               mpu9250.deltat);

        MotionModule::heading = mpu9250.getHeading();
        // LOG_DEBUG("MAG CENTER (%.1f, %.1f, %.1f)\n", mpu9250.mx_centre, mpu9250.my_centre, mpu9250.mz_centre);
    } else if (accelerometer_type == ScanI2C::DeviceType::BNO08x) {
        bno08x.update();
        MotionModule::heading = bno08x.getHeading();
    }

    yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                * *(getQ()+3));
    pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                * *(getQ()+2)));
    roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                * *(getQ()+3));
    pitch *= RAD_TO_DEG;
    yaw   *= RAD_TO_DEG;

    // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
    // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    yaw  -= 8.5;
    roll *= RAD_TO_DEG;

    LOG_DEBUG("[Motion] Pitch=%.1f Yaw=%.1f Roll=%.1f\n", pitch, yaw, roll);
}

float MotionModule::getHeading()
{
    if (accelerometer_type == ScanI2C::DeviceType::BMX160) {
        bmx160.update();
        return bmx160.getHeading();
    } else if (accelerometer_type == ScanI2C::DeviceType::MPU9250) {
        mpu9250.update();
        return mpu9250.getHeading();
    } else if (accelerometer_type == ScanI2C::DeviceType::BNO08x) {
        bno08x.update();
        return bno08x.getHeading();
    }

    return 0.0;
}

void MotionModule::calibrate()
{
    if (accelerometer_type == ScanI2C::DeviceType::BMX160) {
        bmx160.calibrateMag();
    } else if (accelerometer_type == ScanI2C::DeviceType::MPU9250) {
        mpu9250.calibrateMag();
    } else if (accelerometer_type == ScanI2C::DeviceType::BNO08x) {
        bno08x.calibrateMag();
    }
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