#include "BNO08x.h"

struct euler_t {
    float yaw;
    float pitch;
    float roll;
} ypr;

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

BNO08x::BNO08x()
{
    devAddr = 0x4B;
}
BNO08x::BNO08x(uint8_t address)
{
    devAddr = address;
}

bool BNO08x::initialize(uint8_t address, TwoWire &wirePort)
{
    devAddr = address;
    _wire = &wirePort;
    initialize();
}
bool BNO08x::initialize()
{
    Serial.println("BNO will begin");
    // bno080.enableDebugging();
    if (bno080.begin_I2C(devAddr) == true) {
        Serial.println("Setting desired reports");
        if (! bno080.enableReport(SH2_ARVR_STABILIZED_RV, 10000)) {
            Serial.println("Could not enable stabilized remote vector");
        }
        if (! bno080.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 10000)) {
            Serial.println("Could not enable calibrated magnetic field");
        }
        // bno080.calibrateAll();
        Serial.println("BNO init success");
    } else {
        Serial.println("BNO init failed");
    }
}

uint8_t BNO08x::getDeviceID()
{
    // // Initialize the Tx buffer
    // _wire->beginTransmission(devAddr);
    // // Put slave register address in Tx buffer
    // _wire->write(SHTP_REPORT_PRODUCT_ID_REQUEST);
    // // Send the Tx buffer, but send a restart to keep connection alive
    // _wire->endTransmission(false);
    // // Read one byte from slave register address
    // _wire->requestFrom(devAddr, (uint8_t)1);
    // // Fill Rx buffer with result
    // buffer[0] = _wire->read();

    // return buffer[0];
    return 1;
}

bool BNO08x::testConnection()
{
    // uint8_t deviceId = getDeviceID();
    // if (deviceId == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
    //     return true;
    // } else {
    //     Serial.print("BNO08x ID Failed: ");
    //     Serial.println(deviceId);

    //     return false;
    // }

    return true;
}

void BNO08x::calibrateMag()
{
    // bno080.calibrateMagnetometer();
}

void BNO08x::update()
{
    if (bno080.getSensorEvent(&sensorValue)) {
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        heading = ypr.yaw;
        if (heading < 0)
            heading += 360;
        // switch (sensorValue.sensorId) {
        //     case SH2_ARVR_STABILIZED_RV:
        //         quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        //         heading = ypr.yaw;
        //         if (heading < 0)
        //             heading += 360;
        //         break;
        //     case SH2_MAGNETIC_FIELD_CALIBRATED:
        //         Mxyz[0] = sensorValue.un.magneticField.x;
        //         Mxyz[1] = sensorValue.un.magneticField.y;
        //         Mxyz[2] = sensorValue.un.magneticField.z;
        //         break;
        // }
    }
}

float BNO08x::getHeading()
{
    return heading;
}

float BNO08x::getTiltHeading()
{
    return heading;
}