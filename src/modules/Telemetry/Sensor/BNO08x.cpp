#include "BNO08x.h"

BNO08x::BNO08x()
{
    devAddr = BNO080_DEFAULT_ADDRESS;
}
BNO08x::BNO08x(uint8_t address)
{
    devAddr = address;
}

void BNO08x::initialize(uint8_t address)
{
    devAddr = address;
    initialize();
}
void BNO08x::initialize()
{
    if (bno080.begin(devAddr) == true) {
        bno080.calibrateAll();
        Serial.println("BNO init success");
    } else {
        Serial.println("BNO init failed");
    }
}

uint8_t BNO08x::getDeviceID()
{
    I2Cdev::readByte(devAddr, SHTP_REPORT_PRODUCT_ID_REQUEST, buffer);
    return buffer[0];
}

bool BNO08x::testConnection()
{
    uint8_t deviceId = getDeviceID();
    if (deviceId == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
        return true;
    } else {
        Serial.print("BNO08x ID Failed: ");
        Serial.println(deviceId);

        return false;
    }
}

void BNO08x::calibrateMag()
{
    bno080.calibrateMagnetometer();
}

void BNO08x::update()
{
    if (bno080.dataAvailable() == true)
    {
        Mxyz[0] = bno080.getMagX();
        Mxyz[1] = bno080.getMagY();
        Mxyz[2] = bno080.getMagZ();
        // byte accuracy = bno080.getMagAccuracy();

        // float quatI = bno080.getQuatI();
        // float quatJ = bno080.getQuatJ();
        // float quatK = bno080.getQuatK();
        // float quatReal = bno080.getQuatReal();
        // byte sensorAccuracy = bno080.getQuatAccuracy();
    }
}

float BNO08x::getHeading()
{
    float heading = (180 * atan2(Mxyz[0], Mxyz[1]) / PI) - 90 + 9;
    if (heading < 0)
        heading += 360;

    return heading;
}

float BNO08x::getTiltHeading()
{
    return bno080.getYaw();
}