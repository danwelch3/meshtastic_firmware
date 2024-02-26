#include "BMX160.h"

BMX160::BMX160()
{
    devAddr = BMX160_I2C_ADDR;
}
BMX160::BMX160(uint8_t address)
{
    devAddr = address;
    bmx160.setAddress(address);
}

void BMX160::initialize(uint8_t address)
{
    devAddr = address;
    bmx160.setAddress(address);
    initialize();
}
void BMX160::initialize()
{
    if (bmx160.begin() != true) {
        Serial.println("BMX init failed");
    }
}

uint8_t BMX160::getDeviceID()
{
    I2Cdev::readByte(devAddr, BMX160_CHIP_ID_ADDR, buffer);
    return buffer[0];
}

bool BMX160::testConnection()
{
    uint8_t deviceId = getDeviceID();
    if (deviceId == BMX160_CHIP_ID) {
        return true;
    } else {
        Serial.print("BMX160 ID Failed: ");
        Serial.println(deviceId);

        return false;
    }
}

void BMX160::calibrateMag()
{
    int N = 3000;

    Serial.print("Collecting ");
    Serial.print(N);
    Serial.println(" points for magnetometer calibration, 3/second");
    Serial.println("TURN SENSOR VERY SLOWLY AND CAREFULLY IN 3D");
    Serial.println("Starting in 1 seconds...");
    delay(1000);

    for (int i = 0; i < N; i++) {

        bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);

        mx = Omagn.x; // uT
        my = Omagn.y; // uT
        mz = Omagn.z; // uT

        if (mx >= mx_max)
            mx_max = mx;
        if (my >= my_max)
            my_max = my; // find max value
        if (mz >= mz_max)
            mz_max = mz;

        if (mx <= mx_min)
            mx_min = mx;
        if (my <= my_min)
            my_min = my; // find min value
        if (mz <= mz_min)
            mz_min = mz;

        delay(20);

        Serial.print("MAG = ");
        Serial.print(mx);
        Serial.print(",");
        Serial.print(my);
        Serial.print(",");
        Serial.print(mz);
        Serial.print("\n");

        // Serial.print("GYRO = ");
        // Serial.print(Ogyro.x);
        // Serial.print(",");
        // Serial.print(Ogyro.y);
        // Serial.print(",");
        // Serial.print(Ogyro.z);
        // Serial.print("\n");

        // Serial.print("ACCL = ");
        // Serial.print(Oaccel.x);
        // Serial.print(",");
        // Serial.print(Oaccel.y);
        // Serial.print(",");
        // Serial.print(Oaccel.z);
        // Serial.print("\n");
    }

    mx_centre = (mx_max + mx_min) / 2;
    my_centre = (my_max + my_min) / 2;
    mz_centre = (mz_max + mz_min) / 2;
    Serial.print("Done. ");
    Serial.print("Mag Center X=");
    Serial.print(mx_centre);
    Serial.print(" Y=");
    Serial.print(my_centre);
    Serial.print(" Z=");
    Serial.println(mz_centre);
}

void BMX160::update()
{

    bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);

    Mxyz[0] = Omagn.x; // uT
    Mxyz[1] = Omagn.y; // uT
    Mxyz[2] = Omagn.z; // uT

    // apply compass calibration
    Mxyz[0] = Mxyz[0] - mx_centre;
    Mxyz[1] = Mxyz[1] - my_centre;
    Mxyz[2] = Mxyz[2] - mz_centre;

    Gxyz[0] = Ogyro.x;
    Gxyz[1] = Ogyro.y;
    Gxyz[2] = Ogyro.z;

    Axyz[0] = Oaccel.x;
    Axyz[1] = Oaccel.y;
    Axyz[2] = Oaccel.z;
}

float BMX160::getHeading()
{
    float heading = (180 * atan2(Mxyz[0], Mxyz[1]) / PI) - 90 + 9;
    if (heading < 0)
        heading += 360;

    return heading;
}

float BMX160::getTiltHeading()
{
    float pitch = asin(-Axyz[0]);
    float roll = asin(Axyz[1] / cos(pitch));

    float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
    float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
    float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
    float tiltheading = 180 * atan2(yh, xh) / PI;
    if (yh < 0)
        tiltheading += 360;

    return tiltheading;
}

// void BMX160::loadCalibration()
// {
// #ifdef FSCom
//     auto file = FSCom.open(bmx160ConfigFileName, FILE_O_READ);
//     if (file) {
//         file.read((uint8_t *)&bmx160State, BMX160_MAX_STATE_BLOB_SIZE);
//         file.close();
//         bme680.setState(bsecState);
//         LOG_INFO("BMX160 config read from %s.\n", bmx160ConfigFileName);
//     } else {
//         LOG_INFO("No BMX160 config found (File: %s).\n", bmx160ConfigFileName);
//     }
// #else
//     LOG_ERROR("ERROR: Filesystem not implemented\n");
// #endif
// }

// void BMX160::saveCalibration()
// {
// #ifdef FSCom
//         std::string filenameTmp = bmx160ConfigFileName;
//         filenameTmp += ".tmp";
//         auto file = FSCom.open(bmx160ConfigFileName, FILE_O_WRITE);
//         if (file) {
//             LOG_INFO("BMX160 config write to %s.\n", bmx160ConfigFileName);
//             file.write((uint8_t *)&bsecState, BSEC_MAX_STATE_BLOB_SIZE);
//             file.flush();
//             file.close();
//             // brief window of risk here ;-)
//             if (FSCom.exists(bmx160ConfigFileName) && !FSCom.remove(bmx160ConfigFileName)) {
//                 LOG_WARN("Can't remove old config file\n");
//             }
//             if (!renameFile(filenameTmp.c_str(), bmx160ConfigFileName)) {
//                 LOG_ERROR("Error: can't rename new config file\n");
//             }

//         } else {
//             LOG_INFO("Can't write BMX160 config (File: %s).\n", bmx160ConfigFileName);
//         }
//     }
// #else
//     LOG_ERROR("ERROR: Filesystem not implemented\n");
// #endif
// }