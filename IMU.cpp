#include <IMU.h>
#include <utils.h>

IMU::IMU()
{
}

void IMU::init(ifstream& imuData, Matrix3f& R, Vector3f& T)
{
    imuDataID = 1; // data id starts from 1
    imuData >> imuTime >> angularVel(0) >> angularVel(1) >> angularVel(2) >> accel(0) >> accel(1) >> accel(2);
    prevIMUTime = imuTime;
    IMUtoCameraRotation = R;
    IMUtoCameraTranslation = T;
}

void IMU::update(ifstream& imuData)
{
    imuDataID++;
    prevIMUTime = imuTime;
    if (!imuData.eof())
        imuData >> imuTime >> angularVel(0) >> angularVel(1) >> angularVel(2) >> accel(0) >> accel(1) >> accel(2);
    else
        cout << "no more data from IMU " << endl;
}

int IMU::getID()
{
    return imuDataID;
}

Vector3f IMU::getAngularVel()
{
    return angularVel;
}

Vector3f IMU::getAccel()
{
    return accel;
}

float IMU::getTime()
{
    return imuTime;
}

float IMU::getPrevTime()
{
    return prevIMUTime;
}

Matrix3f IMU::getIMUtoCameraRotation()
{
    return IMUtoCameraRotation;
}

Vector3f IMU::getIMUtoCameraTranslation()
{
    return IMUtoCameraTranslation;
}




