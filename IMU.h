#ifndef IMU_H
#define IMU_H
#include <utils.h>

class IMU
{
private:
    int imuDataID;
    Vector3f angularVel;
    Vector3f accel;
    float imuTime;
    float prevIMUTime;
    Matrix3f IMUtoCameraRotation;
    Vector3f IMUtoCameraTranslation;

public:
    IMU();
    void init(ifstream& imuData, Matrix3f& R, Vector3f& T);
    void update(ifstream& imuData);
    int getID();
    Vector3f getAngularVel();
    Vector3f getAccel();
    float getTime();
    float getPrevTime();
    Matrix3f getIMUtoCameraRotation();
    Vector3f getIMUtoCameraTranslation();

};

#endif // IMU_H
