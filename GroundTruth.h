#ifndef GROUNDTRUTH_H
#define GROUNDTRUTH_H
#include <utils.h>

class GroundTruth
{
private:
    int dataID;
    Quaternionf quaternion;
    Quaternionf prevQuaternion;
    Vector3f pose3D;
    float time;
    float prevTime;

public:
    GroundTruth();
    void init(ifstream& Data);
    void update(ifstream& Data);    
    int getID();
    Quaternionf getQuaternion();
    Quaternionf getPrevQuaternion();
    Vector3f getPose();
    float getTime();
    float getPrevTime();
    Matrix3f getCameraToWorldRotation();
    Matrix3f getPrevCameraToWorldRotation();
};

#endif // GROUNDTRUTH_H
