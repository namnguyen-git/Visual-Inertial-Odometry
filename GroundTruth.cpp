#include <GroundTruth.h>
#include <utils.h>

GroundTruth::GroundTruth()
{
}

void GroundTruth::init(ifstream& Data)
{
    dataID = 1; // data id starts from 1
    Data >> time >> pose3D(0) >> pose3D(1) >> pose3D(2) >> quaternion.x() >> quaternion.y() >> quaternion.z() >> quaternion.w();
    prevTime = time;
    prevQuaternion = quaternion;
}

void GroundTruth::update(ifstream& Data)
{
    dataID++;
    prevTime = time;
    prevQuaternion = quaternion;
    if (!Data.eof())
        Data >> time >> pose3D(0) >> pose3D(1) >> pose3D(2) >> quaternion.x() >> quaternion.y() >> quaternion.z() >> quaternion.w();
    else
        cout << "no more data from groundtruth " << endl;
}

int GroundTruth::getID()
{
    return dataID;
}

Quaternionf GroundTruth::getQuaternion()
{
    return quaternion;
}

Quaternionf GroundTruth::getPrevQuaternion()
{
    return prevQuaternion;
}

Vector3f GroundTruth::getPose()
{
    return pose3D;
}

float GroundTruth::getTime()
{
    return time;
}

float GroundTruth::getPrevTime()
{
    return prevTime;
}

Matrix3f GroundTruth::getCameraToWorldRotation()
{
    return quaternion.toRotationMatrix();
}

Matrix3f GroundTruth::getPrevCameraToWorldRotation()
{
    return prevQuaternion.toRotationMatrix();
}


