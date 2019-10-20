#ifndef IMUCALIB_H
#define IMUCALIB_H
#include <utils.h>
using namespace std;

class IMUCalib
{
public:
    //! Nam - static is necessary, don't know why
    static string groundTruthFile;
    static string IMUFile;

    //! IMU to camera
    static Matrix3f IMUtoCameraRotation;
    static Vector3f IMUtoCameraTranslation;
};

#endif // IMUCALIB_H

