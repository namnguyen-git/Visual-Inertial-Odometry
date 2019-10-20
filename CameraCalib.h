#ifndef CAMERACALIB_H
#define CAMERACALIB_H
#include <utils.h>
using namespace std;

class CameraCalib
{
public:
    static string cameraFile;
    static string sourceFileBase;
    static Matrix3f priIntrinsic;
    static Matrix3f secIntrinsic;
    static MatrixXf priDistortion;
    static MatrixXf secDistortion;
    static MatrixXf extrinsic; // 1->2 extrinsic parameters
    static Matrix3f rotation;
    static Vector3f translation;
    static int meanVelocityAveSize;
};

#endif // CAMERACALIB_H
