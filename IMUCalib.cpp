#include <IMUCalib.h>

string IMUCalib::IMUFile = "/home/nam/workingroom/opencv_project/BinocularVIO/stereoSLAM-TestVideos/imu.txt";

Matrix3f IMUCalib::IMUtoCameraRotation = []{
    Matrix3f tmp;
    tmp << -0.99987545, 0.01531,     0.0038332,
           -0.01531016, -0.99988279, -0.00001215,
            0.00383257, -0.00007083, 0.99999265;
    return tmp;
}();

Vector3f IMUCalib::IMUtoCameraTranslation = []{
    Vector3f tmp;
    tmp << 0.09357779, 0.00965728, -0.00158094;
    return tmp;
}();


