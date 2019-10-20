#include "CameraCalib.h"

// Camera config
string CameraCalib::cameraFile = "/home/nam/workingroom/opencv_project/BinocularVIO/stereoSLAM-TestVideos/image-read.txt";
string CameraCalib::sourceFileBase = "/home/nam/workingroom/opencv_project/BinocularVIO/stereoSLAM-TestVideos/";

int CameraCalib::meanVelocityAveSize = 10;

Matrix3f CameraCalib::priIntrinsic = []{
    Matrix3f tmp;
    tmp << 726.28741455078, 0.0, 354.6496887207, 0.0, 726.28741455078, 186.46566772461, 0.0, 0.0, 1.0;
    return tmp;
}();

Matrix3f CameraCalib::secIntrinsic = []{
    Matrix3f tmp;
    tmp << 726.04388427734, 0.0, 385.73248291016, 0.0, 726.04388427734, 262.19641113281, 0.0, 0.0, 1.0;
    return tmp;
}();

MatrixXf CameraCalib::priDistortion = []{
    MatrixXf tmp(5,1);
    tmp << 0, 0, 0, 0, 0;
    return tmp;
}();

MatrixXf CameraCalib::secDistortion = []{
    MatrixXf tmp(5,1);
    tmp << 0, 0, 0, 0, 0;
    return tmp;
}();

MatrixXf CameraCalib::extrinsic = []{
    MatrixXf tmp(3,4);
    tmp << 0.99992078542709, 0.012314549647272, -0.0026051788590848, -0.089730150997639,
           -0.012324691750109, 0.99991637468338, -0.003913477063179, -8.6142681539059e-05,
           0.002556768245995, 0.0039452752098441, 0.99998897314072, 0.00037138815969229;
    return tmp;
}();

Matrix3f CameraCalib::rotation = []{
    Matrix3f tmp;
    tmp << 0.99992078542709, 0.012314549647272, -0.0026051788590848,
           -0.012324691750109, 0.99991637468338, -0.003913477063179,
           0.002556768245995, 0.0039452752098441, 0.99998897314072;
    return tmp;
}();

Vector3f CameraCalib::translation = []{
    Vector3f tmp;
    tmp << -0.089730150997639, -8.6142681539059e-05, 0.00037138815969229;
    return tmp;
}();
