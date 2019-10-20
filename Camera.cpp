#include "Camera.h"

Camera::Camera()
{
}

void Camera::init(int N, ifstream& videoData, Matrix3f K1, Matrix3f K2, MatrixXf D1, MatrixXf D2, MatrixXf RT, Matrix3f R, Vector3f T)
{
    // set camera's parameters
    priIntrinsic = K1;
    secIntrinsic = K2;
    priDistortion = D1;
    secDistortion = D2;
    extrinsic = RT;
    rotation = R;
    translation = T;
    MatrixXf tmp(3,4);
    tmp << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0;
    priCamMatrix = priIntrinsic*tmp; // set the world coordinate to be at the primary camera center
    secCamMatrix = secIntrinsic*extrinsic;

    // calculate fundamental matrix F = K'^(−T) R K^T [K R^T T]×
    fundMatrix = computeFundMatrix(priIntrinsic, secIntrinsic, rotation, translation);

    // start streaming videos
    frameID = 1;
    string tmpStr1, tmpStr2;

    // skip the first N frames
    for (int i = 0; i < N; i++)
        videoData >> time >> tmpStr1;

    prevTime = time;
    tmpStr2 = tmpStr1;
    tmpStr1.insert(tmpStr1.begin() + 3,'1');
    tmpStr2.insert(tmpStr2.begin() + 3,'2');
    priFrameRGB = imread(CameraCalib::sourceFileBase + tmpStr1);
    secFrameRGB = imread(CameraCalib::sourceFileBase + tmpStr2);
    resize(secFrameRGB, secFrameRGB, priFrameRGB.size());
    if (priFrameRGB.empty() || secFrameRGB.empty()){
        cout << "frame empty!" << endl;
        return;
    }
    if (priFrameRGB.size != secFrameRGB.size){
        cout << "frame sizes do not match!" << endl;
        return;
    }

    priPrevFrameRGB = priFrameRGB;
    secPrevFrameRGB = secFrameRGB;
    cvtColor(priFrameRGB, priFrame, COLOR_RGB2GRAY); // convert to grayscale
    cvtColor(secFrameRGB, secFrame, COLOR_RGB2GRAY); // convert to grayscale
    priPrevFrame = priFrame.clone();
    secPrevFrame = secFrame.clone();

    // set initial vel
    vel3D.push_back(Vector3f(0,0,0));
    meanVel3D = Vector3f(0,0,0);
}

void Camera::updateFrame(ifstream& videoData)
{
    // calculate velocity of the previous frame, given the newly estimated camera 3D pose
    if (pose3D.size() >= 2){
        Vector3f v = (pose3D[pose3D.size() - 1] - pose3D[pose3D.size() - 2])/(time - prevTime);
        vel3D.push_back(v);
    }
    meanVel3D = Vector3f(0,0,0);
    int size = vel3D.size();
    if (size >= 3){
        for (int i = 1; i <= CameraCalib::meanVelocityAveSize && i < size - 1; i++){ // stay away from the vel3D[1], it's too high due to the fact that we set initial pose to 0
            int N = min(CameraCalib::meanVelocityAveSize, size - 2);
            meanVel3D += 1/float(N)*vel3D[size - i];
        }
    }
    // update frames    
    frameID++;
    prevTime = time;
    priPrevFrame = priFrame.clone();
    secPrevFrame = secFrame.clone();
    priPrevFrameRGB = priFrameRGB.clone();
    secPrevFrameRGB = secFrameRGB.clone();

    string tmpStr1, tmpStr2;
    videoData >> time >> tmpStr1;
    tmpStr2 = tmpStr1;
    tmpStr1.insert(tmpStr1.begin() + 3,'1');
    tmpStr2.insert(tmpStr2.begin() + 3,'2');    
    priFrameRGB = imread(CameraCalib::sourceFileBase + tmpStr1);
    secFrameRGB = imread(CameraCalib::sourceFileBase + tmpStr2);
    resize(secFrameRGB, secFrameRGB, priFrameRGB.size());
    if (priFrameRGB.empty() || secFrameRGB.empty()){
        cout << "frame empty!" << endl;
        return;
    }
    if (priFrameRGB.size != secFrameRGB.size){
        cout << "frame sizes do not match!" << endl;
        return;
    }

    cvtColor(priFrameRGB, priFrame, COLOR_RGB2GRAY); // convert to grayscale
    cvtColor(secFrameRGB, secFrame, COLOR_RGB2GRAY); // convert to grayscale
}

void Camera::updatePose(Vector3f pose)
{
    pose3D.push_back(pose);
}

Vector3f Camera::getCurrPose()
{
    return pose3D.back();
}

Vector3f Camera::getCurrVel()
{
    return vel3D.back();
}


int Camera::getFrameID()
{
    return frameID;
}

float Camera::getTime()
{
    return time;
}

Mat Camera::getCurrFrame(bool secondary)
{
    if (!secondary)
        return priFrame;
    else
        return secFrame;
}

Mat Camera::getCurrFrameRGB(bool secondary)
{
    if (!secondary)
        return priFrameRGB;
    else
        return secFrameRGB;
}

Mat Camera::getPrevFrame(bool secondary)
{
    if (!secondary)
        return priPrevFrame;
    else
        return secPrevFrame;
}

Mat Camera::getPrevFrameRGB(bool secondary)
{
    if (!secondary)
        return priPrevFrameRGB;
    else
        return secPrevFrameRGB;
}

Matrix3f Camera::getPriIntrinsic()
{
    return priIntrinsic;
}

Matrix3f Camera::getSecIntrinsic()
{
    return secIntrinsic;
}

MatrixXf Camera::getPriDistortion()
{
    return priDistortion;
}

MatrixXf Camera::getSecDistortion()
{
    return secDistortion;
}

Matrix3f Camera::getRotation()
{
    return rotation;
}

Vector3f Camera::getTranslation()
{
    return translation;
}

MatrixXf Camera::getPriCamMatrix()
{
    return priCamMatrix;
}

MatrixXf Camera::getSecCamMatrix()
{
    return secCamMatrix;
}

Matrix3f Camera::getFundMatrix()
{
    return fundMatrix;
}

Vector3f Camera::getMeanVelocity()
{
    return meanVel3D;
}


Matrix3f Camera::computeFundMatrix(Matrix3f priIntrinsic, Matrix3f secIntrinsic, Matrix3f rotation, Vector3f translation)
{
    // calculate fundamental matrix F = K'^(−T) R K^T [K R^T T]×
    Matrix3f fundMatrix;
    Vector3f v = priIntrinsic*rotation.transpose()*translation;
    Matrix3f vX;
    vX << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    Matrix3f secIntrinsicTInv = secIntrinsic.transpose().inverse();
    fundMatrix = secIntrinsicTInv*rotation*priIntrinsic.transpose()*vX;
    fundMatrix = 1/fundMatrix(2,2)*fundMatrix;
    return fundMatrix;   
}



