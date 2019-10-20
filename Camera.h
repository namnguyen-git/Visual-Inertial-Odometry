#ifndef CAMERA_H
#define CAMERA_H

#include <utils.h>

using namespace std;

class Camera
{
private:    
    Mat priFrame; // primary frame
    Mat secFrame; // secondary frame
    Mat priFrameRGB; // color primary frame for debugging
    Mat secFrameRGB; // color secondary frame for debugging
    Mat priPrevFrame; // previous primary frame
    Mat priPrevFrameRGB;
    Mat secPrevFrame; // previous secondary frame
    Mat secPrevFrameRGB;

    int frameID; // frame ID
    float time;
    float prevTime;
    int H; // image height
    int W; // image width

    Matrix3f priIntrinsic;
    Matrix3f secIntrinsic;
    MatrixXf priDistortion;
    MatrixXf secDistortion;
    MatrixXf extrinsic;
    Matrix3f rotation; // 1->2 rotation
    Vector3f translation; // 1->2 translation
    MatrixXf priCamMatrix; // camera matrix of the primary camera
    MatrixXf secCamMatrix; // camera matrix of the secondary camera
    Matrix3f fundMatrix; // fundamental matrix between 02 cameras

    vector<Vector3f> pose3D; // camera's 3D pose. Nam - to do - can't keep adding pose to the vector. This may cause overflow
    vector<Vector3f> vel3D; // camera's 3D velocity
    Vector3f meanVel3D; // camera's mean velocity

public:
    Camera();
    void init(int N, ifstream& videoData, Matrix3f K1, Matrix3f K2, MatrixXf D1, MatrixXf D2, MatrixXf RT, Matrix3f R, Vector3f T);
    void updateFrame(ifstream& videoData);
    void updatePose(Vector3f pose);
    Vector3f getCurrPose();
    Vector3f getCurrVel();
    int getFrameID();
    float getTime();
    Mat getCurrFrame(bool secondary);
    Mat getCurrFrameRGB(bool secondary);
    Mat getPrevFrame(bool secondary);
    Mat getPrevFrameRGB(bool secondary);
    Matrix3f getPriIntrinsic();
    Matrix3f getSecIntrinsic();
    MatrixXf getPriDistortion();
    MatrixXf getSecDistortion();
    Matrix3f getRotation(); // 1->2 rotation
    Vector3f getTranslation(); // 1->2 translation
    MatrixXf getPriCamMatrix();
    MatrixXf getSecCamMatrix();
    Matrix3f getFundMatrix();
    Vector3f getMeanVelocity();

    // helper function
    Matrix3f computeFundMatrix(Matrix3f priIntrinsic, Matrix3f secIntrinsic, Matrix3f rotation, Vector3f translation);
};

#endif // FRAME_H
