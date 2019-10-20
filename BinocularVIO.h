#ifndef BINOCULARVIO_H
#define BINOCULARVIO_H

#include <utils.h>

class IMU;
class GroundTruth;
class LocalMap;
class Camera;
class ImageProcessor;
class UKF;
class BinocularVIO
{
private:
    Matrix3f M;

    //! Groundtruth
    int dataID;
    float time;
    float prevTime;
    Matrix3f keyFrameToWorldRotation;
    Matrix3f cameraToWorldRotation;
    Matrix3f prevCameraToWorldRotation;
    Vector3f eulerAngles;
    Vector3f prevEulerAngles;
    Vector3f truePose;
    Vector3f prevTruePose;
    float distFromLastKeyFrame;

    //! IMU
    int imuDataID;
    float imuTime;
    float prevIMUTime;
    Vector3f currAgularVel;
    Vector3f currAccel;
    Matrix3f IMUtoCameraRotation;
    Vector3f IMUtoCameraTranslation;

    //! camera
    Vector3f currCamPose;
    Vector3f currCamVel;
    Vector3f keyFrameCamPose;
    int frameID;
    int numOfKeyFrame;
    float frameTime;
    float prevFrameTime;
    int W;
    int H;
    Vector3f initCamPose;
    Vector3f initCamPoseError;
    Vector3f prevCamPose;

    MatrixXf priCamMatrix;
    Matrix3f priIntrinsic;
    Matrix3f secIntrinsic;
    MatrixXf priDistortion;
    MatrixXf secDistortion;
    Matrix3f rotation;
    Vector3f translation;
    Matrix3f fundMatrix;

    Mat keyPriFrame;
    Mat keyPriFrameRGB;
    Mat priFrame;
    Mat priFrameRGB;
    Mat priPrevFrame;
    Mat priPrevFrameRGB;
    Mat secFrame;
    Mat secFrameRGB;
    Mat secPrevFrame;
    Mat secPrevFrameRGB;

    //! image processing
    // feature detect and match
    vector<KeyPoint> priFeatures; // 2D features in the primary frame. Only for debugging
    vector<KeyPoint> secFeatures; // 2D featuers in the secondary frame. Only for debugging
    vector<KeyPoint> priPrevFeatures; // 2D features in the previous primary frame. Only for debugging
    vector<KeyPoint> secPrevFeatures; // 2D featuers in the previous secondary frame. Only for debugging
    vector<DMatch> matches; // matched features between 02 frames. Only for debugging

    vector<DMatch> stereoMatchesBeforeComputePose; // matched features between 02 frames that satisfy epipolar constraint before compute features' 3D pose
    vector<DMatch> stereoMatchesAfterComputePose; // matched features between 02 frames that satisfy epipolar constraint after compute features' 3D pose
    vector<float> stereoMatchQuality;
    vector<Point2f> stereoMatchedPriFeatures; // contains stereo matched features of the primary camera
    vector<Point2f> stereoMatchedSecFeatures;
    vector<Vector3f> stereo3DPose;

    //! feature tracking
    vector<Point2f> tobetrackedFeatures;
    vector<Point2f> tobetrackedSecFeatures;
    vector<Point2f> trackedFeatures;
    vector<Point2f> trackedSecFeatures;
    vector<uchar> trackStatus;
    vector<float> trackError;
    int numOfTrackedFeatures;
    float meanTrackingError;    

public:
    BinocularVIO();

    // VIO pipeline functions
    void init(IMU& imu, ifstream& imuData, Camera& cams, ifstream& videoData, GroundTruth& grTruth, ifstream& data, int N);

    void updateIMU(IMU& imu, ifstream& imuData);

    void updateGrTruth(GroundTruth& grTruth, ifstream& Data);

    void initializeCamera(Camera& cams, ifstream& videoData);

    void updateCameraFrame(Camera& cams, ifstream& videoData);

    int stereoMatchAndFeature3DPose(ImageProcessor& imgProcessor);

    int addFeaturesToLocalMap(ImageProcessor& imgProcessor, LocalMap& localMap);

    void computeTrackWinSize(LocalMap& localMap, Camera& cams);

    int track2DFeatures(ImageProcessor& imgProcessor);    

    void updateLocalMap(ImageProcessor& imgProcessor, LocalMap& localMap);

    Vector3f computeAndUpdateCameraPose(ImageProcessor& imgProcessor, LocalMap& localMap, Camera& cams);

    int getFrameID();

    float getFrameTime();

    float getIMUTime();

    Vector3f getCurrCamPose();

    Vector3f getTruePose();

    int getNoOfTrackedFeatures();

    void clearData();

    // utility functions
    template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
    void eigen2cvMat(const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& src, Mat& dst);

    //debug functions
    void debugFeaturesMatching(bool epiConstraint);

    void debugFeatureTracking();

    void debugPrintLocalMap(LocalMap localMap);

    template<typename _Tp>
    void debugPrintVector(vector<_Tp> v);

    void createLog(ofstream& filename);
};

#endif // BINOCULARVIO_H
