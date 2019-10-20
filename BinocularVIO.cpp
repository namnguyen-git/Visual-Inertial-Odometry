#include <BinocularVIO.h>
#include <GroundTruth.h>
#include <Camera.h>
#include <IMU.h>
#include <ImageProcessor.h>
#include <LocalMap.h>
#include <utils.h>

BinocularVIO::BinocularVIO()
{
}

void BinocularVIO::init(IMU& imu, ifstream& imuData, Camera& cams, ifstream& videoData, GroundTruth& grTruth, ifstream& data, int N)
{
    // initialize camera frame and parameters
    if (!videoData.is_open()){
        cout << "camera not working! " << endl;
        return;
    }
    cams.init(N, videoData, CameraCalib::priIntrinsic, CameraCalib::secIntrinsic, CameraCalib::priDistortion, CameraCalib::secDistortion, CameraCalib::extrinsic, CameraCalib::rotation, CameraCalib::translation);
    priFrame = cams.getCurrFrame(0);;
    priFrameRGB = cams.getCurrFrameRGB(0);
    secFrame = cams.getCurrFrame(1);
    secFrameRGB = cams.getCurrFrameRGB(1);

    W = priFrame.cols;
    H = priFrame.rows;
    frameID = cams.getFrameID();
    frameTime = cams.getTime();
    prevFrameTime = frameTime;

    priIntrinsic = cams.getPriIntrinsic();
    secIntrinsic = cams.getSecIntrinsic();
    priDistortion = cams.getPriDistortion();
    secDistortion = cams.getSecDistortion();
    rotation = cams.getRotation();
    translation = cams.getTranslation();
    fundMatrix = cams.getFundMatrix();

    // initialize IMU
    if (!imuData.is_open()){
        cout << "IMU not working! " << endl;
        return;
    }

    imu.init(imuData, IMUCalib::IMUtoCameraRotation, IMUCalib::IMUtoCameraTranslation);
    imuTime = imu.getTime();
    while (imuTime < frameTime){
        imu.update(imuData);
        imuTime = imu.getTime();
    }
    prevIMUTime = imuTime;
    imuDataID = imu.getID();
    IMUtoCameraRotation = imu.getIMUtoCameraRotation();
    IMUtoCameraTranslation = imu.getIMUtoCameraTranslation();
    currAgularVel = IMUtoCameraRotation*imu.getAngularVel(); // transform to camera's body frame
    currAccel = IMUtoCameraRotation*imu.getAccel(); // transform to camera's body frame

    // initialize groundtruth
    if (!data.is_open()){
        cout << "ground truth data not working! " << endl;
        return;
    }

    grTruth.init(data);
    time = grTruth.getTime();
    while (time < frameTime){
        grTruth.update(data);
        time = grTruth.getTime();
    }
    prevTime = grTruth.getPrevTime();
    dataID = grTruth.getID();
    truePose = grTruth.getPose();
    prevTruePose = truePose;
    initCamPose = truePose;
    float t = (frameTime - prevTime)/(time - prevTime);
    if (t <= 0) cout << "bad timing for interpolation, t < 0 " << endl;
    Quaternionf currentquaternion = grTruth.getQuaternion();
    Quaternionf prevQuaternion = grTruth.getPrevQuaternion();
    Quaternionf frameQuaternion = prevQuaternion.slerp(t, currentquaternion);
    frameQuaternion.normalize();    

    cameraToWorldRotation = frameQuaternion.toRotationMatrix();
    prevCameraToWorldRotation = cameraToWorldRotation;
    keyFrameToWorldRotation = cameraToWorldRotation;
    numOfKeyFrame = 1;
    eulerAngles = cameraToWorldRotation.eulerAngles(0, 1, 2);
    prevEulerAngles = eulerAngles;

    // initialize camera pose
    cams.updatePose(truePose);
    currCamPose = cams.getCurrPose();
    currCamVel = cams.getCurrVel();
    keyFrameCamPose = currCamPose;

    // initialize feature tracking
    numOfTrackedFeatures = 0;
}

void BinocularVIO::updateGrTruth(GroundTruth &grTruth, ifstream& data)
{
    if (!data.is_open()){
        cout << "imu not working! " << endl;
        return;
    }

    grTruth.update(data);
    time = grTruth.getTime();
    while (time < frameTime){
        grTruth.update(data);
        time = grTruth.getTime();
    }
    prevTime = grTruth.getPrevTime();
    dataID = grTruth.getID();
    prevTruePose = truePose;
    truePose = grTruth.getPose();
    float t = (frameTime - prevTime)/(time - prevTime);
    if (t <= 0) cout << "bad timing for interpolation, t < 0 " << endl;
    Quaternionf currIMUquaternion = grTruth.getQuaternion();
    Quaternionf prevIMUQuaternion = grTruth.getPrevQuaternion();
    Quaternionf frameQuaternion = prevIMUQuaternion.slerp(t, currIMUquaternion);
    frameQuaternion.normalize();
    prevCameraToWorldRotation = cameraToWorldRotation;
    cameraToWorldRotation = frameQuaternion.toRotationMatrix();
    prevEulerAngles = eulerAngles;
    eulerAngles = cameraToWorldRotation.eulerAngles(0, 1, 2);
}

void BinocularVIO::updateIMU(IMU& imu, ifstream& imuData)
{
    if (!imuData.is_open()){
        cout << "IMU not working! " << endl;
        return;
    }

    imu.update(imuData);
    prevIMUTime = imuTime;
    imuTime = imu.getTime();
    imuDataID = imu.getID();
    currAgularVel = imu.getAngularVel();
    currAccel = imu.getAccel();
}

void BinocularVIO::updateCameraFrame(Camera& cams, ifstream& videoData)
{
    cams.updateFrame(videoData);
    priFrame = cams.getCurrFrame(0);
    priPrevFrame = cams.getPrevFrame(0);
    priFrameRGB = cams.getCurrFrameRGB(0);
    priPrevFrameRGB = cams.getPrevFrameRGB(0);

    secFrame = cams.getCurrFrame(1);
    secPrevFrame = cams.getPrevFrame(1);
    secFrameRGB = cams.getCurrFrameRGB(1);
    secPrevFrameRGB = cams.getPrevFrameRGB(1);

    frameID = cams.getFrameID();
    prevFrameTime = frameTime;
    frameTime = cams.getTime();    
    prevCamPose = cams.getCurrPose();
}

int BinocularVIO::stereoMatchAndFeature3DPose(ImageProcessor& imgProcessor)
{
    // update keyframe to be the current primary frame
    keyPriFrame = priFrame.clone();
    keyPriFrameRGB = priFrameRGB.clone();
    keyFrameToWorldRotation = cameraToWorldRotation;
    keyFrameCamPose = currCamPose;
    numOfKeyFrame++;   

    int numOfFeaturesToAdd = Config::maxFeatureTrack - numOfTrackedFeatures;
    Mat mask(H, W, CV_8U, Scalar(255));
    secFeatures = imgProcessor.detectFeatures(secFrame, mask, numOfFeaturesToAdd);
    Mat secDescriptor = imgProcessor.computeDescriptors(secFrame, secFeatures);

    for (int i = 0; i < tobetrackedFeatures.size(); i++){
        if (tobetrackedFeatures[i].x - 2 >= 0 && tobetrackedFeatures[i].x + 2 < mask.cols && tobetrackedFeatures[i].y - 2 >=0 && tobetrackedFeatures[i].y + 2 < mask.rows){
            Mat pRoi = mask(Rect(tobetrackedFeatures[i].x - 2, tobetrackedFeatures[i].y - 2, 4, 4));
            pRoi.setTo(0);
        }
    }
    priFeatures = imgProcessor.detectFeatures(keyPriFrame, mask, numOfFeaturesToAdd);
    Mat priDescriptor = imgProcessor.computeDescriptors(keyPriFrame, priFeatures);
    matches = imgProcessor.matchFeatures(priDescriptor, secDescriptor);  

    // apply epipolar constraint to get stereo matched features
    if (matches.size() > 0){
        stereoMatchesBeforeComputePose = imgProcessor.rejectStereoOutliers(priFeatures, secFeatures, matches, fundMatrix);
    }

    if (stereoMatchesBeforeComputePose.size() > 0){
        stereoMatchesAfterComputePose = stereoMatchesBeforeComputePose;
        stereo3DPose = imgProcessor.computeStereoFeaturePose(priFeatures, secFeatures, stereoMatchesAfterComputePose, priIntrinsic, secIntrinsic, priDistortion, secDistortion, rotation, translation, stereoMatchQuality);
    }    

    if (stereoMatchesAfterComputePose.size() > 0){
        stereoMatchedPriFeatures = imgProcessor.extractMatchedFeatures(priFeatures, stereoMatchesAfterComputePose, 1);
        tobetrackedFeatures = stereoMatchedPriFeatures;
    }

    return stereo3DPose.size();
}

int BinocularVIO::addFeaturesToLocalMap(ImageProcessor& imgProcessor, LocalMap& localMap)
{
    // add features to local map
    LocalMap m;
    vector<Vector3f> obs = imgProcessor.convert2DFeaturesToObservation(stereoMatchedPriFeatures, priIntrinsic, priDistortion);
    int size = stereoMatchedPriFeatures.size();
    for (int i = 0; i < size; i++){
        Feature tmp;
        tmp.pose3D = keyFrameToWorldRotation*stereo3DPose[i] + keyFrameCamPose;
        tmp.pose2D.push_back(stereoMatchedPriFeatures[i]);
        tmp.observation.push_back(obs[i]);
        localMap.add(tmp);
    }
    vector<Feature> pointCloud = localMap.getPointCloud();

    // prepare features to be tracked in the next frame
    tobetrackedFeatures.clear();
    for (int i = 0; i < pointCloud.size(); i++){
        tobetrackedFeatures.push_back(localMap.getPointCloud()[i].pose2D.back());
    }    

    return tobetrackedFeatures.size();
}

int BinocularVIO::track2DFeatures(ImageProcessor& imgProcessor)
{
    // track features with KLT
    trackStatus.clear();
    trackError.clear();
    trackedFeatures.clear();
    imgProcessor.trackFeaturesKLT(priPrevFrame, priFrame, tobetrackedFeatures, trackedFeatures, trackStatus, trackError, Config::winSizeTrack, Config::termcritTrack, Config::minEigenThres);

    // filter out bad tracked features to prepare for camera pose estimation
    float errThres = Config::maxTrackingError;
    numOfTrackedFeatures = 0;
    meanTrackingError = 0;
    for (int i = 0; i < trackStatus.size(); i++){
        if (trackStatus[i] == 1){
            bool b = trackError[i] < errThres;
            numOfTrackedFeatures += b;
            meanTrackingError += trackError[i];
            trackStatus[i] *= b;
        }
    }
    meanTrackingError /= numOfTrackedFeatures;

    // get rid of bad tracking in the tracking error - prepare for ransac for camera pose estimation
    vector<float> tmp;
    for (int i = 0; i < trackStatus.size(); i++){
        if (trackStatus[i]){
            tmp.push_back(trackError[i]);
        }
    }
    trackError = tmp;

    return numOfTrackedFeatures;
}

void BinocularVIO::updateLocalMap(ImageProcessor& imgProcessor, LocalMap& localMap)
{   
    localMap.updatePose2D(trackedFeatures, trackStatus);
    vector<Vector3f> obs = imgProcessor.convert2DFeaturesToObservation(trackedFeatures, priIntrinsic, priDistortion);
    localMap.updateObservation(obs, trackStatus);    

    // prepare features to track
    localMap.deleteLostFeatures(trackStatus);
    tobetrackedFeatures.clear();
    vector<Feature> pointCloud = localMap.getPointCloud();
    for (int i = 0; i < pointCloud.size(); i++){
        tobetrackedFeatures.push_back(localMap.getPointCloud()[i].pose2D.back());
    }
}

Vector3f BinocularVIO::computeAndUpdateCameraPose(ImageProcessor& imgProcessor, LocalMap& localMap, Camera& cams)
{
    // estimate camera pose
    vector<Feature> pointCloud = localMap.getPointCloud();
    Vector3f predictedCamPose = prevCamPose + cams.getMeanVelocity()*(frameTime - prevFrameTime);
    int thresInliers = round(min((float)Config::maxNumOfInliersRANSAC, numOfTrackedFeatures*Config::percentage));
    thresInliers = max(thresInliers, Config::minNumOfInliersRANSAC);
    currCamPose = imgProcessor.computeCameraPose(pointCloud, predictedCamPose, cameraToWorldRotation, thresInliers, trackError);

    // remove bad features once again
    MatrixXf tmp(3,4);
    Matrix3f R = cameraToWorldRotation.inverse();
    tmp << R, -R*currCamPose;
    priCamMatrix = priIntrinsic*tmp;
    float error;
    vector<uchar> status(pointCloud.size(),1);
    for (int i = 0; i < pointCloud.size(); i++){
        Point2f last2Dpose = pointCloud[i].pose2D.back();
        Vector3f p = pointCloud[i].pose3D;
        Vector3f q = priCamMatrix*Vector4f(p(0), p(1), p(2), 1);
        if (q(2) != 0){
            q /= q(2);
            error = sqrt(pow(q(0) - last2Dpose.x, 2) + pow(q(1) - last2Dpose.y, 2));
            if (error > Config::reprojectionErrorThres)
                status[i] = 0;
        }
    }

    // re-update local map
    localMap.deleteLostFeatures(status);
    tobetrackedFeatures.clear();
    pointCloud = localMap.getPointCloud();
    for (int i = 0; i < pointCloud.size(); i++){
        tobetrackedFeatures.push_back(localMap.getPointCloud()[i].pose2D.back());
    }   

    // update camera pose
    cams.updatePose(currCamPose);

    // return camera pose error
    return currCamPose;
}

int BinocularVIO::getFrameID()
{
    return frameID;
}

float BinocularVIO::getFrameTime()
{
    return frameTime;
}

float BinocularVIO::getIMUTime()
{
    return imuTime;
}

Vector3f BinocularVIO::getCurrCamPose()
{
    return currCamPose;
}

Vector3f BinocularVIO::getTruePose()
{
    return truePose;
}

int BinocularVIO::getNoOfTrackedFeatures()
{
    return numOfTrackedFeatures;
}

void BinocularVIO::clearData()
{
    priFeatures.clear();
    secFeatures.clear();
    priPrevFeatures.clear();
    secPrevFeatures.clear();
    matches.clear();
    stereoMatchesBeforeComputePose.clear();
    stereoMatchesAfterComputePose.clear();
    stereoMatchQuality.clear();
    stereo3DPose.clear();
    stereoMatchedPriFeatures.clear();
}

template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
void BinocularVIO::eigen2cvMat(const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& src, cv::Mat& dst)
{
    if (!(src.Flags & RowMajorBit)){
        Mat _src(src.cols(), src.rows(), cv::DataType<_Tp>::type,
            (void*)src.data(), src.stride() * sizeof(_Tp));
        transpose(_src, dst);
    }
    else{
        Mat _src(src.rows(), src.cols(), DataType<_Tp>::type,
            (void*)src.data(), src.stride() * sizeof(_Tp));
        _src.copyTo(dst);
    }
}

void BinocularVIO::debugFeaturesMatching(bool epiConstraint)
{
    vector<DMatch> matchedFeatures;
    if (epiConstraint)
        matchedFeatures = stereoMatchesAfterComputePose;
    else
        matchedFeatures = matches;

    if (matchedFeatures.size() == 0)
        cout << "no features matched!" << endl;
    else{
        cout << "number of feature matches " << matchedFeatures.size() << endl;
        Mat combinedFrame;
        hconcat(priFrameRGB, secFrameRGB, combinedFrame);
        for (int i = 0; i < matchedFeatures.size(); i++){
            Point p = priFeatures[matchedFeatures[i].queryIdx].pt;
            Point q = Point(secFeatures[matchedFeatures[i].trainIdx].pt.x + W, secFeatures[matchedFeatures[i].trainIdx].pt.y);
            line(combinedFrame, p, q, Scalar(0,0,255));
        }
        imshow("feature matching Debug", combinedFrame);
        waitKey(200);
    }
}

void BinocularVIO::debugFeatureTracking()
{
    Mat combinedFrame;
    hconcat(priPrevFrameRGB, priFrameRGB, combinedFrame);
    for (int i = 0; i < trackStatus.size(); i++){
        if (trackStatus[i]){
            Point q = Point((int)trackedFeatures[i].x + W, (int)trackedFeatures[i].y);
            line(combinedFrame, tobetrackedFeatures[i], q, Scalar(0,0,255));
        }
    }
    string s = "feature tracking debug ";
    imshow(s, combinedFrame);
    waitKey(200);
}

void BinocularVIO::debugPrintLocalMap(LocalMap localMap)
{
    vector<Feature> pointCloud = localMap.getPointCloud();
    cout << "MAP SIZE " << pointCloud.size() << endl << endl;
    for (int i = 0; i < pointCloud.size(); i++){
        cout << "Feature " << i << endl;
        cout << "3D pose " << pointCloud[i].pose3D(0) << " " << pointCloud[i].pose3D(1) << " " << pointCloud[i].pose3D(2) << endl;
        cout << "Last feature pose 2D " << pointCloud[i].pose2D.back() << endl;;
        cout << "Last observation " << pointCloud[i].observation.back()(0) << " " << pointCloud[i].observation.back()(1) << " " << pointCloud[i].observation.back()(2) << endl << endl;
    }
}

template<typename _Tp>
void BinocularVIO::debugPrintVector(vector<_Tp> v)
{
    for (int i = 0; i < v.size(); i++){
        cout << v[i] << endl << endl;;
    }
}

void BinocularVIO::createLog(ofstream& filename)
{
    // number of features matches - stereo matches before feature 3D pose - stereo matches after feature 3D pose
    filename << matches.size() << " " << stereoMatchesBeforeComputePose.size() << " " << stereoMatchesAfterComputePose.size() << endl;
}






