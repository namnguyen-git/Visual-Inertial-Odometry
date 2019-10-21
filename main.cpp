#include "mainwindow.h"
#include <QApplication>
#include <GroundTruth.h>
#include <IMU.h>
#include <Camera.h>
#include <ImageProcessor.h>
#include <LocalMap.h>
#include <BinocularVIO.h>
#include <Config.h>
#include <utils.h>

using namespace std;

int main()
{    
    int initFrameID = 1;
    int FrameID = 1;
    int maxFrameID = 300;
    int numOfTrackedFeatures = 0;    
    int minNumOfStereoFeatures = Config::minNumOfStereoFeatures;
    int minNumOfTrackedFeatures = Config::minNumOfTrackedFeatures;
    float percentage = Config::percentOfTrackedFeatures;
    bool newKeyFrameAsCurrFrame = 0;
    bool newKeyFrameAsNextFrame = 0;
    float camPoseMeanError = 0;
    float traveledDistance = 0;
    Vector3f estimatedcamPose;
    Vector3f truecamPose;
    Vector3f prevTrueCamPose;

    // create object that hanldes and implements binocularVIO's pipeline
    BinocularVIO* VIO = new BinocularVIO();

    // call "device driver" to get sensor signals. Here we have 02 cameras and an IMU as sensors.
    // The groundtruth gives camera orientation (roll/pitch/yaw). Change these files to use your own dataset
    ifstream data(Config::groundTruthFile);
    ifstream imuData(IMUCalib::IMUFile);
    ifstream videoData(CameraCalib::cameraFile);

    // create imu interface, camera interface, image processor and local map
    GroundTruth grTruth;
    IMU imu;
    Camera cams;
    ImageProcessor imgProcessor;
    LocalMap localMap;

    // initialize camera and IMU. Stream data from groundthuth for camera's orientation.
    VIO->init(imu, imuData, cams, videoData, grTruth, data, initFrameID);
    truecamPose = VIO->getTruePose();
    prevTrueCamPose = truecamPose;

    // detect and match stereo features and compute feature 3D pose (in global frame)
    int numOfStereoFeatures = VIO->stereoMatchAndFeature3DPose(imgProcessor);

    // add features to local map
    int numOfFeaturesKeyFrame = VIO->addFeaturesToLocalMap(imgProcessor, localMap);

    //! main while loop //////////////////////////////////////////////////////////////////////////////////////////////
    while (FrameID < maxFrameID){
        // in case we don't have enough features for RANSAC
        if (newKeyFrameAsCurrFrame){
            numOfStereoFeatures = VIO->stereoMatchAndFeature3DPose(imgProcessor);
            newKeyFrameAsNextFrame = (numOfStereoFeatures < minNumOfStereoFeatures);
            while (newKeyFrameAsNextFrame){
                // get the next camera frame and orirentation data
                VIO->updateCameraFrame(cams, videoData);
                VIO->updateGrTruth(grTruth, data);
                FrameID = VIO->getFrameID();
                numOfStereoFeatures = VIO->stereoMatchAndFeature3DPose(imgProcessor);
                newKeyFrameAsNextFrame = (numOfStereoFeatures < minNumOfStereoFeatures);
            }
            numOfFeaturesKeyFrame = VIO->addFeaturesToLocalMap(imgProcessor, localMap);
        }

        // update next frame and data prepare for feature tracking
        VIO->updateCameraFrame(cams, videoData);
        VIO->updateGrTruth(grTruth, data);
        VIO->updateIMU(imu, imuData);
        FrameID = VIO->getFrameID();

        // track features
        numOfTrackedFeatures = VIO->track2DFeatures(imgProcessor);
        newKeyFrameAsCurrFrame = (numOfTrackedFeatures < max(percentage*numOfFeaturesKeyFrame, (float)minNumOfTrackedFeatures));

        // update tracked features to local map
        VIO->updateLocalMap(imgProcessor, localMap);

        // compute camera pose and camera pose error from ground truth
        estimatedcamPose = VIO->computeAndUpdateCameraPose(imgProcessor, localMap, cams);

        // calculate error and total distance traveled so far
        prevTrueCamPose = truecamPose;
        truecamPose = VIO->getTruePose();
        Vector3f camPoseError = estimatedcamPose - truecamPose;
        Vector3f traveledCamPose = truecamPose - prevTrueCamPose;
        camPoseMeanError += sqrt(pow(camPoseError(0),2) + pow(camPoseError(1),2) + pow(camPoseError(2),2));
        traveledDistance += sqrt(pow(traveledCamPose(0),2) + pow(traveledCamPose(1),2) + pow(traveledCamPose(2),2));

        // clear data to prepare for next frame
        VIO->clearData();
    }

    // clean up
    videoData.close();
    data.close();
    delete VIO;

    // print out result
    cout << "total distance traveled (m) " << traveledDistance << endl;
    cout << "mean pose error (m) " << camPoseMeanError/(maxFrameID) << endl;

    return 0;
}
