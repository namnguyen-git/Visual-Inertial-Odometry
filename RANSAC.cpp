//! RANSAC: implementation adopted from Phuc H. Truong
//! following Shaojie Shen et. al. "Vision-Based State Estimation for Autonomous Rotorcraft MAVs in Complex Environments"

#include "RANSAC.h"
#include <utils.h>

Utility::RANSAC::RANSAC()
{
}

Vector3f Utility::RANSAC::compute3DPose(const vector<Matrix3f>& all_A, const vector<Vector3f>& all_Axp, const vector<int>& points)
{
    // sum of A for all points
    Matrix3f A = Matrix3f::Zero();
    Vector3f Axp = Vector3f(0, 0, 0);
    for (int i = 0; i < points.size(); i++){
        A += all_A[points[i]];
        Axp += all_Axp[points[i]];
    }

    // camera pose return
    Matrix3f AInv = A.inverse();
    Vector3f cameraPose = AInv*Axp;
    return cameraPose;
}

void Utility::RANSAC::computeErrAndInliers(const Vector3f& cameraPose, const vector<Vector3f>& all_uij, const vector<Feature>& allFeatures, vector<int>& inliers, float& error, vector<float> trackError)
{
    error = 0;
    inliers.clear();
    int size = all_uij.size();
    for (int i = 0; i < size; i++){
        // cross product of vector uij and feature 3D vector
        float tmpErr;
        Vector3f tmp = allFeatures[i].pose3D - cameraPose;
        tmp = tmp/tmp.norm();
        tmpErr = sqrt((tmp.cross(all_uij[i])).squaredNorm());
        tmpErr *= trackError[i]; // Nam - testing - weight features by tracking error - turn off when done
        if (tmpErr < Config::threshErrRANSAC*Config::maxTrackingError){
            inliers.push_back(i);
            error += tmpErr;
        }
    }
    if (inliers.size() > 0)
        error /= inliers.size();
    else
        error = 1E10;
}

void Utility::RANSAC::ransac2PointsCamPose(const vector<Matrix3f>& all_A, const vector<Vector3f>& all_Axp, const vector<Vector3f>& all_uij, const vector<Feature>& allFeatures, Vector3f& camPose, vector<int>& resultInliers, int thresInliers, vector<float> trackError)
{
    random_device rd;
    default_random_engine generator(rd());
    uniform_real_distribution<float> distribution(0.0,1.0);
    float t;
    int idx;

    int nSamples = all_A.size();

    // for all RANSAC iterations, do:
    vector<int> sampleRANSACPoints; // sample points for RANSAC
    float error = 1E10; // best error
    vector<int> inliers; // best inliers
    inliers.clear();
    for (int i = 0; i < Config::loopRANSAC; i++){
        float currErr = 0;
        vector<int> currInliers;
        Vector3f currCamPose;
        sampleRANSACPoints.clear();
        // sample points randomly
        for (int j = 0; j < Config::numPointsRANSAC; j++){
            t = distribution(generator);
            idx = round(t*nSamples);
            sampleRANSACPoints.push_back(idx);
        }

        // calculate camera pose form sample points
        currCamPose = compute3DPose(all_A, all_Axp, sampleRANSACPoints);

        // from camera pose compute error and inliers
        computeErrAndInliers(currCamPose, all_uij, allFeatures, currInliers, currErr, trackError);

        // no need to continue, create next samples
        if (currErr > Config::threshSumErrRANSAC*Config::maxTrackingError || currInliers.size() < thresInliers)
            continue;

        // use all inliers to compute camera pose
        currCamPose = compute3DPose(all_A, all_Axp, currInliers);

        // compute err and inliers
        computeErrAndInliers(currCamPose, all_uij, allFeatures, currInliers, currErr, trackError);

        // check again, if can't reject outlier -> continue
        if (currErr > Config::threshSumErrRANSAC*Config::maxTrackingError || currInliers.size() < thresInliers)
            continue;

        // compare with best result
        if (currErr < error && currInliers.size() > inliers.size()){
            error = currErr;
            inliers = currInliers;
            camPose = currCamPose;
        }
    }
    resultInliers = inliers;    
}







