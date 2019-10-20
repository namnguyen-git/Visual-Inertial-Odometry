#ifndef RANSAC_H
#define RANSAC_H
#include <utils.h>

namespace Utility{
    class RANSAC;
}

class Utility::RANSAC
{
private:
    void computeErrAndInliers(const Vector3f& cameraPose, const vector<Vector3f>& all_uij, const vector<Feature>& allFeatures, vector<int>& inliers, float& error, vector<float> trackError);
    Vector3f compute3DPose(const vector<Matrix3f>& all_A, const vector<Vector3f>& all_Axp, const vector<int>& points);

public:
    RANSAC();
    void ransac2PointsCamPose(const vector<Matrix3f>& all_A, const vector<Vector3f>& all_Axp, const vector<Vector3f>& all_uij, const vector<Feature>& allFeatures, Vector3f& camPose, vector<int>& resultInliers, int thresInliers, vector<float> trackError);
};

#endif // RANSAC_H
