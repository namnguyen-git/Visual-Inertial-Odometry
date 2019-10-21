#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include "utils.h"

class ImageProcessor
{
private:
    Ptr<FastFeatureDetector> detector;
    Ptr<Feature2D> extractor;
    Ptr<BFMatcher> matcher;
    //cv::Ptr<cv::FeatureDetector> detector;
    //cv::Ptr<cv::DescriptorExtractor> extractor;
    //cv::Ptr<cv::DescriptorMatcher> matcher;

public:
    ImageProcessor();

    //! KEY FUNCTIONS //////////////////////////////////////////////////////////////////////////////////////////////////
    // detect Key Points from image
    vector<KeyPoint> detectFeatures(const Mat &image, const Mat &mask, int& maxNumOfFeatures);

    // compute Description of keyPoints on image
    Mat computeDescriptors(const Mat &image, vector<KeyPoint> &keyPoints);

    // match features of two images with their corresponding key points
    vector<DMatch> matchFeatures(const Mat &descriptor1, const Mat &descriptor2);

    // reject outlier of stereo correspondence with epipolar constrains [P2,1] * F * [P1,1]' = 0 and return matches for stereo corresponding
    vector<DMatch> rejectStereoOutliers(const vector<KeyPoint>& keyPoints1, const vector<KeyPoint>& keyPoints2, const vector<DMatch>& matches, const Matrix3f& fundMatrix);

    void rejectStereoOutliers(const vector<Point2f>& feature1, const vector<Point2f>& feature2, vector<uchar>& status, const Matrix3f& fundMatrix);

    void rejectStereoOutliers(const vector<Point2f>& feature1, const vector<uchar>& status1, const vector<Point2f>& feature2, const vector<uchar>& status2, vector<uchar>& status, const Matrix3f& fundMatrix);

    Vector3f triangulateStereoPoints(const Vector3f& firstObserv, const Vector3f& secondObserv, float& quality, const Matrix3f& R, const Vector3f& T);

    void trackFeaturesKLT(const Mat& prevFrame, const Mat& currFrame, const vector<Point2f>& prevFeatures, vector<Point2f>& currFeatures, vector<uchar>& status, vector<float>& error, Size& winSizeTrack, TermCriteria& termcritTrack, float minEigenThres);

    vector<Vector3f> computeStereoFeaturePose(const vector<KeyPoint> &keyPoints1, const vector<KeyPoint> &keyPoints2, vector<DMatch> &stereoMatches, const Matrix3f& K1, const Matrix3f& K2, const MatrixXf& D1, const MatrixXf& D2, const Matrix3f& R, const Vector3f& T, vector<float>& allQuality);

    Vector3f computeCameraPose(const vector<Feature>& pointCloud, const Vector3f& preCameraPose, const Matrix3f& R, const int thresInliers, const vector<float>& trackError);

    //! HELPER FUNCTIONS ///////////////////////////////////////////////////////////////////////////////////////////////
    vector<Point2f> computeStatistic(vector<Vector3f> v);

    Point2f computeStatistic(vector<float> v);

    bool checkStereoValidPose(Vector3f& point);

    vector<Vector3f> convertKeypointsToObservation(const vector<KeyPoint>& keyPoints, vector<DMatch> stereoMatches, bool primary, const Matrix3f& K, const MatrixXf& D);

    vector<Vector3f> convert2DFeaturesToObservation(const vector<Point2f> features, const Matrix3f& K, const MatrixXf& D);

    vector<Point2f> extractMatchedFeatures(const vector<KeyPoint> keyPoints, const vector<DMatch> matches, bool primary);

    vector<Point2f> convertKeyPointToPoint2f(const vector<KeyPoint>& keyP);

    vector<KeyPoint> convertPoint2fToKeyPoint(const vector<Point2f>& p);

    template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
    void eigen2cvMat(const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& src, cv::Mat& dst);   
};

#endif // IMAGEPROCESSOR_H


