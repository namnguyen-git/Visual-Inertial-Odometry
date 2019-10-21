#include <ImageProcessor.h>
#include <utils.h>
#include <RANSAC.h>
#include <LocalMap.h>

bool responseComparator(const KeyPoint& p1, const KeyPoint& p2)
{
    return p1.response > p2.response;
}

vector<Point2f> keyPointsToPoints(vector<KeyPoint>& keyP)
{
    vector<Point2f> p;
    int size = keyP.size();
    for(int i=0; i<size; i++)
        p.push_back(keyP[i].pt);
    return p;
}

ImageProcessor::ImageProcessor()
{
    detector = FastFeatureDetector::create(Config::detectorThreshold,true);
    extractor = xfeatures2d::BriefDescriptorExtractor::create();
    matcher = cv::BFMatcher::create(NORM_L2);    
}


vector<KeyPoint> ImageProcessor::detectFeatures(const Mat& image, const Mat& mask, int& maxNumOfFeatures)
{
    vector<KeyPoint> keyP;
    detector->detect(image, keyP, mask);

    KeyPointsFilter filter;
    filter.retainBest(keyP, maxNumOfFeatures);
    sort(keyP.begin(), keyP.end(), responseComparator);

    vector<Point2f> p  = convertKeyPointToPoint2f(keyP);
    cornerSubPix(image, p, Size(2, 2), Size(-1, -1), Config::termcritTrack); // find subpixel features
    keyP = convertPoint2fToKeyPoint(p);

    return keyP;
}

Mat ImageProcessor::computeDescriptors(const Mat &image, vector<KeyPoint> &keyPoints)
{
    Mat descriptors;
    extractor->compute(image, keyPoints, descriptors);
    return descriptors;
}

vector<DMatch> ImageProcessor::matchFeatures(const Mat &descriptor1, const Mat &descriptor2)
{
    vector<DMatch> matches;
    matcher->match(descriptor1, descriptor2, matches);
    return matches;
}

vector<DMatch> ImageProcessor::rejectStereoOutliers(const vector<KeyPoint>& keyPoints1, const vector<KeyPoint>& keyPoints2, const vector<DMatch>& matches, const Matrix3f& fundMatrix)
{
    vector<DMatch> bestMatches;    
    int N = matches.size();
    if(N == 0)
        return bestMatches;    

    // check epipolar constrains: v2^T*F*v1 = 0
    Vector3f v1, v2;
    float result;
    for(int i = 0; i < N ; i++){
        v1 = Vector3f(keyPoints1[matches[i].queryIdx].pt.x, keyPoints1[matches[i].queryIdx].pt.y, 1);
        v2 = Vector3f(keyPoints2[matches[i].trainIdx].pt.x, keyPoints2[matches[i].trainIdx].pt.y, 1);
        result = v2.dot(fundMatrix*v1);
        if(abs(result) < Config::rejectStereoThresh)
            bestMatches.push_back(matches[i]);
    }
    return bestMatches;
}

void ImageProcessor::rejectStereoOutliers(const vector<Point2f>& feature1, const vector<Point2f>& feature2, vector<uchar>& status, const Matrix3f& fundMatrix)
{
    Vector3f v1, v2;
    float result;
    int N = feature1.size();
    for(int i = 0; i < N && status[i] != 0; i++){
        v1 = Vector3f(feature1[i].x, feature1[i].y, 1);
        v2 = Vector3f(feature2[i].x, feature2[i].y, 1);
        result = v2.dot(fundMatrix*v1);
        if(abs(result) > Config::rejectStereoThresh)
            status[i] = 0;
    }
}

void ImageProcessor::rejectStereoOutliers(const vector<Point2f>& feature1, const vector<uchar>& status1, const vector<Point2f>& feature2, const vector<uchar>& status2, vector<uchar>& status, const Matrix3f& fundMatrix)
{
    Vector3f v1, v2;
    float result;
    int N = feature1.size();
    for(int i = 0; i < N && status1[i] != 0 && status2[i] != 0; i++){
        v1 = Vector3f(feature1[i].x, feature1[i].y, 1);
        v2 = Vector3f(feature2[i].x, feature2[i].y, 1);
        result = v2.dot(fundMatrix*v1);
        if(abs(result) < 100)
            status[i] = 1;
    }
}

Vector3f ImageProcessor::triangulateStereoPoints(const Vector3f& firstObserv, const Vector3f& secondObserv, float& quality, const Matrix3f& R, const Vector3f& T)
{
    // calculate unrotated feature observation
    Vector3f U1, U2;
    U1 = firstObserv;
    U2 = R*secondObserv; // Nam - to do - is it R or R.inverse?

    Matrix3f Identity;
    Identity << 1,0,0,0,1,0,0,0,1;
    Matrix3f A1 = Identity - U1*U1.transpose();
    Matrix3f A2 = Identity - U2*U2.transpose();
    Matrix3f A = A1+A2;
    Matrix3f Ainv = A.inverse();    

    //  calculate and return position
    Vector3f pose = Ainv*(A2*T);

    // calculate triangulate quality
    Vector3f d1 = pose;
    d1.normalize();
    float e1 = d1.cross(U1).squaredNorm();
    Vector3f d2 = pose - T;
    d2.normalize();
    float e2 = d2.cross(U2).squaredNorm();
    quality = e1 + e2;

    return pose;
}

vector<Vector3f> ImageProcessor::computeStereoFeaturePose(const vector<KeyPoint> &keyPoints1, const vector<KeyPoint> &keyPoints2, vector<DMatch> &stereoMatches, const Matrix3f& K1, const Matrix3f& K2, const MatrixXf& D1, const MatrixXf& D2, const Matrix3f& R, const Vector3f& T, vector<float>& allQuality)
{
    vector<Vector3f> stereo3DPose;
    vector<Vector3f> observation1 = convertKeypointsToObservation(keyPoints1, stereoMatches, 1, K1, D1);
    vector<Vector3f> observation2 = convertKeypointsToObservation(keyPoints2, stereoMatches, 0, K2, D2);

    vector<DMatch> qualifiedStereoMatches;
    for(int i = 0; i < stereoMatches.size(); i++){
        float quality;
        Vector3f pose = triangulateStereoPoints(observation1[i], observation2[i], quality, R, T);
        if (checkStereoValidPose(pose) && quality < Config::minTriangulationQuality){
            stereo3DPose.push_back(pose);
            qualifiedStereoMatches.push_back(stereoMatches[i]);
            allQuality.push_back(quality);
        }
    }
    stereoMatches.clear();
    stereoMatches = qualifiedStereoMatches;
    return stereo3DPose;
}

void ImageProcessor::trackFeaturesKLT(const Mat& prevFrame, const Mat& currFrame, const vector<Point2f>& prevFeatures, vector<Point2f>& currFeatures, vector<uchar>& status, vector<float>& error, Size& winSizeTrack, TermCriteria& termcritTrack, float minEigenThres)
{
    calcOpticalFlowPyrLK(prevFrame, currFrame, prevFeatures, currFeatures, status, error, winSizeTrack, 3, termcritTrack, minEigenThres);
}

Vector3f ImageProcessor::computeCameraPose(const vector<Feature>& pointCloud, const Vector3f& preCameraPose, const Matrix3f& R, const int thresInliers, const vector<float>& trackError)
{
    // set current camera pos to the previous value
    Vector3f currCameraPose = preCameraPose;

    // check if we have features
    int size = pointCloud.size();
    if (pointCloud.size()==0){
        cout << "point cloud has no feature" << endl;
        return currCameraPose;
    }

    // compute distance d = || p - (r)t-1 || for each feature
    float di;
    Vector3f uij;
    vector<Matrix3f> allA;
    vector<Vector3f> allAp;
    vector<Vector3f> allUij;
    Matrix3f Ai;
    for (int i = 0; i < size; i++){        
        di = (pointCloud[i].pose3D - preCameraPose).squaredNorm();
        uij = R*pointCloud[i].observation.back();
        uij.normalize();
        Ai = Matrix3f::Identity() - uij*uij.transpose();
        Ai /= di;
        allA.push_back(Ai);
        allAp.push_back(Ai*pointCloud[i].pose3D);
        allUij.push_back(uij);
    }

    vector<int> resultInliers;
    Utility::RANSAC ransac;    
    ransac.ransac2PointsCamPose(allA, allAp, allUij, pointCloud, currCameraPose, resultInliers, thresInliers, trackError);

    return currCameraPose;
}

vector<Point2f> ImageProcessor::computeStatistic(vector<Vector3f> v)
{
    float meanX = 0;
    float meanY = 0;
    float meanZ = 0;
    int size = v.size();
    for (int i = 0; i < size; i++){
        meanX += v[i](0)/size;
        meanY += v[i](1)/size;
        meanZ += v[i](2)/size;
    }

    float stddevX = 0;
    float stddevY = 0;
    float stddevZ = 0;
    for (int i = 0; i < size; i++){
        stddevX += pow(v[i](0) - meanX, 2)/size;
        stddevY += pow(v[i](1) - meanY, 2)/size;
        stddevZ += pow(v[i](2) - meanZ, 2)/size;
    }
    stddevX = sqrt(stddevX);
    stddevX = sqrt(stddevX);
    stddevX = sqrt(stddevX);

    vector<Point2f> statistic;
    statistic. push_back(Point2f(meanX, stddevX));
    statistic. push_back(Point2f(meanY, stddevY));
    statistic. push_back(Point2f(meanZ, stddevZ));

    return statistic;
}

Point2f ImageProcessor::computeStatistic(vector<float> v)
{
    vector<float> tmp;
    for (auto it = v.begin(); it != v.end(); it++){
        if (*it != 0)
            tmp.push_back(*it);
    }
    v = tmp;
    float mean = 0;
    int size = v.size();
    for (int i = 0; i < size; i++){
        mean += v[i]/size;
    }

    float stddev = 0;
    for (int i = 0; i < size; i++){
        stddev += pow(v[i] - mean, 2)/size;
    }
    stddev = sqrt(stddev);

    Point2f statistic = Point2f(mean, stddev);

    return statistic;
}

bool ImageProcessor::checkStereoValidPose(Vector3f& point)
{
    if(point(0) == 0 && point(1) == 0 && point(2) == 0)
        return false;
    if(point(2) < 0 || point(2) > Config::maxDepth)
        return false;
    return true;
}

vector<Vector3f> ImageProcessor::convertKeypointsToObservation(const vector<KeyPoint>& keyPoints, vector<DMatch> matches, bool primary, const Matrix3f& K, const MatrixXf& D)
{
    vector<Point2f> features = extractMatchedFeatures(keyPoints, matches, primary);
    vector<Vector3f> observation = convert2DFeaturesToObservation(features, K, D);
    return observation;
}

vector<Point2f> ImageProcessor::extractMatchedFeatures(const vector<KeyPoint> keyPoints, const vector<DMatch> matches, bool primary)
{
    vector<Point2f> features;
    for (int i = 0; i < matches.size(); i++){
        if (primary)
            features.push_back(keyPoints[matches[i].queryIdx].pt);
        else
            features.push_back(keyPoints[matches[i].trainIdx].pt);
    }
    return features;
}

vector<Vector3f> ImageProcessor::convert2DFeaturesToObservation(const vector<Point2f> features, const Matrix3f& K, const MatrixXf& D)
{
    // Undistort Points for observation in normalized image coordinates
    Mat toMat(features);
    Mat KMat, DMat;
    eigen2cvMat(K,KMat);
    eigen2cvMat(D,DMat);
    cv::undistortPoints(toMat, toMat, KMat, DMat);

    // normalize and push into observation vector
    vector<Vector3f> observation;
    for (int i = 0; i < features.size(); i++){
        Point2f p;
        p = toMat.at<Point2f>(i,0);
        Vector3f obs(p.x, p.y, 1);
        obs.normalize();
        observation.push_back(obs);
    }

    return observation;
}

vector<Point2f> ImageProcessor::convertKeyPointToPoint2f(const vector<KeyPoint>& keyP)
{
    vector<Point2f> p;
    int size = keyP.size();
    for (int i = 0; i < size; i++){
        p.push_back(keyP[i].pt);
    }
    return p;
}

vector<KeyPoint> ImageProcessor::convertPoint2fToKeyPoint(const vector<Point2f>& p)
{
    vector<KeyPoint> keyP;
    Point2f tmp;
    for(int i = 0; i < p.size(); i++ ){
        tmp = p[i];
        keyP.push_back(KeyPoint(tmp, 1.f));
    }
    return keyP;
}

template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
void ImageProcessor::eigen2cvMat(const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& src, cv::Mat& dst)
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

