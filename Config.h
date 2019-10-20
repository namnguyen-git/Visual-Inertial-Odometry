#ifndef CONFIG_H
#define CONFIG_H
#include "utils.h"

using namespace std;

class Config
{
public:
    // Groundtruth source file
    static string groundTruthFile;

    // Config for detect feature algorithm
    static char* featureDetector;
    static char* featureExtractor;
    static char* featureMatcher;
    static int detectorThreshold;

    // Config for tracking
    static int maxFeatureTrack;   
    static TermCriteria termcritTrack;
    static Size winSizeTrack;
    static float minEigenThres;
    static float maxTrackingError;

    // stereo threshold for outliers rejection
    static float rejectStereoThresh;
    static float minTriangulationQuality;
    static float maxDepth;

    // RANSAC for outliers reject  pose estimation
    static int loopRANSAC;          // RANSAC iterations
    static int numPointsRANSAC;     // num of points using for RANSAC
    static float threshErrRANSAC;        // threshold for choose inliers RANSAC
    static float threshSumErrRANSAC;     // err need to be choose
    static int minNumOfInliersRANSAC;  // min number of inliers need to be choose
    static int maxNumOfInliersRANSAC;  // max number of inliers need to be choose
    static float percentage; // ratio between num of inliers required and num of features available

    // threshold for feature removal post pose estimation
    static float reprojectionErrorThres;

    // failure recovery
    static int minNumOfStereoFeatures;
    static int minNumOfTrackedFeatures;
    static float percentOfTrackedFeatures;    
};


#endif // CONFIG_H
