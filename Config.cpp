#include "Config.h"

// GroundTruth source file
string Config::groundTruthFile = "/home/nam/workingroom/opencv_project/BinocularVIO/stereoSLAM-TestVideos/groundtruth.txt";

// Config for detect feature algorithm
char* Config::featureDetector = "FAST";
char* Config::featureExtractor = "BRIEF";
char* Config::featureMatcher = "BruteForce";
int Config::detectorThreshold = 25; // 25

// Config for tracking
int Config::maxFeatureTrack = 500;
TermCriteria Config::termcritTrack(1 | 2, 40, 0.0001);
Size Config::winSizeTrack(20, 20); // 20 20
float Config::minEigenThres = 0.0001;
float Config::maxTrackingError = 10; // 10

// stereo threshold for outliers rejection
float Config::rejectStereoThresh = 0.5;
float Config::minTriangulationQuality = 0.00004; // 0.00004
float Config::maxDepth = 10; // 10

// RANSAC for outliers reject pose estimation
int Config::loopRANSAC = 50;
int Config::numPointsRANSAC = 2;
float Config::threshErrRANSAC = 1; // 0.1
float Config::threshSumErrRANSAC = 1; // 0.1
int Config::minNumOfInliersRANSAC = 5;
int Config::maxNumOfInliersRANSAC = 30;
float Config::percentage = 0.25;

// threshold for feature removal post pose estimation
float Config::reprojectionErrorThres = 5; // 5

// failure recovery
int Config::minNumOfStereoFeatures = 1;
int Config::minNumOfTrackedFeatures = 20; // 20
float Config::percentOfTrackedFeatures = 0.5; // 0.5




