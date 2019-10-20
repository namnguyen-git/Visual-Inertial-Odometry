#ifndef UTILS_H
#define UTILS_H

#define EIGEN_NO_DEBUG
#include <iostream>
#include <fstream>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <ctime>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/Eigen/Core>
#include <opencv2/Eigen/Dense>
#include <opencv2/Eigen/Eigen>
#include <opencv2/Eigen/StdVector>
#include <opencv2/Eigen/Geometry>

using namespace std;
using namespace cv;
using namespace Eigen;

#include <Config.h>
#include <CameraCalib.h>
#include <IMUCalib.h>

struct Feature{
    Vector3f pose3D;
    vector<Point2f> pose2D;
    vector<Vector3f> observation; // 2D pixel location in homogeneous coordinate (x,y,1) in different frames
};

#endif // UTILS_H


