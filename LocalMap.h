#ifndef LOCALMAP_H
#define LOCALMAP_H
#include <utils.h>



class LocalMap
{
private:
    vector<Feature> pointCloud;

public:
    LocalMap();
    void add(Feature& featureToAdd);
    void remove (Feature& featureToRemove);
    void updatePose2D(vector<Point2f>& feature2D, vector<uchar>& status);
    void updateObservation(vector<Vector3f>& observation, vector<uchar>& status);
    void deleteLostFeatures(vector<uchar>& mask);
    vector<Feature> getPointCloud();
    void clearPointCould();
};

#endif // LOCALMAP_H
