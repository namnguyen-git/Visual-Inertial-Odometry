#include <LocalMap.h>
#include <utils.h>

LocalMap::LocalMap()
{
}

void LocalMap::add(Feature& featureToAdd)
{
    pointCloud.push_back(featureToAdd);
}

void LocalMap::updatePose2D(vector<Point2f>& feature2D, vector<uchar>& mask)
{
    for (int i = 0; i < mask.size(); i++){
        if (mask[i])
            pointCloud[i].pose2D.push_back(feature2D[i]);
    }
}

void LocalMap::updateObservation(vector<Vector3f>& obs, vector<uchar>& mask)
{
    for (int i = 0; i < mask.size(); i++){
        if (mask[i])
            pointCloud[i].observation.push_back(obs[i]);
    }
}

void LocalMap::deleteLostFeatures(vector<uchar>& mask)
{
    if (pointCloud.size() != mask.size()){
        cout << "delete lost features: size do not match!" << endl;
        return;
    }

    vector<Feature> tmp;
    for (int i = 0; i < mask.size(); i++){
        if (mask[i])
            tmp.push_back(pointCloud[i]);
    }
    pointCloud.clear();
    pointCloud = tmp;
}


vector<Feature> LocalMap::getPointCloud()
{
    return pointCloud;
}

void LocalMap::clearPointCould()
{
    if (!pointCloud.empty())
        pointCloud.clear();
}



