#ifndef CORRELATIVESM_H
#define CORRELATIVESM_H

#include<queue>
using namespace std;

#include <Eigen/Dense>
using namespace Eigen;

#include "Utils.h"
#include "DistanceTransform.h"
//#include "DroneRobot.h"


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

class CorrelativeScanMatching
{
public:
    CorrelativeScanMatching(int rows, int cols);
    void compute(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_prev, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cur, Pose& outPose, Matrix3d& outCov);
    void draw(int t);

private:
    // Configuration Parameters
    double stepD, stepTh;
    double deltaD, deltaTh;
    double maxLaserRange;

    // Private functions
    void clearLikelihoodMap();
    void updateLikelihoodMap();
    void updateDistanceMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_prev, const Pose p);
    void updatePoints(vector<pair<int, int> > &points, int w, int h, bool insert);
    void updateBoundaries(const Pose p1, const Pose p2);
    void matchingWith2DSlices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cur, Pose& outPose, Matrix3d& outCov);
//    vector<pair<int,int> > rayCasting(const vector<double>& readings, int xc, int yc, double th);
    vector<pair<int,int> > transformCloudPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_prev, int xc, int yc, double th);


    // Private attributes
    vector<vector<bool> > occupationMap;
    vector<vector<bool> > occupationMap2;
    vector<vector<double> > distMap;
    vector<vector<double> > normDistMap;
    vector<vector<double> > likelihoodMap;
    vector<vector<double> > logLikelihoodMap;
    vector<vector<double> > normLikelihoodMap;
    vector<vector<vector<double>  > > matchingMap;
    vector<vector<double> > normMatchingMap;

    int mapHeight, mapWidth;
    int windowHeight, windowWidth, windowDeltaTh;
    double margin;
    int centerX, centerY;

    // Search boundaries
    int minX, minY;
    double minTh;

    // Gaussian stuff -- for the likelihood map
    double gaussStdev;
    double gaussVar;
    double gaussNu;
    double minValue;
    double logMinValue;

    Trigonometry trig;
    DistanceTransform dt;
    Timer timer;
};

#endif // CORRELATIVESM_H
