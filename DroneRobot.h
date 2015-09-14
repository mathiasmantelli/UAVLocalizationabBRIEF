#ifndef DRONEROBOT_H
#define DRONEROBOT_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

#include "Robot.h"
#include "densityheuristic.h"
#include "colorheuristic.h"
#include "miheuristic.h"
#include "MeanShiftHeuristic.h"

int selectMapID(int colorDiff);

class DroneRobot: public Robot
{
public:
    DroneRobot();
    DroneRobot(string& mapPath, string& trajectoryPath, vector< heuristicType* > &heuristicTypes, bool quiet, string& outputName, int start, int finish);
    ~DroneRobot();

    void initialize(ConnectionMode cmode, LogMode lmode, string fname);
    void run();

private:

    int getNextProperHeuristicID(STRATEGY type);
    void generateObservations(string imagePath);
    bool readRawOdometryFromFile(Pose& p);
    Pose readOdometry();
    pair<Pose, bool> readOdometryNew();
    Pose readGroundTruth();
    pair<Pose, bool> findOdometryUsingECC(Mat &prevImage, Mat &curImage);
    pair<Pose, bool> findOdometry(Mat &prevImage, Mat &curImage);
    pair<Pose, bool> findOdometryUsingFeatures(Mat &prevImage, Mat &curImage, double cT=0.04);
    void drawMatchedImages(Mat& prevImage, Mat& curImage, const Mat& warp_matrix, const int warp_mode = MOTION_EUCLIDEAN);

    void reinitialize();
    void initializeFeatureMatching();
    void localizeWithTemplateMatching(Mat &currentMap);
    void localizeWithFeatureMatching(Mat& currentMap);
    void localizeWithHierarchicalFeatureMatching(Mat& currentMap);

    bool slowMethod;
    bool offlineOdom;
    bool isRawOdom;
    bool availableGTruth;
    Pose prevRawOdom;
    Pose prevOdometry;
    Pose realPose;
    fstream odomFile;
    fstream truthFile;

    int start;
    int finish;
    int current;
    string rawname;
    string outputName;

    STRATEGY locTechnique;

    // Heuristics vectors
    vector<Heuristic*> heuristics;
    vector<MapGrid*> cachedMaps;

//    vector<ColorHeuristic*> ssdHeuristics;
//    vector<ColorHeuristic*> colorHeuristics;
//    vector<DensityHeuristic*> densityHeuristic;

    vector<Mat> globalMaps;
    vector<string> imagesNames;

    // Used for feature matching
    Ptr<Feature2D> feature_detector;
    Ptr<Feature2D> feature_extractor;
    FlannBasedMatcher feature_matcher;
    std::vector<KeyPoint> keypoints_globalMap;
    Mat descriptors_globalMap;
    vector< vector<FlannBasedMatcher*> > hMatcher;
    vector< vector< vector<unsigned int>* > > idKeypoints;
    Mat likelihood;

    Mat prevMap;

    unsigned int step;

};

#endif // DRONEROBOT_H
