#ifndef DRONEROBOT_H
#define DRONEROBOT_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//using namespace cv;

#include "Robot.h"
#include "densityheuristic.h"
#include "colorheuristic.h"
#include "miheuristic.h"
#include "MeanShiftHeuristic.h"
#include "SiftHeuristic.h"
#include "BriefHeuristic.h"

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
    pair<Pose, bool> findOdometryUsingECC(cv::Mat &prevImage, cv::Mat &curImage);
    pair<Pose, bool> findOdometry(cv::Mat &prevImage, cv::Mat &curImage);
    pair<Pose, bool> findOdometryUsingFeatures(cv::Mat &prevImage, cv::Mat &curImage, double cT=0.04);
    void drawMatchedImages(cv::Mat& prevImage, cv::Mat& curImage, const cv::Mat& warp_matrix, const int warp_mode = cv::MOTION_EUCLIDEAN);

    void reinitialize();
    void initializeFeatureMatching();
    void localizeWithTemplateMatching(cv::Mat &currentMap);
    void localizeWithFeatureMatching(cv::Mat& currentMap);
    void localizeWithHierarchicalFeatureMatching(cv::Mat& currentMap);

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

    vector<cv::Mat> globalMaps;
    vector<string> imagesNames;

    // Used for feature matching
    cv::Ptr<cv::Feature2D> feature_detector;
    cv::Ptr<cv::Feature2D> feature_extractor;
    cv::FlannBasedMatcher feature_matcher;
    std::vector<cv::KeyPoint> keypoints_globalMap;
    cv::Mat descriptors_globalMap;
    vector< vector<cv::FlannBasedMatcher*> > hMatcher;
    vector< vector< vector<unsigned int>* > > idKeypoints;
    cv::Mat likelihood;

    cv::Mat prevMap;

    unsigned int step;

};

#endif // DRONEROBOT_H
