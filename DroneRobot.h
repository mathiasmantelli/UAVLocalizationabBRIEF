#ifndef DRONEROBOT_H
#define DRONEROBOT_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

#include "Robot.h"
#include "densityheuristic.h"
#include "colorheuristic.h"
#include "miheuristic.h"

int selectMapID(int colorDiff);

class DroneRobot: public Robot
{
public:
    DroneRobot();
    DroneRobot(string& mapPath, string& trajectoryPath, vector< heuristicType* > &heuristicTypes);
    ~DroneRobot();

    void initialize(ConnectionMode cmode, LogMode lmode, string fname);
    void run();

private:

    int getNextProperHeuristicID(STRATEGY type);
    void generateObservations(string imagePath);
    bool readRawOdometryFromFile(Pose& p);
    Pose readOdometry();
    Pose findOdometry(Mat &prevImage, Mat &curImage);
    Pose findOdometryUsingFeatures(Mat &prevImage, Mat &curImage);
    void drawMatchedImages(Mat& prevImage, Mat& curImage, Mat& warp_matrix, const int warp_mode = MOTION_EUCLIDEAN);
    void localizeWithTemplateMatching(Mat &currentMap);

    bool offlineOdom;
    Pose prevRawOdom;
    fstream odomFile;

    STRATEGY locTechnique;

    // Heuristics vectors
    vector<Heuristic*> heuristics;
    vector<MapGrid*> cachedMaps;

//    vector<ColorHeuristic*> ssdHeuristics;
//    vector<ColorHeuristic*> colorHeuristics;
//    vector<DensityHeuristic*> densityHeuristic;

    vector<Mat> globalMaps;
    vector<string> imagesNames;


    Mat prevMap;

    unsigned int step;

};

#endif // DRONEROBOT_H
