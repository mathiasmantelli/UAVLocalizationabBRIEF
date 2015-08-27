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
    void createColorVersions(Mat& imageRGB);


private:

    void generateObservations(string imagePath);
    bool readRawOdometryFromFile(Pose& p);
    Pose readOdometry();
    Pose findOdometry(Mat &prevImage, Mat &curImage);

    bool offlineOdom;
    Pose prevRawOdom;
    fstream odomFile;

    STRATEGY locTechnique;

    // Heuristics vectors
    vector<ColorHeuristic*> ssdHeuristics;
    vector<ColorHeuristic*> colorHeuristics;
    vector<DensityHeuristic*> densityHeuristic;

    vector<Mat> globalMaps;
    vector<Mat> mapsColorConverted;
    vector<string> imagesNames;

    vector<MapGrid*> maps;

    Mat prevMap;

    unsigned int step;

};

#endif // DRONEROBOT_H
