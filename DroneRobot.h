#ifndef DRONEROBOT_H
#define DRONEROBOT_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

#include "Robot.h"
#include "densityheuristic.h"

class DroneRobot: public Robot
{
public:
    DroneRobot();
    DroneRobot(string& mapPath, string& trajectoryPath, vector< heuristicType* > &heuristicTypes);
    ~DroneRobot();

    void initialize(ConnectionMode cmode, LogMode lmode, string fname);
    void run();

    int selectMapID(int colorDiff);
    void createColorVersions(Mat& imageRGB);


private:

    void generateObservations(string imagePath);
    bool readRawOdometryFromFile(Pose& p);
    Pose readOdometry();
    Pose findOdometry(Mat &prevImage, Mat &curImage);

    bool offlineOdom;
    Pose prevRawOdom;
    fstream odomFile;

    vector<string> imagesNames;
    vector<DensityHeuristic*> heuristics;
    vector<MapGrid*> maps;
    vector<Mat> globalMaps;
    Mat prevMap;
    vector<Mat> mapsColorConverted;

    unsigned int step;

};

#endif // DRONEROBOT_H
