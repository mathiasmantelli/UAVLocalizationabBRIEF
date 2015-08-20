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

private:

    void generateObservations(string path);
    Pose findOdometry(Mat &prevImage, Mat &curImage);

    vector<string> imagesNames;
    vector<Heuristic*> heuristics;
    vector<MapGrid*> maps;
    vector<Mat> globalMaps;
    Mat prevMap;

    unsigned int step;

};

#endif // DRONEROBOT_H
