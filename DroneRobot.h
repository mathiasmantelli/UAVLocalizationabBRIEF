#ifndef DRONEROBOT_H
#define DRONEROBOT_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

#include "Robot.h"

class DroneRobot: public Robot
{
public:
    DroneRobot();
    ~DroneRobot();

    void initialize(ConnectionMode cmode, LogMode lmode, string fname);
    void run();

private:

    Pose findOdometry(Mat &prevImage, Mat &curImage);

    string dataset;
    vector<string> imagesNames;
    Mat globalMap;
    Mat prevMap;

    unsigned int step;

};

#endif // DRONEROBOT_H
