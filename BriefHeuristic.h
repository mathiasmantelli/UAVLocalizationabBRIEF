#ifndef BRIEFHEURISTIC_H
#define BRIEFHEURISTIC_H

#include "Heuristic.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "ColorCPU.h"
#include "RadiusVolumeTransferFunctions.h"

#include "Utils.h"

class BriefHeuristic : public Heuristic
{
public:
    BriefHeuristic(STRATEGY s, int id, int cd, double l);

    double calculateValue(int x, int y, cv::Mat *image, cv::Mat* map=NULL);
    double calculateValue2(Pose p, cv::Mat *map);

    void updateDroneDescriptor(cv::Mat& drone);

    void printInfo();

    int totalPairs;
    float lowThreshold;
    float multiplierThreshold;
    int margin;
    int width, height;
    vector<int> droneDescriptor;

    vector< vector<cv::Point> > pairs;

    cv::Point transform(cv::Point pt, cv::Mat rot, cv::Point trans, int max_x, int max_y);
};

#endif // BRIEFHEURISTIC_H
