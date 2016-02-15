#ifndef COLORHEURISTIC_H
#define COLORHEURISTIC_H

#include "BriefHeuristic.h"
#include "Heuristic.h"
#include "ColorCPU.h"
#include "RadiusVolumeTransferFunctions.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "Utils.h"

class ColorHeuristic : public Heuristic
{
public:
    ColorHeuristic(STRATEGY s, int id, int cd, double limiar);
    double calculateValue(int x, int y, cv::Mat *image, cv::Mat *map=NULL);
    double calculateValue2(int x, int y, cv::Mat *image, cv::Mat *map=NULL);

    // set methods
    void setBaselineColor(int x, int y, cv::Mat *image);
    void setTestedColor(int x, int y, cv::Mat *image); // receives rgb image

    // baseline color
    vec3 baselineColor;
    vec3 testedColor;
protected:
    // Image density stuff
    CPUColorConverter convert;


};

class UnscentedColorHeuristic: public ColorHeuristic
{
public:
    UnscentedColorHeuristic(STRATEGY s, int id, int cd, double limiar);

    double calculateValue(int x, int y, Pose p, cv::Mat *image, cv::Mat *map=NULL);

    // set methods
    void setBaselineColors(int x, int y, cv::Mat *image);

    cv::Point transform_rotation(cv::Point pt, cv::Mat rot, cv::Point trans);

    // baseline color
    vector<vec3> baselineColors;
    int delta;
    int deltax;
    int deltay;

private:

    double computeDiffColor(int x, int y, int whichPoint, cv::Mat *image, cv::Mat *map=NULL);


};

#endif // COLORHEURISTIC_H
