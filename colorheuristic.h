#ifndef COLORHEURISTIC_H
#define COLORHEURISTIC_H

#include "Heuristic.h"
#include "ColorCPU.h"
#include "RadiusVolumeTransferFunctions.h"

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
private:
    // Image density stuff
    CPUColorConverter convert;
};

#endif // COLORHEURISTIC_H
