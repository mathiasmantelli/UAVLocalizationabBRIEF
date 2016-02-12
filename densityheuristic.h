#ifndef DENSITYHEURISTIC_H
#define DENSITYHEURISTIC_H

#include "Heuristic.h"
//#include "angleutil.h"

// Creating a struct for heuristic config
struct heuristicType{
    // variables
    STRATEGY strategy;
    int kernelType;
    double radius;
    int colorDifference;
    double threshold;
    COLOR_NAME color;

    //brief variables
    float lowThreshold;
    float multiplierThreshold;
    int numberPairs;
    int margin;

    // constructor
    heuristicType(STRATEGY s = DENSITY, int kt=KGAUSSIAN, double r=1.0, int cd=-1, double t=2.3, COLOR_NAME color=BLACK, float blt = -1, float bmt = -1, int bp = -1, int bm = -1) :
        strategy(s),
        kernelType(kt),
        radius(r),
        colorDifference(cd),
        threshold(t),
        color(color),
        lowThreshold(blt),
        multiplierThreshold(bmt),
        numberPairs(bp),
        margin(bm)
    {}
};

std::string colorDifferenceName(int colorDiffID);
std::string kernelName(int k);
std::string strategyName(STRATEGY s);
std::string getColorName(COLOR_NAME cn);

class DensityHeuristic : public KernelHeuristic
{
public:
    DensityHeuristic(STRATEGY s, int id, double *kernel,int kWidth, int kHeight,int radius, double l=5.0, unsigned int cd=INTENSITYC);
    virtual double calculateValue(int x, int y, cv::Mat *image, cv::Mat* map);

    vec3 calculateMeanColor(cv::Mat &image, cv::Mat &map);
    double calculateMeanDifference(cv::Mat &image, cv::Mat &map);
    void setLimiarAsMeanDifference(cv::Mat &image, cv::Mat &map);

    double sech5(double val);

protected:

    // Image density stuff
    CPUColorConverter convert;
    vec3 color_mean;
    vec3 color_stdev;
    double mean_diff;
};

class SingleColorDensityHeuristic: public DensityHeuristic
{
public:
    SingleColorDensityHeuristic(STRATEGY s, int id, double *kernel,int kWidth, int kHeight,int radius, double l=5.0, unsigned int cd=INTENSITYC, COLOR_NAME colorName=WHITE);
    double calculateValue(int x, int y, cv::Mat *image, cv::Mat* map);

private:
    vec3 color;
};

#endif // DENSITYHEURISTIC_H
