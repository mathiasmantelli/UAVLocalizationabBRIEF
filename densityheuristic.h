#ifndef DENSITYHEURISTIC_H
#define DENSITYHEURISTIC_H

#include "Heuristic.h"
#include "angleutil.h"

// Creating a struct for heuristic config
struct heuristicType{
    // variables
    STRATEGY strategy;
    int kernelType;
    double radius;
    int colorDifference;
    double threshold;
    COLOR_NAME color;

    // constructor
    heuristicType(STRATEGY s = DENSITY, int kt=KGAUSSIAN, double r=1.0, int cd=-1, double t=2.3, COLOR_NAME color=BLACK) :
        strategy(s),
        kernelType(kt),
        radius(r),
        colorDifference(cd),
        threshold(t),
        color(color)
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
    virtual double calculateValue(int x, int y, Mat *image, Mat* map);

    vec3 calculateMeanColor(Mat &image, Mat &map);
    double calculateMeanDifference(Mat &image, Mat &map);
    void setLimiarAsMeanDifference(Mat &image, Mat &map);

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
    double calculateValue(int x, int y, Mat *image, Mat* map);

private:
    vec3 color;
};

#endif // DENSITYHEURISTIC_H
