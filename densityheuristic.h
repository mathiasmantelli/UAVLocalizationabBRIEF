#ifndef DENSITYHEURISTIC_H
#define DENSITYHEURISTIC_H

#include "Heuristic.h"
#include "angleutil.h"
#include "ColorCPU.h"
#include "RadiusVolumeTransferFunctions.h"

// Heuristics
enum STRATEGY{
    SSD,
    COLOR_ONLY,
    DENSITY,
    DENSITY_LOCALCOLORDIFF
};
string colorDifferenceName(int colorDiffID);
string strategyName(STRATEGY s);
string kernelName(int k);

// Creating a struct for heuristic config
struct heuristicType{
    // variables
    STRATEGY strategy;
    int kernelType;
    double radius;
    int colorDifference;
    double threshold;

    // constructor
    heuristicType(STRATEGY s = DENSITY, int kt=KGAUSSIAN, double r=1.0, int cd=-1, double t=2.3) :
        strategy(s),
        kernelType(kt),
        radius(r),
        colorDifference(cd),
        threshold(t)
    {}
};

class DensityHeuristic : public Heuristic
{
public:
    DensityHeuristic(double *kernel,int kWidth, int kHeight,int radius, double l=5.0, unsigned int cd=INTENSITYC);
    double calculateValue(int x, int y, Mat *image, Mat* map);
    double calculateGradientOrientation(int x, int y, Mat *image, Mat* map);
    double calculateGradientSobelOrientation(int x, int y, Mat *image, Mat* map);

    vec3 calculateMeanColor(Mat &image, Mat &map);
    double calculateMeanDifference(Mat &image, Mat &map);
    void setLimiarAsMeanDifference(Mat &image, Mat &map);

    double sech5(double val);

    // get values
    vec3 getValuefromPixel(int x, int y, Mat *image);
    double getRadius();
    double getLimiar();
    int getColorDifference();

    // set values
    void setLimiar(double val);


private:


    int radius;
    double *kernel;
    int kWidth;
    int kHeight;

    // Image density stuff
    double limiar;
    unsigned int color_difference;
    CPUColorConverter convert;
    vec3 color_mean;
    vec3 color_stdev;
    double mean_diff;
};

#endif // DENSITYHEURISTIC_H
