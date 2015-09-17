#ifndef HEURISTIC_H
#define HEURISTIC_H

class Heuristic;

#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
//using namespace cv;


#include <climits>
#include <float.h>

#include "ColorCPU.h"
#include "RadiusVolumeTransferFunctions.h"

#define HEURISTIC_UNDEFINED -DBL_MAX
#define HEURISTIC_UNDEFINED_INT INT_MIN
#define QUANTIZATION_LEVELS 255

//#define IS_UNDEF(X) fabs(X - HEURISTIC_UNDEFINED) < 0.00001
#define IS_UNDEF(X) (X == HEURISTIC_UNDEFINED)

// Heuristics
enum STRATEGY{
    SSD,
    COLOR_ONLY,
    ENTROPY,
    MUTUAL_INFORMATION,
    DENSITY,
    MEAN_SHIFT,
    SINGLE_COLOR_DENSITY,
    DENSITY_LOCALCOLORDIFF,
    CREATE_OBSERVATIONS,
    TEMPLATE_MATCHING,
    FEATURE_MATCHING,
    SIFT_MCL,
};

class Heuristic
{
public:
    Heuristic(STRATEGY s, int id, double l, unsigned int cd);

    virtual double calculateValue(int x, int y, cv::Mat *image, cv::Mat* map=NULL) = 0;

    double calculateGradientOrientation(int x, int y, cv::Mat *image, cv::Mat* map);
    double calculateGradientSobelOrientation(int x, int y, cv::Mat *image, cv::Mat* map);

    STRATEGY getType();
    double getLimiar();
    int getID();
    int getColorDifference();
    void setLimiar(double val);
    void setColorDifference(double val);

    vec3 getValuefromPixel(int x, int y, cv::Mat *image);

protected:
    STRATEGY type;
    int id;
    double limiar;
    unsigned int color_difference;
};

class KernelHeuristic : public Heuristic
{
public:
    KernelHeuristic(STRATEGY s, int id, double l, unsigned int cd, int rad, double* k, int kW, int kH);

    virtual double calculateValue(int x, int y, cv::Mat *image, cv::Mat* map) = 0;

    double getRadius();

protected:

    int radius;
    double *kernel;
    int kWidth;
    int kHeight;
};

#endif
