#ifndef HEURISTIC_H
#define HEURISTIC_H

class Heuristic;

#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
using namespace cv;


#include <climits>
#include <float.h>

#define HEURISTIC_UNDEFINED -DBL_MAX
#define HEURISTIC_UNDEFINED_INT INT_MIN
#define QUANTIZATION_LEVELS 255

//#define IS_UNDEF(X) fabs(X - HEURISTIC_UNDEFINED) < 0.00001
#define IS_UNDEF(X) (X == HEURISTIC_UNDEFINED)

class MapGrid;

class Heuristic
{
    public:
        virtual double calculateValue(int x, int y, Mat *image, Mat* map) = 0;
        virtual double calculateGradientOrientation(int x, int y, Mat *image, Mat* map) = 0;
        virtual double calculateGradientSobelOrientation(int x, int y, Mat *image, Mat* map) = 0;
};

#endif
