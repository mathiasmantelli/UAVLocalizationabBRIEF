#ifndef COLORHEURISTIC_H
#define COLORHEURISTIC_H

#include "Heuristic.h"
#include "ColorCPU.h"
#include "RadiusVolumeTransferFunctions.h"

class ColorHeuristic : public Heuristic
{
public:
    ColorHeuristic(int cd, double limiar);
    double calculateValue(int x, int y, Mat *image, Mat *map=NULL);
    double calculateGradientSobelOrientation(int x, int y, Mat *image, Mat *map);
    double calculateGradientOrientation(int x, int y, Mat *image, Mat *map);

    // Get methods
    vec3 getValuefromPixel(int x, int y, Mat *image);
    double getLimiar();
    int getColorDifference();

    // set methods
    void setLimiar(double val);
    void setColorDifference(double val);
    void setBaselineColor(int x, int y, Mat *image);
    void setTestedColor(int x, int y, Mat *image); // receives rgb image

    // baseline color
    vec3 baselineColor;
    vec3 testedColor;
private:
    // Image density stuff
    double limiar;
    unsigned int color_difference;
    CPUColorConverter convert;


};

#endif // COLORHEURISTIC_H
