#ifndef ANGLEUTIL_H
#define ANGLEUTIL_H

#include "math.h"

#define DEG2RAD(a) ((a) * M_PI / 180.0)
#define RAD2DEG(a) ((a) * 180.0 / M_PI)

class AngleUtil
{
public:
    AngleUtil();
    static double getMinAngleDiff(double aRad, double bRad);
    static double correctAngle(double a);
private:
        static double correctMod(double v, double n);
};

#endif // ANGLEUTIL_H
