#include "angleutil.h"

AngleUtil::AngleUtil()
{
}

double AngleUtil::correctMod(double v, double n){
    return fmod( (fmod(v,n) + n), n);
}

double AngleUtil::getMinAngleDiff(double aRad, double bRad) {
    double angle = aRad - bRad;
    angle =  correctMod((angle + M_PI), 2*M_PI) - M_PI;
    return angle;
}

double AngleUtil::correctAngle(double a) {
    return getMinAngleDiff(a, 0.0);
}
