#include "colorheuristic.h"

ColorHeuristic::ColorHeuristic(int color_difference, double limiar) :
    limiar(limiar),
    color_difference(color_difference)
{
}

vec3 ColorHeuristic::getValuefromPixel(int x, int y, Mat *image)
{
    if(image->type() != CV_32FC3)
    {
        Vec3b color = image->at<Vec3b>(y,x);
        double r = color[0];
        double g = color[1];
        double b = color[2];

        vec3 V(r, g, b);

        return V;
    }

    Vec3f color = image->at<Vec3f>(y,x);
    vec3 c(color[0], color[1], color[2]);
    return c;
}

double ColorHeuristic::calculateValue(int x, int y, Mat *image, Mat *map)
{
    //determine if pixel is in free region
    if(map!=NULL)
    {
        vec3 pos(getValuefromPixel(x,y,map));
        vec3 free(0.0,0.0,0.0);

        // approach an undef region?
        if(pos!=free)
            return HEURISTIC_UNDEFINED;
    }

    // Proceed with color difference computation
    double diff;
    setTestedColor(x,y,image);
    switch(color_difference){
    case INTENSITYC:
        diff=std::abs(baselineColor.x-testedColor.x);
        break;
    case CIELAB1976:
    case RGBNORMA:
        diff=convert.DeltaECIE1976(baselineColor,testedColor);
        break;
    case CMCLAB1984:
        diff=convert.DeltaECMC1984(baselineColor,testedColor);
        break;
    case CIELAB1994:
        diff=convert.DeltaECIE1994(baselineColor,testedColor);
        break;
    case CIELAB2000:
        diff=convert.DeltaECIE2000(baselineColor,testedColor);
        break;
    case CIELAB2000MIX:
        diff=convert.DeltaEMixCIE2000(baselineColor,testedColor);
        break;
    case CIELAB1994MIX:
        diff=convert.DeltaEMixCIE1994(baselineColor,testedColor);
        break;
    default:
        diff=convert.DeltaECIE1976(baselineColor,testedColor);
    }
    return diff/limiar;
}


// Apply mask
double ColorHeuristic::calculateGradientOrientation(int xCenter, int yCenter, Mat *image, Mat *map)
{
    // The direction of the
    double l = calculateValue(xCenter-1, yCenter, image, map);
    double r = calculateValue(xCenter+1, yCenter, image, map);
    double u = calculateValue(xCenter, yCenter+1, image, map);
    double d = calculateValue(xCenter, yCenter-1, image, map);

    if(IS_UNDEF(l) || IS_UNDEF(r) || IS_UNDEF(u) || IS_UNDEF(d))
        return HEURISTIC_UNDEFINED;

    double dx = r-l;
    double dy = u-d;

    if(fabs(dy) < 0.00001 && fabs(dx) < 0.00001)
        return HEURISTIC_UNDEFINED;

    return  atan2(dy, dx); //radians
}

double ColorHeuristic::calculateGradientSobelOrientation(int xCenter, int yCenter, Mat *image, Mat *map)
{
    // The direction of the
    double l  = calculateValue(xCenter-1, yCenter,   image, map);
    double r  = calculateValue(xCenter+1, yCenter,   image, map);
    double u  = calculateValue(xCenter,   yCenter+1, image, map);
    double d  = calculateValue(xCenter,   yCenter-1, image, map);
    double lu = calculateValue(xCenter-1, yCenter+1, image, map);
    double ru = calculateValue(xCenter+1, yCenter+1, image, map);
    double ld = calculateValue(xCenter-1, yCenter-1, image, map);
    double rd = calculateValue(xCenter+1, yCenter-1, image, map);

    if(IS_UNDEF(l) || IS_UNDEF(r) || IS_UNDEF(u) || IS_UNDEF(d) ||
       IS_UNDEF(lu) || IS_UNDEF(ru) || IS_UNDEF(ld) || IS_UNDEF(rd))
        return HEURISTIC_UNDEFINED;

    double dx = ru + 2*r + rd - lu - 2*l - ld;
    double dy = lu + 2*u + ru - ld - 2*d - rd;

    if(fabs(dy) < 0.00001 && fabs(dx) < 0.00001)
        return HEURISTIC_UNDEFINED;

    return atan2(dy, dx); //radians
}

// get Methods
double ColorHeuristic::getLimiar()
{
    return limiar;
}
int ColorHeuristic::getColorDifference()
{
    return color_difference;
}

// set methods
void ColorHeuristic::setLimiar(double val)
{
    limiar = val;
}
void ColorHeuristic::setColorDifference(double val)
{
    color_difference = val;
}
void ColorHeuristic::setBaselineColor(int x, int y, Mat *image)
{
    // get color as double values
    baselineColor = getValuefromPixel(x,y,image);
}

void ColorHeuristic::setTestedColor(int x, int y, Mat *image) // receives rgb image
{
    // get color as double values
    testedColor = getValuefromPixel(x,y,image);
//    switch (color_difference) {
//    case INTENSITYC:
//        testedColor=convert.RGB255toGrayscale(testedColor);
//        break;
//    case CIELAB1976:
//    case CIELAB1994:
//    case CIELAB2000:
//    case CIELAB1994MIX:
//    case CIELAB2000MIX:
//    case CMCLAB1984:
//        testedColor=convert.RGB255toCIELAB(testedColor);
//        break;
//    default: // RGB DO NOTHING
//        break;
//    }
}
