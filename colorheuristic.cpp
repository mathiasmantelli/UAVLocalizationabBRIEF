#include "colorheuristic.h"

ColorHeuristic::ColorHeuristic(STRATEGY s, int cd, double l):
Heuristic(s,l,cd)
{

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
