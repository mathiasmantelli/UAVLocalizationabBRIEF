#include "colorheuristic.h"

ColorHeuristic::ColorHeuristic(STRATEGY s, int id, int cd, double l):
Heuristic(s,id,l,cd)
{

}

double ColorHeuristic::calculateValue(int x, int y, cv::Mat *image, cv::Mat *map)
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

double ColorHeuristic::calculateValue2(int x, int y, cv::Mat *image, cv::Mat *map)
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
    baselineColor.x = (baselineColor.x+ testedColor.x)/2.0f;
    testedColor.x = baselineColor.x;
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

void ColorHeuristic::setBaselineColor(int x, int y, cv::Mat *image)
{
    // get color as double values
    baselineColor = getValuefromPixel(x,y,image);

    /*(2*getValuefromPixel(x,y,image)
            +getValuefromPixel(x-1,y,image)
            +getValuefromPixel(x+1,y,image)
            +getValuefromPixel(x,y-1,image)
            +getValuefromPixel(x,y+1,image)
            +1/2.0*getValuefromPixel(x+1,y+1,image)
            +1/2.0*getValuefromPixel(x-1,y+1,image)
            +1/2.0*getValuefromPixel(x-1,y-1,image)
            +1/2.0*getValuefromPixel(x+1,y-1,image))/8.0;*/
}

void ColorHeuristic::setTestedColor(int x, int y, cv::Mat *image) // receives rgb image
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

UnscentedColorHeuristic::UnscentedColorHeuristic(STRATEGY s, int id, int cd, double limiar):
    ColorHeuristic(s,id,cd,limiar)
{
    baselineColors.resize(5);
    delta=10;
}

double UnscentedColorHeuristic::calculateValue(int x, int y, Pose p, cv::Mat *image, cv::Mat *map)
{
    int i,j;
    vector<double> vals(5);

    i=x;
    j=y;
    vals[0]=computeDiffColor(i,j,0,image,map);

    i=x+delta*cos(p.theta);
    j=y+delta*sin(p.theta);
    vals[1]=computeDiffColor(i,j,1,image,map);

    i=x+delta*cos(p.theta+M_PI/2);
    j=y+delta*sin(p.theta+M_PI/2);
    vals[2]=computeDiffColor(i,j,2,image,map);

    i=x+delta*cos(p.theta+M_PI);
    j=y+delta*sin(p.theta+M_PI);
    vals[3]=computeDiffColor(i,j,3,image,map);

    i=x+delta*cos(p.theta+3*M_PI/2);
    j=y+delta*sin(p.theta+3*M_PI/2);
    vals[4]=computeDiffColor(i,j,4,image,map);

    double mean=0.0;
    for(int k=0;k<5;k++)
        mean += vals[k];
    mean /= 5;

    return mean;
}

void UnscentedColorHeuristic::setBaselineColors(int x, int y, cv::Mat *image)
{
    int i,j;
    double theta = M_PI/2;

    // get color as double values
    i=x;
    j=y;
    baselineColors[0] = getValuefromPixel(i,j,image);

    i=x+delta*cos(theta);
    j=y+delta*sin(theta);
    baselineColors[1] = getValuefromPixel(i,j,image);

    i=x+delta*cos(theta+M_PI/2);
    j=y+delta*sin(theta+M_PI/2);
    baselineColors[2] = getValuefromPixel(i,j,image);

    i=x+delta*cos(theta+M_PI);
    j=y+delta*sin(theta+M_PI);
    baselineColors[3] = getValuefromPixel(i,j,image);

    i=x+delta*cos(theta+3*M_PI/2);
    j=y+delta*sin(theta+3*M_PI/2);
    baselineColors[4] = getValuefromPixel(i,j,image);

    /*(2*getValuefromPixel(x,y,image)
            +getValuefromPixel(x-1,y,image)
            +getValuefromPixel(x+1,y,image)
            +getValuefromPixel(x,y-1,image)
            +getValuefromPixel(x,y+1,image)
            +1/2.0*getValuefromPixel(x+1,y+1,image)
            +1/2.0*getValuefromPixel(x-1,y+1,image)
            +1/2.0*getValuefromPixel(x-1,y-1,image)
            +1/2.0*getValuefromPixel(x+1,y-1,image))/8.0;*/
}

double UnscentedColorHeuristic::computeDiffColor(int x, int y, int whichPoint, cv::Mat *image, cv::Mat *map)
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
        diff=std::abs(baselineColors[whichPoint].x-testedColor.x);
        break;
    case CIELAB1976:
    case RGBNORMA:
        diff=convert.DeltaECIE1976(baselineColors[whichPoint],testedColor);
        break;
    case CMCLAB1984:
        diff=convert.DeltaECMC1984(baselineColors[whichPoint],testedColor);
        break;
    case CIELAB1994:
        diff=convert.DeltaECIE1994(baselineColors[whichPoint],testedColor);
        break;
    case CIELAB2000:
        diff=convert.DeltaECIE2000(baselineColors[whichPoint],testedColor);
        break;
    case CIELAB2000MIX:
        diff=convert.DeltaEMixCIE2000(baselineColors[whichPoint],testedColor);
        break;
    case CIELAB1994MIX:
        diff=convert.DeltaEMixCIE1994(baselineColors[whichPoint],testedColor);
        break;
    default:
        diff=convert.DeltaECIE1976(baselineColors[whichPoint],testedColor);
    }
    return diff/limiar;
}
