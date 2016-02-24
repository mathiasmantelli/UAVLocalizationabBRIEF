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
    //cout << "Testing limiar: " << limiar << "     " << diff << endl;

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

UnscentedColorHeuristic::UnscentedColorHeuristic(STRATEGY s, int id, int cd, double limiar, int uNP):
    ColorHeuristic(s,id,cd,limiar)
{
    numberPoints = uNP;
    baselineColors.resize(numberPoints);
    delta=10;
    deltax = 40;
    deltay = 30;

    points.resize(numberPoints);
    marginUnscented = 10;
}

double UnscentedColorHeuristic::calculateValue(int x, int y, Pose p, cv::Mat *image, cv::Mat *map)
{

    int i,j;
    int auxX, auxY;

    vector<double> vals;

    float theta = p.theta*(-1)-1.5708;

    int xPad = width/2;
    int yPad = height/2;

    cv::Point firstPoint;
    for(int i = 0; i < points.size(); i++){
         firstPoint = cv::Point(((points[i].x-xPad)*cos(theta)+(points[i].y-yPad)*sin(theta))+p.x,
                                         ((points[i].x-xPad)*-sin(theta)+(points[i].y-yPad)*cos(theta))+p.y);
//         cout<<"firstPoint.x:"<<firstPoint.x<<endl;
//         cout<<"firstPoint.y:"<<firstPoint.y<<endl;

        if(firstPoint.x < 0 || firstPoint.x >= image->cols || firstPoint.y < 0 || firstPoint.y >= image->rows){
            vals.push_back(HEURISTIC_UNDEFINED);
        }else{
            vals.push_back(computeDiffColor(firstPoint.x,firstPoint.y,i,image,map));
        }
    }

    double mean=0.0;
    int wrong = 0;
    for(int k=0;k<vals.size();k++)
        if(vals[k]!=HEURISTIC_UNDEFINED)
            mean += vals[k];
        else
            wrong++;
    mean /= (vals.size()-wrong);

//    double prob=1.0;
//    /// Gaussian weighing
//    for(int k=0;k<vals.size();k++)

//    if(diff!=HEURISTIC_UNDEFINED)
//        prob *= 1.0/(sqrt(2*M_PI*9))*exp(-0.5*(pow(vals[k],2)/9));

//    else
//        prob *= 1.0/(numParticles);

    return mean;
}


void UnscentedColorHeuristic::setBaselineColors(int x, int y, cv::Mat *image)
{
    int i,j;
    int auxX, auxY;

    double theta = M_PI/2;

    width = image->cols;
    height = image->rows;

    cv::RNG rng;
    //generating random pairs
    for(int i = 0; i < numberPoints; i++){
        points[i] = cv::Point((int)rng.uniform(marginUnscented, image->cols-marginUnscented-1), (int)rng.uniform(marginUnscented,image->rows-marginUnscented-1));
        baselineColors[i] = getValuefromPixel(points[i].x,points[i].y,image);

    }

    /*
    for(int cx = -1; cx <= 1; cx++){
        for(int cy = -1; cy <= 1; cy++){
            auxX=x+(cx*deltax*3);
            auxY=y+(cy*deltay*3);
            i = auxX;
            j = auxY;
            baselineColors[cont] = getValuefromPixel(i,j,image);
            cont++;
            i=auxX+deltax*cos(theta);
            j=auxY+deltay*sin(theta);
            baselineColors[cont] = getValuefromPixel(i,j,image);
            cont++;
            i=auxX+deltax*cos(theta+M_PI/2);
            j=auxY+deltay*sin(theta+M_PI/2);
            baselineColors[cont] = getValuefromPixel(i,j,image);
            cont++;
            i=auxX+deltax*cos(theta+M_PI);
            j=auxY+deltay*sin(theta+M_PI);
            baselineColors[cont] = getValuefromPixel(i,j,image);
            cont++;
            i=auxX+deltax*cos(theta+3*M_PI/2);
            j=auxY+deltay*sin(theta+3*M_PI/2);
            baselineColors[cont] = getValuefromPixel(i,j,image);
            cont++;
        }
    }

    // get color as double values
    i=x;
    j=y;
    baselineColors[cont] = getValuefromPixel(i,j,image);

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
*/
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

cv::Point UnscentedColorHeuristic::transform_rotation(cv::Point pt, cv::Mat rot, cv::Point trans){
    cv::Mat res(1,2,CV_64F);

    res.at<double>(0,0)=pt.x;
    res.at<double>(0,1)=pt.y;

    cv::Mat dst = res*rot;

    cv::Point point = cv::Point(dst.at<double>(0,0)+trans.x,dst.at<double>(0,1)+trans.y);

    return point;
}
