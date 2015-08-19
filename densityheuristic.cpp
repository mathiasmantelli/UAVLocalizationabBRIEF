#include "densityheuristic.h"
#include "ColorCPU.h"
#include <iostream>

string colorDifferenceName(int colorDiffID)
{
    switch(colorDiffID) {
    case INTENSITYC:
        return "INTENSITYC";
    case CMCLAB1984:
        return "INTENSITYC";
    case CIELAB1976:
        return"CIELAB1976";
    case CIELAB1994:
        return"CIELAB1994";
    case CIELAB2000:
        return"CIELAB2000";
    case CIELAB2000MIX:
        return"CIELAB2000MIX";
    case CIELAB1994MIX:
        return"CIELAB1994MIX";
    case RGBNORMA:
        return"RGBNORMA";
    default:
        return "ERROR";
    }
}

string strategyName(STRATEGY s)
{
    switch(s)
    {
    case SSD:
        return "SSD";
        break;
    case COLOR_ONLY:
        return "COLOR_ONLY";
        break;
    case DENSITY:
        return "DENSITY";
        break;
    case DENSITY_LOCALCOLORDIFF:
        return "DENSITY_LOCALCOLORDIFF";
        break;
    }
    return "ERROR";
}

string kernelName(int k)
{
    switch(k)
    {
    case 0:
        return "CIRCULAR";
        break;
    case 1:
        return "GAUSSIAN";
        break;
    case 2:
        return "ANTIELIP";
        break;
    }
    return "ERROR";
}

DensityHeuristic::DensityHeuristic(double *k, int kWidth, int kHeight, int radius, double l, unsigned int cd)  :
    radius(radius),
    kernel(NULL),
    kWidth(kWidth),
    kHeight(kHeight),
    limiar(l),
    color_difference(cd),
    convert(),
    color_mean(),
    color_stdev(),
    mean_diff(-1.0) // set as negative --> undefined
{
    this->kernel = new double[kWidth*kHeight];
    memcpy(this->kernel, k, sizeof(double)*kWidth*kHeight);
}

double DensityHeuristic::calculateValue(int x, int y, Mat *image, Mat *map)
{
    // parameters for kernel read
    int xini = x-radius;
    int yini = y-radius;
    int xend = x+radius;
    int yend = y+radius;

    // weighed mask having similar color
    double wmsc = 0.0;
    int kpos = 0;

    // get color as double values
    vec3 center = getValuefromPixel(x,y,image);

    // compute color density estimate
    for(int w = xini; w<=xend; ++w)
    {
        for(int h = yini; h<=yend; ++h)
        {
            // Check if there is an UNDEF value
            if(kernel[kpos]==0.0)
            {
                kpos++;
                continue;
            }

            //determine if pixel is in free region
            vec3 pos(getValuefromPixel(w,h,map));
            vec3 free(0.0,0.0,0.0);

            // approach an undef region?
            if(pos!=free)
                return HEURISTIC_UNDEFINED;

            // Proceed with color difference computation
            double diff;
            vec3 currentColor(getValuefromPixel(w,h,image));
            switch(color_difference){
            case INTENSITYC:
                diff=std::abs(center.x-currentColor.x);
                break;
            case CIELAB1976:
            case RGBNORMA:
                diff=convert.DeltaECIE1976(center,currentColor);
                break;
            case CMCLAB1984:
                diff=convert.DeltaECMC1984(center,currentColor);
                break;
            case CIELAB1994:
                diff=convert.DeltaECIE1994(center,currentColor);
                break;
            case CIELAB2000:
                diff=convert.DeltaECIE2000(center,currentColor);
                break;
            case CIELAB2000MIX:
                diff=convert.DeltaEMixCIE2000(center,currentColor);
                break;
            case CIELAB1994MIX:
                diff=convert.DeltaEMixCIE1994(center,currentColor);
                break;
            default:
                diff=convert.DeltaECIE1976(center,currentColor);
            }
            wmsc += sech5(diff/limiar)*kernel[kpos];
            kpos++;
        }
    }
    return wmsc;
}


double DensityHeuristic::sech5(double val)
{
    return 1/cosh(val*val*val*val*val);
}

vec3 DensityHeuristic::calculateMeanColor(Mat &image, Mat& map)
{
    // reset and recompute mean
    color_mean.set(0.0,0.0,0.0);
    int count = 0;
    for(int x=0;x<image.cols;++x)
        for(int y=0;y<image.rows;++y)
        {
            // check if it is free region
            vec3 pos(getValuefromPixel(x,y,&map));
            vec3 free(0.0,0.0,0.0);

            // approach an undef region?
            if(pos==free)
            {
                // increment mean
                count++;
                color_mean+= getValuefromPixel(x,y,&image);
            }
        }

    // get mean color and return
    color_mean=color_mean/count;
    //cout << "Mean  color: " << color_mean << " count: " << count << " " << image.rows*image.cols << endl;
    return color_mean;
}

// Apply mask
double DensityHeuristic::calculateGradientOrientation(int xCenter, int yCenter, Mat *image, Mat *map)
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

double DensityHeuristic::calculateGradientSobelOrientation(int xCenter, int yCenter, Mat *image, Mat *map)
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


double DensityHeuristic::calculateMeanDifference(Mat &image, Mat& map)
{
    // update color_mean
    calculateMeanColor(image, map);
    int count = 0;
    // compute mean difference
    for(int w = 0; w<image.cols;++w)
        for(int h = 0; h<image.rows;++h)
        {
            // check if it is free region
            vec3 pos(getValuefromPixel(w,h,&map));
            vec3 free(0.0,0.0,0.0);

            // approach an undef region?
            if(pos==free)
            {
                count++;
                vec3 currentColor(getValuefromPixel(w,h,&image));
                switch(color_difference){
                case INTENSITYC:
                    mean_diff+=std::abs(color_mean.x-currentColor.x);
                    break;
                case CIELAB1976:
                    mean_diff+=convert.DeltaECIE1976(color_mean,currentColor);
                    break;
                case CMCLAB1984:
                    mean_diff+=convert.DeltaECMC1984(color_mean,currentColor);
                    break;
                case CIELAB1994:
                    mean_diff+=convert.DeltaECIE1994(color_mean,currentColor);
                    break;
                case CIELAB2000:
                    mean_diff+=convert.DeltaECIE2000(color_mean,currentColor);
                    break;
                case CIELAB2000MIX:
                    mean_diff+=convert.DeltaEMixCIE2000(color_mean,currentColor);
                    break;
                case CIELAB1994MIX:
                    mean_diff+=convert.DeltaEMixCIE1994(color_mean,currentColor);
                    break;
                default:
                    mean_diff+=convert.DeltaECIE1976(color_mean,currentColor);
                }
            }
        }

    // get mean difference and return
    double sum_diff = mean_diff;
    mean_diff /=count;
    cout << "Mean difference: " << mean_diff << " Sum diff: " << sum_diff << " count: " << count << " " << image.rows*image.cols << endl;
    return mean_diff;
}

void DensityHeuristic::setLimiarAsMeanDifference(Mat &image, Mat &map)
{
    if(mean_diff<0.0)
        calculateMeanDifference(image, map);
    limiar = mean_diff;
}

vec3 DensityHeuristic::getValuefromPixel(int x, int y, Mat *image)
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
double DensityHeuristic::getRadius()
{
    return radius;
}

double DensityHeuristic::getLimiar()
{
    return limiar;
}

void DensityHeuristic::setLimiar(double val)
{
    if(val<0)
    {
        // error value is stupid, setting default
        cerr << val << " is not valid, setting threshold to standard value" << endl;
        limiar = 5.0;
    }
    else
        limiar = val;
}
