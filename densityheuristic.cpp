#include "densityheuristic.h"
#include "ColorCPU.h"
#include <iostream>

std::string colorDifferenceName(int colorDiffID)
{
    switch(colorDiffID) {
    case INTENSITYC:
        return "INTENSITYC";
    case CMCLAB1984:
        return "CMC1984";
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
std::string kernelName(int k)
{
    switch(k)
    {
    case KCIRCULAR:
        return "CIRCULAR";
        break;
    case KGAUSSIAN:
        return "GAUSSIAN";
        break;
    case KANTIELIP:
        return "ANTIELIP";
        break;
    }
    return "ERROR";
}
std::string strategyName(STRATEGY s)
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
    case ENTROPY:
        return "ENTROPY";
        break;
    case SINGLE_COLOR_DENSITY:
        return "SDENSITY";
        break;
    case DENSITY_LOCALCOLORDIFF:
        return "DENSITY_LOCALCOLORDIFF";
        break;
    }
    return "ERROR";
}

std::string getColorName(COLOR_NAME cn)
{
    switch(cn)
    {
    case WHITE:
        return "WHITE";
        break;
    case BLACK:
        return "BLACK";
        break;
    case RED:
        return "RED";
        break;
    case GREEN:
        return "GREEN";
        break;
    case BLUE:
        return "BLUE";
        break;
    }
    return "ERROR";
}

DensityHeuristic::DensityHeuristic(STRATEGY s, int id, double *k, int kW, int kH, int rad, double l, unsigned int cd):
KernelHeuristic(s,id,l,cd,rad,k,kW,kH)
{
    color_mean.set(0.0,0.0,0.0);
    color_stdev.set(0.0,0.0,0.0);
    mean_diff = -1.0; // set as negative --> undefined
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

double DensityHeuristic::calculateMeanDifference(Mat &image, Mat& map)
{
    // update color_mean    int getColorDifference();

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

SingleColorDensityHeuristic::SingleColorDensityHeuristic(STRATEGY s, int id, double *k, int kW, int kH, int rad, double l, unsigned int cd, COLOR_NAME colorName):
DensityHeuristic(s,id,k,kW,kH,rad,l,cd)
{
    double min=0;
    double max=255;//255;

    // colors in RGB
    switch(colorName)
    {
        case WHITE:
            color = vec3(max,max,max);
            break;
        case BLACK:
            color = vec3(min,min,min);
            break;
        case RED:
            color = vec3(max,min,min);
            break;
        case GREEN:
            color = vec3(min,max,min);
            break;
        case BLUE:
            color = vec3(min,min,max);
            break;
        default:
            color = vec3(min,min,min);
    }

    if(color_difference == RGBNORMA){ // convert to bgr
        vec3 tmp = color;
        color.x = tmp.z;
        color.y = tmp.y;
        color.z = tmp.x;
        limiar = sqrt(pow(255.0,2)*3);
    }else if(color_difference == CIELAB1976){
        color = convert.RGB255toCIELAB(color);
        limiar = sqrt(pow(100.0,2)+pow(99.0+87.0,2)+pow(95.0+108.0,2))/2;
    }

}

double SingleColorDensityHeuristic::calculateValue(int x, int y, Mat *image, Mat *map)
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
    vec3 center = color;
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
//            cout << "color:" << color << " pixel:" << currentColor << " diff:" << diff << endl;
//            wmsc += (diff/limiar<0.45?diff/limiar:0)*kernel[kpos];
//            wmsc += diff/limiar*kernel[kpos];
            wmsc += sech5(diff/limiar)*kernel[kpos];
            kpos++;
        }
    }
//    cout << "WMSC:" << wmsc << endl;
//    if(wmsc>0.1)
//        cout << wmsc << endl;
    return wmsc;
}
