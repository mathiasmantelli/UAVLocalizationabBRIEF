#include "miheuristic.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

#include <map>
using namespace std;

MIHeuristic::MIHeuristic(STRATEGY s, double *k, int kW, int kH, int rad, double l, unsigned int cd, unsigned int nbins):
EntropyHeuristic(s, k,kW,kH,rad,l,cd,nbins)
{

}

EntropyHeuristic::EntropyHeuristic(STRATEGY s, double *k,int kW, int kH,int rad, double l, unsigned int cd, unsigned int nbins):
KernelHeuristic(s,l,cd,rad,k,kW,kH), numBinsPerChannel(nbins)
{
    maxEntropyValue = log2(numBinsPerChannel);
    if(color_difference != INTENSITYC)
        maxEntropyValue = log2(pow(numBinsPerChannel,3));
}

double EntropyHeuristic::calculateValue(int x, int y, Mat *image, Mat* map)
{
    // parameters for kernel read
//    int xini = max(x-radius,0);
//    int yini = max(y-radius,0);
//    int xend = min(x+radius,image->cols);
//    int yend = min(y+radius,image->rows);
    int xini = x-radius;
    int yini = y-radius;
    int xend = x+radius;
    int yend = y+radius;

    if(xini<0 || yini<0 || xend>=image->cols || yend>=image->rows)
        return HEURISTIC_UNDEFINED;

    int kpos = 0;
    vec3 free(0.0,0.0,0.0);

    std::map<ID,double> histogram;
    std::map<ID,double>::iterator it;
//    std::map<ID,unsigned int> histogram_int;
//    std::map<ID,unsigned int>::iterator it_int;

//    Mat kernelImage = image->colRange(xini,xend).rowRange(yini,yend);
//    resize(kernelImage,kernelImage,Size(200,200),0,0,INTER_NEAREST);
//    imshow("kernel",kernelImage);
//    waitKey(1);

    // compute histogram
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

            // ignore pixel outside map
            if(pos!=free)
                return HEURISTIC_UNDEFINED;

            vec3 currentColor(getValuefromPixel(w,h,image));
            ID id =  getIDfromColor(currentColor);

            it = histogram.find(id);
            if (it != histogram.end()){
                histogram[id] += kernel[kpos];
//                histogram_int[id] += 1;
            }else{
                histogram[id] = kernel[kpos];
//                histogram_int[id] = 1;
            }

            kpos++;
        }
    }

//    vector<int> histValues;
//    for(it_int = histogram_int.begin(); it_int!= histogram_int.end(); ++it_int)
//        histValues.push_back(it_int->second);
//    int maxValue = *std::min_element(histValues.begin(),histValues.end());

//    int sum_int=0;
//    for(it_int = histogram_int.begin(); it_int!= histogram_int.end(); ++it_int)
//        sum_int += it_int->second;
//    if(fabs(sum-1.0) > 0.0001)
//        cout << "sum Int " << sum_int << endl;

//    double sum=0;
//    for(it = histogram.begin(); it!= histogram.end(); ++it)
//        sum += it->second;
//    if(fabs(sum-1.0) > 0.0001)
//        cout << "sum Double " << sum << endl;

    // Compute entropy
    double entropy = 0.0;
    for(it = histogram.begin(); it!= histogram.end(); ++it)
    {
        entropy -= it->second * log2(it->second);
    }
    entropy /= maxEntropyValue;
//    cout << maxEntropyValue << " numValues " << histValues.size() << " maxValue " << maxValue << " entropy " << entropy << endl;

    return entropy;
}

double MIHeuristic::calculateValue(int x, int y, Mat *image, Mat* map)
{

}

ID EntropyHeuristic::getIDfromColor(vec3 color)
{
    ID id;

    switch(color_difference){
        case INTENSITYC: // 0 -> 255
            id.push_back(color.x*numBinsPerChannel/256);
            break;
        case RGBNORMA: // 0 -> 255 // 0 -> 255 // 0 -> 255
            id.push_back(color.x*numBinsPerChannel/256);
            id.push_back(color.y*numBinsPerChannel/256);
            id.push_back(color.z*numBinsPerChannel/256);
            break;
        case CIELAB1976:
        case CMCLAB1984:
        case CIELAB1994:
        case CIELAB2000:
        case CIELAB2000MIX:
        case CIELAB1994MIX: // 0 -> 100 // -87 -> 99 // -108-> 95
            id.push_back(int(color.x*numBinsPerChannel/100.0));
            id.push_back(int((color.y+87.0)*numBinsPerChannel/(99.0+87.0)));
            id.push_back(int((color.z+108.0)*numBinsPerChannel/(95.0+108.0)));
            break;
    }

    return id;
}

