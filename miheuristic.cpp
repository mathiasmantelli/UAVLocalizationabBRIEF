#include "miheuristic.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <tuple>
#include "Utils.h"

#include <map>
using namespace std;

MIHeuristic::MIHeuristic(STRATEGY s, int id, double *k, int kW, int kH, int rad, double l, unsigned int cd, unsigned int nbins):
EntropyHeuristic(s,id,k,kW,kH,rad,l,cd,nbins)
{
    if(color_difference == INTENSITYC)
        maxJointEntropy = 2*log2(numBinsPerChannel);
    else
        maxEntropyValue = 6*log2(numBinsPerChannel);

}
HistogramHeuristic::HistogramHeuristic(STRATEGY s, int id, double *k, int kW, int kH, int rad, double l, unsigned int cd, unsigned int nbins):
    EntropyHeuristic(s,id,k,kW,kH,rad,l,cd,nbins)
{
}

double HistogramHeuristic::setObservedHistogram(int x, int y, Mat *frame, Mat *frameMap)
{
    int xini = x-radius;
    int yini = y-radius;
    int xend = x+radius;
    int yend = y+radius;

    if(xini<0 || yini<0 || xend>=frame->cols || yend>=frame->rows)
        return HEURISTIC_UNDEFINED;

    int kpos = 0;
    vec3 free(0.0,0.0,0.0);


    // clear histogram
    frameHistogram.clear();
    std::map<ID,double>::iterator it;


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
            vec3 pos(getValuefromPixel(w,h,frameMap));

            // ignore pixel outside map
            if(pos!=free)
                return HEURISTIC_UNDEFINED;

            vec3 currentColor(getValuefromPixel(w,h,frame));
            ID id =  getIDfromColor(currentColor);

            it = frameHistogram.find(id);
            if (it != frameHistogram.end()){
                frameHistogram[id] += kernel[kpos];
            }else{
                frameHistogram[id] = kernel[kpos];
            }

            kpos++;
        }
    }

    for(it = frameHistogram.begin(); it!= frameHistogram.end(); ++it)
        cout << it->second << " ";
    cout << endl;
}

double HistogramHeuristic::calculateValue(int x, int y, Mat *image, Mat *map)
{
    int xini = x-radius;
    int yini = y-radius;
    int xend = x+radius;
    int yend = y+radius;

    if(xini<0 || yini<0 || xend>=image->cols || yend>=image->rows)
        return HEURISTIC_UNDEFINED;

    int kpos = 0;
    vec3 free(0.0,0.0,0.0);


    // clear histogram
    std::map<ID,double> particleHistogram;
    std::map<ID,double>::iterator it;


    // compute histogram
    for(int w = xini; w<=xend; w++)
    {
        for(int h = yini; h<=yend; h++)
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
            {
                return HEURISTIC_UNDEFINED;
            }

            vec3 currentColor(getValuefromPixel(w,h,image));
            ID id =  getIDfromColor(currentColor);

            it = particleHistogram.find(id);
            if (it != particleHistogram.end()){
                particleHistogram[id] += kernel[kpos];
            }else{
                particleHistogram[id] = kernel[kpos];
            }

            kpos++;
        }
    }

    // Compute
    double bathacharryaCoefficient = 0.0;
    std::map<ID,double>::iterator it2;
    for(it = particleHistogram.begin(); it!= particleHistogram.end(); ++it)
    {
        it2 = frameHistogram.find(it->first);
        if (it2 != frameHistogram.end()){
            bathacharryaCoefficient+=sqrt(it->second*it2->second);
       }
    }
    // hellingerDistance
//    cout << "Batha: " << bathacharryaCoefficient2 << endl;
    return sqrt(1.0-bathacharryaCoefficient*bathacharryaCoefficient);
}

EntropyHeuristic::EntropyHeuristic(STRATEGY s, int id, double *k,int kW, int kH,int rad, double l, unsigned int cd, unsigned int nbins):
KernelHeuristic(s,id,l,cd,rad,k,kW,kH), numBinsPerChannel(nbins)
{
    maxEntropyValue = log2(numBinsPerChannel);
    if(color_difference != INTENSITYC)
        maxEntropyValue = log2(pow(numBinsPerChannel,3));
}

double EntropyHeuristic::calculateValue(int x, int y, Mat *image, Mat* map)
{
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

// Mutual Information Computation
double MIHeuristic::calculateValue(int x, int y, Mat &image, Mat *map, Mat *frame, Mat *frameMap, Pose p)
{
    /// Compute Mutual information
    double mi=0;

    /// Joint distribution of image at position (x,y) and
    /// frame at position (cols/2, rows/2)
    std::map<std::pair<ID,ID>, double> jointPDF;
    std::map<std::pair<ID,ID>, double>::iterator it;

    int size = 2*radius+1+2;
    //cv::Mat subImg = Utils::getRotatedROIFromImage(p,Point2f(size, size), image);
    // The subimage ranges
    int xini = x-radius;
    int yini = y-radius;
    int xend = x+radius;
    int yend = y+radius;

    // There are two extra pixels to facilitate subImage rotation and interpolation
    if(xini-2<0 || yini-2<0 || xend+2>=image.cols || yend+2>=image.rows)
    {
        return HEURISTIC_UNDEFINED;
    }

    Range cRange(xini, xend+1);
    Range rRange(yini, yend+1);
    cv::Mat subImg(image, rRange, cRange);
    cv::Point2f pt( subImg.cols/2.0f, subImg.rows/2.0f);
    cv::Mat r = cv::getRotationMatrix2D(pt, RAD2DEG(p.theta)+90.0, 1.0);
    cv::warpAffine(subImg, subImg, r, cv::Size(size, size));

    // Get new boundaries for the patch extracted from the image
    int xiniROI = subImg.cols/2-radius;
    int yiniROI = subImg.rows/2-radius;
    if(subImg.rows%2!=0)
        yiniROI--;
    if(subImg.cols%2!=0)
        xiniROI--;

    int xiniF = frame->cols/2-radius;
    int yiniF = frame->rows/2-radius;
    int kpos=0;

    // compute histogram
    int wF=xiniF;
    for(int wROI = xiniROI; wROI<size; ++wROI, ++wF)
    {
        int hF = yiniF;
        for(int hROI = yiniROI; hROI<size; ++hROI)
        {
            // Check if there is an UNDEF value
            if(kernel[kpos]==0.0)
            {
                kpos++;
                continue;
            }

            // Construct the n-d ID of the joint distribution
            vec3 imageColor(getValuefromPixel(wROI,hROI,&subImg));
            ID idImage =  getIDfromColor(imageColor);

            vec3 frameColor(getValuefromPixel(wF,hF,frame));
            ID idFrame =  getIDfromColor(frameColor);
            pair<ID,ID> ndID(std::make_pair(idFrame,idImage));

            // Add to joint distribution
            it = jointPDF.find(ndID);
            if (it != jointPDF.end()){
                jointPDF[ndID] += kernel[kpos];
            }else{
                jointPDF[ndID] = kernel[kpos];
            }

            kpos++;
            hF++;
        }
    }

    // Compute joint Entropy
    double jointEntropy = 0.0;
    for(it = jointPDF.begin(); it!= jointPDF.end(); ++it)
    {
        jointEntropy -= it->second * log2(it->second);
    }

    // Denormalize the entropy values and compute mutual information
    // Normalization, trial one
    mi = ((observedEntropy+cashedEntropy)*maxEntropyValue-jointEntropy)/(2*maxEntropyValue);

    //return mutual information;
//    cout << "MI: " << mi << endl;
    return mi;
}

// Frame entropy
void MIHeuristic::setObservedEntropy(int x, int y, Mat* image, Mat*map)
{
    observedEntropy=EntropyHeuristic::calculateValue(x, y, image, map);
}
double MIHeuristic::getObservedEntropy()
{
    return observedEntropy;
}

// Mapgrid entropy
double MIHeuristic::setCashedEntropy(double value)
{
    cashedEntropy=value;
}
