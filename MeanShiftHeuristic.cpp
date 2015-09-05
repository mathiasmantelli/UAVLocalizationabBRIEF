#include "MeanShiftHeuristic.h"

MeanShiftHeuristic::MeanShiftHeuristic(STRATEGY s, int id, double *kernel, int kW, int kH, int rad, double l, unsigned int cd):
KernelHeuristic(s,id,l,cd,rad,kernel,kW,kH)
{
    improvedSimilarityMap=false;
}

double MeanShiftHeuristic::calculateValue(int x, int y, Mat *image, Mat *map)
{
    return calculateValue(x,y,map);
}

double MeanShiftHeuristic::calculateValue(int x, int y, Mat* map)
{
    // parameters for kernel read
    int xini = x-radius;
    int yini = y-radius;
    int xend = x+radius;
    int yend = y+radius;

    if(xini<0 || yini<0 || xend>similarityMap.cols || yend>similarityMap.rows)
        return HEURISTIC_UNDEFINED;

    // weighed mask having similar color
    double wmsc = 0.0;
    int kpos = 0;

    // get color as double values
    // compute similarity
    for(int w = xini; w<=xend; ++w)
    {
        for(int h = yini; h<=yend; ++h)
        {

//            //determine if pixel is in free region
//            vec3 pos(getValuefromPixel(w,h,map));
//            vec3 free(0.0,0.0,0.0);

//            // approach an undef region?
//            if(pos!=free)
//                return HEURISTIC_UNDEFINED;

            wmsc += similarityMap.at<float>(h,w)*kernel[kpos];
            kpos++;
        }
    }
//    cout << "WMSC:" << wmsc << endl;

    return wmsc;
}

void MeanShiftHeuristic::updateSimilarityMap(Mat &localImage, Mat &hsvGlobalImage)
{
    // Histogram information
    const int channels[] = { 0, 1 };
    const int histSize[] = { 64, 64 };
    float range[] = { 0, 256 };
    const float *ranges[] = { range, range };

    // Local Image
    Mat hsvLocal, localHistogram;
    // 1 -- Convert to HSV
    cvtColor(localImage, hsvLocal, CV_BGR2HSV);
    // 2 -- Compute Histogram
    Mat localROI = hsvLocal(Rect(Point2f(hsvLocal.cols/2,hsvLocal.rows/2), Size(kWidth, kHeight)));
    calcHist(&localROI, 1, channels, noArray(), localHistogram, 2, histSize, ranges, true, false);

    // Global Image
    if(improvedSimilarityMap){
        Mat globalHistogram;
        // A priori color distribution with cumulative histogram
        calcHist(&hsvGlobalImage, 1, channels, noArray(), globalHistogram, 2, histSize, ranges, true, true);
        // Boosting: Divide conditional probabilities in object area by a priori probabilities of colors
        for (int y = 0; y < localHistogram.rows; y++) {
            for (int x = 0; x < localHistogram.cols; x++) {
                localHistogram.at<float>(y, x) /= globalHistogram.at<float>(y, x);
            }
        }
        cv::normalize(localHistogram, localHistogram, 0, 255, NORM_MINMAX);
    }

    // Compute P(O|B)
    calcBackProject(&hsvGlobalImage, 1, channels, localHistogram, similarityMap, ranges);

    Mat window;
//    normalize(similarityMap,window,0,1,NORM_MINMAX);
    resize(similarityMap,window,Size(0,0),0.2,0.2);
    imshow("BP",window);
    waitKey(0);
}

Pose MeanShiftHeuristic::computeMeanShift(int x, int y)
{
    Rect trackingWindow(x, y, kWidth, kHeight);
    meanShift(similarityMap, trackingWindow, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 0.01));
    return Pose(trackingWindow.x-x,trackingWindow.y-y,0.0);
}
