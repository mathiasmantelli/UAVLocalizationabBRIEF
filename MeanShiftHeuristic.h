#ifndef MEANSHIFTHEURISTIC_H
#define MEANSHIFTHEURISTIC_H

#include "Heuristic.h"
#include "Utils.h"

class MeanShiftHeuristic : public KernelHeuristic
{
public:
    MeanShiftHeuristic(STRATEGY s, int id, double *kernel, int kW, int kH, int rad, double l=5.0, unsigned int cd=INTENSITYC);
    double calculateValue(int x, int y, cv::Mat *image, cv::Mat* map);
    double calculateValue(int x, int y, cv::Mat *map=NULL);

    void updateSimilarityMap(cv::Mat &localImage, cv::Mat &globalImage);
    Pose computeMeanShift(int x, int y);

private:
    bool improvedSimilarityMap;
    cv::Mat similarityMap;
};

#endif // MEANSHIFTHEURISTIC_H
