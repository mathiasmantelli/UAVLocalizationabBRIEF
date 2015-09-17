#ifndef MIHEURISTIC_H
#define MIHEURISTIC_H

#include "Heuristic.h"
#include "RadiusVolumeTransferFunctions.h"
#include "Utils.h"

typedef vector<unsigned char> ID;

class EntropyHeuristic : public KernelHeuristic
{
public:
    EntropyHeuristic(STRATEGY s, int id, double *kernel, int kW, int kH, int rad, double l=5.0, unsigned int cd=INTENSITYC, unsigned int nbins=8);
    virtual double calculateValue(int x, int y, cv::Mat *image, cv::Mat* map);

    ID getIDfromColor(vec3 color);

protected:
    unsigned int numBinsPerChannel;
    double maxEntropyValue;

};

class MIHeuristic : public EntropyHeuristic
{
public:
    // Constructor
    MIHeuristic(STRATEGY s, int id, double *kernel, int kW, int kH, int rad, double l=5.0, unsigned int cd=INTENSITYC, unsigned int nbins=8);
    double calculateValue(int x, int y, cv::Mat &image, cv::Mat* map, cv::Mat* frame, cv::Mat* frameMap, Pose p);

    //double calculateValue(int x, int y, cv::Mat *image, cv::Mat* map);

    // sets and gets
    void   setObservedEntropy(int x, int y, cv::Mat* image, cv::Mat*map);
    double setCashedEntropy(double value);
    double getObservedEntropy();


private:
    double maxJointEntropy;
    double observedEntropy;
    double cashedEntropy;
};

#endif // MIHEURISTIC_H
