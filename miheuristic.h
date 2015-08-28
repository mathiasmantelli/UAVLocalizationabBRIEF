#ifndef MIHEURISTIC_H
#define MIHEURISTIC_H

#include "Heuristic.h"
#include "RadiusVolumeTransferFunctions.h"

typedef vector<unsigned char> ID;

class EntropyHeuristic : public KernelHeuristic
{
public:
    EntropyHeuristic(STRATEGY s, int id, double *kernel, int kW, int kH, int rad, double l=5.0, unsigned int cd=INTENSITYC, unsigned int nbins=8);
    virtual double calculateValue(int x, int y, Mat *image, Mat* map);

    ID getIDfromColor(vec3 color);

    double calculateEntropy(int x, int y, Mat *image, Mat* map);

protected:
    unsigned int numBinsPerChannel;
    double maxEntropyValue;

};

class MIHeuristic : public EntropyHeuristic
{
public:
    MIHeuristic(STRATEGY s, int id, double *kernel, int kW, int kH, int rad, double l=5.0, unsigned int cd=INTENSITYC, unsigned int nbins=8);
    double calculateValue(int x, int y, Mat *image, Mat* map);

private:

};

#endif // MIHEURISTIC_H
