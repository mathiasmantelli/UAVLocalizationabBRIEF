#ifndef DISTANCETRANSFORM_H
#define DISTANCETRANSFORM_H

#include <vector>
#include <queue>
using namespace std;

#define INF 1E20

template <class T>
inline T square(const T &x) { return x*x; }

class DistanceTransform
{
public:
    DistanceTransform();

    void computeManhattanDT(vector<vector<double> >& distMap, queue<pair<int, int> > &cellsQueue); // 4-neighbor
    void computeChebyshevDT(vector<vector<double> >& distMap, queue<pair<int, int> > &cellsQueue); // 8-neighbor
    void computeEuclideanDT(vector<vector<double> >& distMap); // euclidean
    void computeSquareEuclideanDT(vector<vector<double> >& distMap); // euclidean

private:
    static double* dt1D(double *f, int n);
};

#endif // DISTANCETRANSFORM_H
