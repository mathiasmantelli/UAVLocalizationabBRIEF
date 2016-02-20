#include "DistanceTransform.h"

#include <cmath>

DistanceTransform::DistanceTransform()
{

}

int offset4N[][4]={{ 0,  1},{ 1,  0},{ 0, -1},{-1,  0}};
int offset8N[][8]={{-1,  1},{ 0,  1},{ 1,  1},{ 1,  0},{ 1, -1},{ 0, -1},{-1, -1},{-1,  0}};

void DistanceTransform::computeManhattanDT(vector<vector<double> > &distMap, queue<pair<int,int> >& cellsQueue)
{
    int width = distMap.size();
    int height = distMap[0].size();

    // Compute manhattan (4-neighbor) distance to obstacles
    while(!cellsQueue.empty())
    {
        pair<int,int> p = cellsQueue.front();
        cellsQueue.pop();
        double curDist = distMap[p.first][p.second];

        // Add unvisited neighbors
        for(int n=0;n<4;++n)
        {
            int nx = p.first+offset4N[n][0];
            int ny = p.second+offset4N[n][1];
            if(nx < 0 || ny < 0 || nx >= width || ny >= height)
                continue;

            if(distMap[nx][ny] == INF){
                distMap[nx][ny] = curDist+1.0;
                cellsQueue.push(pair<int,int>(nx,ny));
            }
        }
    }
}

void DistanceTransform::computeChebyshevDT(vector<vector<double> > &distMap, queue<pair<int,int> >& cellsQueue)
{
    int width = distMap.size();
    int height = distMap[0].size();

    // Compute chebyshev (8-neighbor) distance to obstacles
    while(!cellsQueue.empty())
    {
        pair<int,int> p = cellsQueue.front();
        cellsQueue.pop();
        double curDist = distMap[p.first][p.second];

        // Add unvisited neighbors
        for(int n=0;n<8;++n)
        {
            int nx = p.first+offset8N[n][0];
            int ny = p.second+offset8N[n][1];
            if(nx < 0 || ny < 0 || nx >= width || ny >= height)
                continue;

            if(distMap[nx][ny] == INF){
                distMap[nx][ny] = curDist+1.0;
                cellsQueue.push(pair<int,int>(nx,ny));
            }
        }
    }

}

void DistanceTransform::computeEuclideanDT(vector<vector<double> > &distMap)
{
    int width = distMap.size();
    int height = distMap[0].size();

    computeSquareEuclideanDT(distMap);

    // Compute square root of all cells
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            distMap[x][y] = sqrt(distMap[x][y]);
        }
    }
}

void DistanceTransform::computeSquareEuclideanDT(vector<vector<double> > &distMap)
{
    int width = distMap.size();
    int height = distMap[0].size();

    double *f = new double[std::max(width,height)];

    // Distance Transform of 2d function using squared euclidean distance

    // Transform along columns
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            f[y] = distMap[x][y];
        }
        double *d = dt1D(f, height);
        for (int y = 0; y < height; y++) {
            distMap[x][y] = d[y];
        }
        delete [] d;
    }

    // Transform along rows
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            f[x] = distMap[x][y];
        }
        double *d = dt1D(f, width);
        for (int x = 0; x < width; x++) {
            distMap[x][y] = d[x];
        }
        delete [] d;
    }

    delete f;
}

/* dt of 1d function using squared distance */
double* DistanceTransform::dt1D(double *f, int n)
{
    double *d = new double[n];
    int *v = new int[n];
    double *z = new double[n+1];
    int k = 0;
    v[0] = 0;
    z[0] = -INF;
    z[1] = +INF;
    for (int q = 1; q <= n-1; q++) {
        double s  = ((f[q]+square(q))-(f[v[k]]+square(v[k])))/(2*q-2*v[k]);
        while (s <= z[k]) {
            k--;
            s  = ((f[q]+square(q))-(f[v[k]]+square(v[k])))/(2*q-2*v[k]);
        }
        k++;
        v[k] = q;
        z[k] = s;
        z[k+1] = +INF;
    }

    k = 0;
    for (int q = 0; q <= n-1; q++) {
        while (z[k+1] < q)
            k++;
        d[q] = square(q-v[k]) + f[v[k]];
    }

    delete [] v;
    delete [] z;
    return d;
}
