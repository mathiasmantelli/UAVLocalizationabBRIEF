#include "CorrelativeSM.h"

#include <GL/glut.h>
#include <cmath>
#include <unistd.h>
#include <queue>
using namespace std;

#define USING_LOGLIKE
#define ZERO_ANGLE
#define MULTIRESOLUTION

//////////////////////////////////////////
/// CORRELATIVE SCANMATCHING FUNCTIONS ///
//////////////////////////////////////////

CorrelativeScanMatching::CorrelativeScanMatching(int rows, int cols):
trig(0.001)
{
    // Parameters
    deltaD = sqrt(pow(rows,2)+pow(cols,2))/2; // in meters
    stepD = 5; // in meters
    deltaTh = 90.0; // in degrees
    stepTh = 10; // in degrees
    maxLaserRange = sqrt(pow(rows,2)+pow(cols,2)); // in meters

    windowWidth = windowHeight = int(2*deltaD)+1;
    windowDeltaTh = int(2*deltaTh/stepTh)+1;

    // Matching map
    matchingMap.resize(windowDeltaTh);
    for(int a=0; a<windowDeltaTh; ++a){
        matchingMap[a].resize(windowWidth);
        for(int w=0; w<windowWidth; ++w){
            matchingMap[a][w].resize(windowHeight);
        }
    }

    mapWidth = rows+int(2*maxLaserRange);
    mapHeight = cols+int(2*maxLaserRange);
    centerX = mapWidth/2;
    centerY = mapHeight/2;
    margin = float(mapWidth)*0.05;

    // Occupation map -- 1=obstacle 0=free space
    // Distance map -- euclidean distance (in cells) from nearest obstacles
    // Likelihood map -- lookup-table rasterization made to speed the computation of the probability p(z|x_i, m)
    occupationMap.resize(mapWidth);
    occupationMap2.resize(mapWidth);
    distMap.resize(mapWidth);
    normDistMap.resize(mapWidth);
    likelihoodMap.resize(mapWidth);
    logLikelihoodMap.resize(mapWidth);
    normLikelihoodMap.resize(mapWidth);
    normMatchingMap.resize(mapWidth);
    for(int w=0; w<mapWidth; ++w){
        occupationMap[w].resize(mapHeight,false);
        occupationMap2[w].resize(mapHeight,false);
        distMap[w].resize(mapHeight,INF);
        normDistMap[w].resize(mapHeight,INF);
        likelihoodMap[w].resize(mapHeight,-1.0);
        logLikelihoodMap[w].resize(mapHeight,-1.0);
        normLikelihoodMap[w].resize(mapHeight,-1.0);
        normMatchingMap[w].resize(mapHeight,-1.0);
    }

    // Attributes of likelihood map
    gaussStdev = 1.0;
    gaussVar = square(gaussStdev);
    gaussNu = 1.0/sqrt(2.0*M_PI*gaussVar);
    minValue = 0.0000000001;
    logMinValue = log(minValue);

    trig.checkValues();
}

void CorrelativeScanMatching::compute(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_prev, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cur, Pose& outPose, Matrix3d& outCov)
{

    timer.startCounting();

    // Clear likelihood map
    clearLikelihoodMap();

    Pose orig;

    // Compute distance for nearest obstacle for all cells
    updateDistanceMap(cloud_prev, orig);

    // Build likelihood map
    updateLikelihoodMap();

    // Define boundaries of x, y, and theta to perform search
    updateBoundaries(orig, orig);

    timer.startLap();

    // Compute matching score for all cells inside searching area
    matchingWith2DSlices(cloud_cur,outPose,outCov);
    cout << "matchingWith2DSlices " << timer.getLapTime() << " total " << timer.getTotalTime() << endl;
}

void CorrelativeScanMatching::draw(int t)
{
    // Update window region
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    glOrtho (-margin, mapWidth+margin, -margin, mapHeight+margin, -1, 1);
    glMatrixMode (GL_MODELVIEW);
    glClearColor(0.9, 0.9, 0.9, 0);
    glClear (GL_COLOR_BUFFER_BIT);

    if(t==0){ // Draw occupied

        glColor3f(1.0,1.0,1.0);
        glBegin( GL_QUADS );
        {   // BR - TR - TL - BL
            glVertex2f(mapWidth+0.5, mapHeight+0.5);
            glVertex2f(mapWidth+0.5, -0.5);
            glVertex2f(-0.5, -0.5);
            glVertex2f(-0.5, mapHeight+0.5);
        }
        glEnd();

        glColor3f(0.0,0.0,0.0);
        for(int i=0; i<mapWidth; ++i){
            for(int j=0; j<mapHeight; ++j){
                if(occupationMap[i][j]){
                    glBegin( GL_QUADS );
                    {   // BR - TR - TL - BL
                        glVertex2f(i+0.5, j+0.5);
                        glVertex2f(i+0.5, j-0.5);
                        glVertex2f(i-0.5, j-0.5);
                        glVertex2f(i-0.5, j+0.5);
                    }
                    glEnd();
                }
            }
        }

        glBegin( GL_LINE_STRIP );
        {   // BR - TR - TL - BL
            glVertex2f(minX+windowWidth, minY+windowHeight);
            glVertex2f(minX+windowWidth, minY             );
            glVertex2f(minX            , minY             );
            glVertex2f(minX            , minY+windowHeight);
            glVertex2f(minX+windowWidth, minY+windowHeight);
        }
        glEnd();
    }else if(t==1){ // Draw distance
        for(int i=0; i<mapWidth; ++i){
            for(int j=0; j<mapHeight; ++j){
//                double color = normDistMap[i][j];
//                glColor3f(color,color,color);
                Colors::setHSVColor(normDistMap[i][j]);
//                if(occupationMap[i][j])
//                    glColor3f(0.0,1.0,0.0);
                glBegin( GL_QUADS );
                {   // BR - TR - TL - BL
                    glVertex2f(i+0.5, j+0.5);
                    glVertex2f(i+0.5, j-0.5);
                    glVertex2f(i-0.5, j-0.5);
                    glVertex2f(i-0.5, j+0.5);
                }
                glEnd();
            }
        }
    }else if(t==2){ // Draw cost
        for(int i=0; i<mapWidth; ++i){
            for(int j=0; j<mapHeight; ++j){
//                double color = costMap[i][j];
//                glColor3f(color,color,color);
                Colors::setHSVColor(likelihoodMap[i][j]);
                if(occupationMap[i][j])
                    glColor3f(0.0,1.0,0.0);
                glBegin( GL_QUADS );
                {   // BR - TR - TL - BL
                    glVertex2f(i+0.5, j+0.5);
                    glVertex2f(i+0.5, j-0.5);
                    glVertex2f(i-0.5, j-0.5);
                    glVertex2f(i-0.5, j+0.5);
                }
                glEnd();
            }
        }
    }else if(t==3){ // Draw normalized cost
        for(int i=0; i<mapWidth; ++i){
            for(int j=0; j<mapHeight; ++j){
//                double color = normCostMap[i][j];
//                glColor3f(color,color,color);
                Colors::setHSVColor(normLikelihoodMap[i][j]);
                if(occupationMap[i][j])
                    glColor3f(0.0,1.0,0.0);
                glBegin( GL_QUADS );
                {   // BR - TR - TL - BL
                    glVertex2f(i+0.5, j+0.5);
                    glVertex2f(i+0.5, j-0.5);
                    glVertex2f(i-0.5, j-0.5);
                    glVertex2f(i-0.5, j+0.5);
                }
                glEnd();
            }
        }
    }else if(t==4){ // Draw normalized matching
        for(int i=0; i<mapWidth; ++i){
            for(int j=0; j<mapHeight; ++j){
//                double color = normCostMap[i][j];
//                glColor3f(color,color,color);
                Colors::setHSVColor(normMatchingMap[i][j]);
                if(occupationMap[i][j])
                    glColor3f(0.0,1.0,0.0);
                if(occupationMap2[i][j])
                    glColor3f(1.0,0.0,0.0);
                glBegin( GL_QUADS );
                {   // BR - TR - TL - BL
                    glVertex2f(i+0.5, j+0.5);
                    glVertex2f(i+0.5, j-0.5);
                    glVertex2f(i-0.5, j-0.5);
                    glVertex2f(i-0.5, j+0.5);
                }
                glEnd();
            }
        }
    }
}

/////////////////////////
/// PRIVATE FUNCTIONS ///
/////////////////////////

void CorrelativeScanMatching::clearLikelihoodMap()
{
    for(int i=0; i<mapWidth; ++i){
        for(int j=0; j<mapHeight; ++j){
            occupationMap[i][j] = false;
            occupationMap2[i][j] = false;
            distMap[i][j] = INF;
            normMatchingMap[i][j] = -1.0;
//            normDistMap[i][j] = INF;
//            costMap[i][j] = -1.0;
        }
    }
}

void CorrelativeScanMatching::updateDistanceMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_prev, Pose  pose)
{
    vector<pair<int,int> > cells = transformCloudPoints(cloud_prev,centerX,centerY,pose.theta);

    queue<pair<int,int> > cellsQueue;

    for(int i=0; i<cells.size(); ++i)
    {
        pair<int,int> p = cells[i];

        if(p.first>=0 && p.first<mapWidth && p.second>=0 && p.second<mapHeight){
            occupationMap[p.first][p.second] = true;
            distMap[p.first][p.second] = 0.0;
            cellsQueue.push(p);
        }
    }

//    dt.computeManhattanDT(distMap,cellsQueue);
//    dt.computeChebyshevDT(distMap,cellsQueue);
//    dt.computeEuclideanDT(distMap);
    dt.computeSquareEuclideanDT(distMap);

    // Find maximum value
    double maxDist=-1.0;
    for(int i=0; i<mapWidth; ++i){
        for(int j=0; j<mapHeight; ++j){
            if(distMap[i][j] > maxDist)
                maxDist = distMap[i][j];
        }
    }

    // Normalize distances using the maximum value
    for(int i=0; i<mapWidth; ++i){
        for(int j=0; j<mapHeight; ++j){
            normDistMap[i][j] = (maxDist-distMap[i][j])/maxDist;
        }
    }
}

void CorrelativeScanMatching::updateLikelihoodMap()
{
    // Compute likelihood of all cells
    for(int i=0; i<mapWidth; ++i){
        for(int j=0; j<mapHeight; ++j){
            // prob = 1/sqrt(2*M_PI*sigma^2) * e^(-1/2 * dist^2/sigma^2)
            // prob = nu * e^(-0.5 * sqrdist/var)
            likelihoodMap[i][j] = (1.0-minValue)*gaussNu*exp(-0.5*distMap[i][j]/gaussVar)+minValue;
            #ifdef USING_LOGLIKE
                logLikelihoodMap[i][j] = log(likelihoodMap[i][j]);
            #endif
        }
    }

    // Find maximum value
    double maxDist=-1.0;
    for(int i=0; i<mapWidth; ++i){
        for(int j=0; j<mapHeight; ++j){
            if(likelihoodMap[i][j] > maxDist)
                maxDist = likelihoodMap[i][j];
        }
    }

    // Normalize distances using the maximum value
    for(int i=0; i<mapWidth; ++i){
        for(int j=0; j<mapHeight; ++j){
            normLikelihoodMap[i][j] = (likelihoodMap[i][j])/maxDist;
        }
    }

}

void CorrelativeScanMatching::updateBoundaries(const Pose p1, const Pose p2)
{
    minX = int(-deltaD)+centerX;
    minY = int(-deltaD)+centerY;
    minTh = round(p2.theta - deltaTh);

    while(minTh<=-180.0)
        minTh += 360.0;
    while(minTh>180.0)
        minTh -= 360.0;
}

void CorrelativeScanMatching::updatePoints(vector<pair<int,int> >& points, int w, int h, bool insert)
{
    for(int k=0; k<points.size(); ++k)
    {
        pair<int,int> p = points[k];
        int i = p.first + w;
        int j = p.second + h;
        if(i>=0 && i<mapWidth && j>=0 && j<mapHeight)
            if(insert)
                occupationMap2[i][j] = true;
            else
                occupationMap2[i][j] = false;
    }
}

void CorrelativeScanMatching::matchingWith2DSlices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cur, Pose& pose, Matrix3d& covariance)
{
    #ifdef USING_LOGLIKE
        double maxCost = -INF;
    #else
        double maxCost = -1.0;
    #endif

    int maxA, maxW, maxH;

    Matrix3d K = Matrix3d::Zero();
    Vector3d u = Vector3d::Zero();
    double s = 0;

    Vector3d Best;
    vector<pair<int,int> > cells;
    // For each possible angle
    double angle=minTh;
    for(int a=0; a<windowDeltaTh; ++a){

         cells = transformCloudPoints(cloud_cur,minX,minY,angle);

        for(int w=0; w<windowWidth; ++w){
            for(int h=0; h<windowHeight; ++h){
                #ifdef USING_LOGLIKE
                    double cost=0.0;
                #else
                    double cost=1.0;
                #endif

                if(cells.empty())
                    continue;

                for(int c=0; c<cells.size(); ++c)
                {
                    pair<int,int> p = cells[c];
                    int i = p.first + w;
                    int j = p.second + h;
                    if(i>=0 && i<mapWidth && j>=0 && j<mapHeight){
                        #ifdef USING_LOGLIKE
                            cost += logLikelihoodMap[i][j]; // if using log-likelihood
                        #else
//                            cost *= normLikelihoodMap[i][j];
                            cost *= likelihoodMap[i][j]; // if using likelihood
                        #endif
                    }
                    else{
                        #ifdef USING_LOGLIKE
                            cost += logMinValue;
                        #else
                            cost *= minValue;
                        #endif
                    }
                }
                matchingMap[a][w][h] = cost;

                Vector3d x;
                x[0] = (minX+w-centerX)*stepD;
                x[1] = (minY+h-centerY)*stepD;
                x[2] = minTh+a*stepTh;

                K += x * x.transpose() * cost;
                u += x * cost;
                s += cost;

                if(cost > maxCost){
                    maxCost = cost;
                    Best = x;
                    maxA = a;
                    maxW = w;
                    maxH = h;
                }
            }
        }
        angle += stepTh;
    }

    Best = u/s;

    // Output
//    pose.x = (minX+maxW-centerX)*stepD;
//    pose.y = (minY+maxH-centerY)*stepD;
//    pose.theta = minTh+maxA*stepTh;
    pose.x = Best[0];
    pose.y = Best[1];
    pose.theta = Best[2];

    covariance = (K/s) - (u*u.transpose()/(s*s));

    cout << " X " << pose.x << ' ' << double(minX+maxW-centerX)*stepD
         << " Y " << pose.y << ' ' << double(minY+maxH-centerY)*stepD << endl;

//    vector<pair<int,int> > cells = rayCasting(readings,pose.x/stepD + centerX,pose.y/stepD + centerY,pose.theta);
//    vector<pair<int,int> > cells = rayCasting(readings,minX+maxW,minY+maxH,pose.theta);
    updatePoints(cells,0,0,true);
    usleep(10000);

    #ifdef USING_LOGLIKE
        maxCost = exp(maxCost);
        for(int w=0; w<windowWidth; ++w)
            for(int h=0; h<windowHeight; ++h)
                normMatchingMap[minX+w][minY+h] = exp(matchingMap[maxA][w][h])/maxCost;
    #else
        for(int w=0; w<windowWidth; ++w)
            for(int h=0; h<windowHeight; ++h)
                normMatchingMap[minX+w][minY+h] = matchingMap[maxA][w][h]/maxCost;
    #endif
}



vector<pair<int,int> > CorrelativeScanMatching::transformCloudPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_prev, int xc, int yc, double th)
{
    vector<pair<int,int> > cells;

    // For each laser beam z_t^k
    for(int r=0; r<cloud_prev->points.size(); ++r){

        // if z_t^k != z_max ---- if the laser beam stops
//        if(readings[r] <= maxLaserRange){


            // Find in what position the laser beam stops
            // x_{z_t^k} = x + x_{k,sens}*cos(Th) - y_{k,sens}*sin(Th) + z_t^k*cos(Th+Th_{k,sens})
            // y_{z_t^k} = y + y_{k,sens}*cos(Th) + x_{k,sens}*sin(Th) + z_t^k*sin(Th+Th_{k,sens})
            // that is
            // x = xc + readings[r]*cos(Th+Th_{k,sens})
            // y = yc + readings[r]*sin(Th+Th_{k,sens})

            double x = cloud_prev->points[r].x*trig.degCos(th)-cloud_prev->points[r].y*trig.degSin(th);
            double y = cloud_prev->points[r].x*trig.degSin(th)+cloud_prev->points[r].y*trig.degCos(th);

            int w = xc + x;
            int h = yc + y;
//            occupationMap[w][h] = true;
//            distMap[w][h] = 0.0;

            cells.push_back(pair<int,int>(w,h));
//        }
//        angle -= 1.0;
    }
    return cells;
}


