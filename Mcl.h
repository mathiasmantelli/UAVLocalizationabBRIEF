#ifndef MCL_H
#define MCL_H

class MCL;

#include <GL/glew.h>

#include "Robot.h"
#include "MapGrid.h"
#include "GlutClass.h"
#include "vec3.h"

#include <GL/glut.h>
#include <opencv2/core/core.hpp>
using namespace cv;

#include "densityheuristic.h"
#include "colorheuristic.h"
#include "miheuristic.h"

typedef struct{
    Pose p;
    double w;
} MCLparticle;

class MCL
{
    public:
        MCL(vector<Heuristic*>& hVector, vector<MapGrid *> &cMaps, vector<Mat> &gMaps, Pose &initial);
        ~MCL();

        bool run(Pose &u, Mat &z, vector<int> &densities, vector<double> &gradients, double time, Pose &real);
        void writeErrorLogFile(double trueX, double trueY, double trueTh);
        void draw(int x_aux, int y_aux, int halfWindowSize);

        // Required to draw
        vector<MCLparticle> particles;
        vector<Mat>& globalMaps;
        GLuint imageTex;

    private:
        fstream particleLog;
        int numParticles;
        int resamplingThreshold;
        double maxRange;
        double neff;
        Pose lastOdometry;
        STRATEGY locTechnique;
        Pose realPose;
        Pose odomPose;
        vector<Pose> realPath;
        vector<Pose> odomPath;

        vector<Heuristic*>& heuristics;
        vector<MapGrid*>& cachedMaps;

        vector<ColorHeuristic*> ssdHeuristics;
        vector<ColorHeuristic*> colorHeuristics;
        vector<DensityHeuristic*> densityHeuristics;

        void sampling(Pose &u);
        void weighting(Mat& z_robot, vector<int>& densities, Pose &u, vector<double> &gradients);
        void resampling();

        void weightingSSD(Mat& z_robot);
        void weightingDensity(vector<int>& densities, Pose &u, vector<double> &gradients);
        void weightingColor();

        void discardInvalidDeltaAngles(Pose &u, vector<double> &gradients);
        double computeNeff();
        double computeError(double trueX, double trueY,double particleX, double particleY);
        double computeAngleError(double trueTh, double particleTh);
        double sumAngles(double a, double b);

};

#endif // MCL_H
