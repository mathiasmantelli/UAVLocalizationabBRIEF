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
//using namespace cv;

#include "densityheuristic.h"
#include "colorheuristic.h"
#include "miheuristic.h"
#include "BriefHeuristic.h"

#include <omp.h>

typedef struct{
    Pose p;
    double w;
} MCLparticle;

class MCL
{
    public:
        MCL(vector<Heuristic*>& hVector, vector<MapGrid *> &cMaps, vector<cv::Mat> &gMaps, Pose &initial, string &lName);
        ~MCL();

        bool run(Pose &u, bool is_u_reliable, cv::Mat &z, double time, Pose &real);
        bool initialRun(Pose &u, bool is_u_reliable, cv::Mat &z, double time, Pose &real, double lastTotalElapsed);
        void writeErrorLogFile(double trueX, double trueY, double trueTh);
        void draw(int x_aux, int y_aux, int halfWindowSize);
        void restart(Pose &initial, string &lName);

        // Required to draw
        vector<MCLparticle> particles;
        vector<cv::Mat>& globalMaps;
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
        vector<double> heuristicValues;
        vector<double> heuristicGradients;
        vector<cv::Mat> frameColorConverted;
        cv::Mat binaryFrameMask;

        bool removeDuplicates;
        bool starting;

        Timer timer;

        void sampling(Pose &u, bool reliable);
        void weighting(cv::Mat& z_robot, Pose &u);
        void prepareWeighting(cv::Mat &z);
        void resampling();

        void weightingSSD(cv::Mat& z_robot);
        void weightingDensity(vector<int>& densities, Pose &u, vector<double> &gradients);
        void weightingColor();

        void discardInvalidDeltaAngles(Pose &u, vector<double> &gradients);
        double computeNeff();
        double computeError(double trueX, double trueY,double particleX, double particleY);
        double computeAngleError(double trueTh, double particleTh);
        double sumAngles(double a, double b);

        void createColorVersions(cv::Mat& imageRGB);

};

#endif // MCL_H
