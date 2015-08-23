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
#include "densityheuristic.h"
#include "colorheuristic.h"


using namespace cv;

typedef struct{
    Pose p;
    double w;
} MCLparticle;

class MCL
{
    public:
        MCL(vector<MapGrid *> &completeDensityMaps, vector<Mat> &gMaps, STRATEGY technique, Pose &initial, vector<ColorHeuristic*>& ssdHeuristics, vector<ColorHeuristic*>& colorHeuristics, vector<DensityHeuristic*>& densityHeuristics);
        ~MCL();

        void draw(int x_aux, int y_aux, int halfWindowSize);

        vector<MCLparticle> particles;

        bool run(Pose &u, Mat &z, vector<int> &densities, vector<double> &gradients, double time, Pose &real);
        void writeErrorLogFile(double trueX, double trueY, double trueTh);
        vector<Mat>& globalMaps;
        GLuint imageTex;

        static Mat getParticleObservation(Pose p, Size2f s, Mat &largeMap);

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


        MapGrid* realMap;
        vector<MapGrid*>& densityMaps;

        vector<ColorHeuristic*> ssdHeuristics;
        vector<ColorHeuristic*> colorHeuristics;
        vector<DensityHeuristic*> densityHeuristics;

        void sampling(Pose &u);

        void weightingSSD(Mat& z_robot);
        void weightingDensity(vector<int>& densities, Pose &u, vector<double> &gradients);
        void weightingColor();

        void resampling();

        void discardInvalidDeltaAngles(Pose &u, vector<double> &gradients);
        double computeNeff();
        double computeError(double trueX, double trueY,double particleX, double particleY);
        double computeAngleError(double trueTh, double particleTh);
        double sumAngles(double a, double b);

        double evaluateParticleUsingSSD(Mat& z_robot, Mat& z_particle);

        // Static stuff used for image matching
        static Mat img;
        static Mat templ;
        static Mat result;
        static int match_method;
        static void MatchingMethod( int, void* );

};

#endif // MCL_H
