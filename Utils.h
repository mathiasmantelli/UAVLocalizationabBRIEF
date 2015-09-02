#ifndef UTILS_H
#define UTILS_H

#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <vector>
using namespace std;

#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
using namespace cv;

#define DEG2RAD(a) ((a) * M_PI / 180.0)
#define RAD2DEG(a) ((a) * 180.0 / M_PI)

class Pose{
    public:
        Pose();
        Pose(double a, double b, double c);
        Pose(double a, double b, bool c);

        friend ostream& operator<<(ostream& os, const Pose& p);

        double x, y, theta;
        bool up;
};

enum LogMode { NONE, RECORDING, PLAYBACK};

class LogFile
{
    public:
        LogFile(LogMode mode, string name);

        Pose readPose(string info);
        vector<float> readSensors(string info);

        void writePose(string info, Pose pose);
        void writeSensors(string s, vector<float> sensors);

        bool hasEnded();

    private:
        fstream file;
        string filename;
};


class Timer{
    public:
        Timer();

        void startCounting();
        void startLap();
        void stopCounting();

        float getTotalTime();
        float getLapTime();

    private:
        struct timeval tstart, tlapstart, tnow;
};

class Utils{
public:
    static double getDiffAngle(double ang1, double ang2);
    static Mat getRotatedROIFromImage(Pose p, Size2f s, Mat &largeMap);
    static Mat rotateImage(Mat& input, double angle);
    static double matchImages(Mat& im_1, Mat& im_2, int match_method, InputArray &im2_mask=noArray());
    static Point templateMatching(Mat& image, Mat& templ, Mat& result, int match_method, InputArray &templ_mask=noArray());

};

#endif // UTILS_H
