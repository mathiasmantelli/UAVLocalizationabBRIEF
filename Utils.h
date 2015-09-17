#ifndef UTILS_H
#define UTILS_H

#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <vector>
using namespace std;

#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
//using namespace cv;
//using cv::cv::Point2f;
//using cv::cv::Mat;
//using cv::Size2f;
//using cv::Size;

#define DEG2RAD(a) ((a) * M_PI / 180.0)
#define RAD2DEG(a) ((a) * 180.0 / M_PI)

class Pose3d
{
public:
    Pose3d();

    double x, y, z;
    double roll, pitch, yaw;
};

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
    static vector<string> getListOfFiles(string dirname);
    static string opencvtype2str(int type);
    static double getNorm(cv::Point2f p);
    static double getDiffAngle(cv::Point2f p1, cv::Point2f p2);
    static double getDiffAngle(double ang1, double ang2);
    static cv::Mat getRotatedROIFromImage(Pose p, cv::Size2f s, cv::Mat &largeMap);
    static cv::Mat rotateImage(cv::Mat& input, double angle);
    static double matchImages(cv::Mat& im_1, cv::Mat& im_2, int match_method, cv::InputArray &im2_mask=cv::noArray());
    static cv::Point templateMatching(cv::Mat& image, cv::Mat& templ, cv::Mat& result, int match_method, cv::InputArray &templ_mask=cv::noArray());

};

#endif // UTILS_H
