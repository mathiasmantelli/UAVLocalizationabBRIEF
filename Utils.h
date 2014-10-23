#ifndef UTILS_H
#define UTILS_H

#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <vector>

using namespace std;

class Pose{
    public:
        Pose();
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

#endif // UTILS_H
