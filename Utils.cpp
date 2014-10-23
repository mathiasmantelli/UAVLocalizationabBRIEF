#include "Utils.h"

#include <sstream>
#include <iomanip>
#include <errno.h>
#include <string.h>

/////////////////////////////////
///// METHODS OF CLASS POSE /////
/////////////////////////////////

Pose::Pose(){
    x=y=theta=0.0;
    up=false;
}

Pose::Pose(double a, double b, bool c){
    x=a; y=b; theta=0.0; up=c;
}

ostream& operator<<(ostream& os, const Pose& p)
{
    os << "(" << p.x << ',' << p.y << ',' << p.theta << ")";
    return os;
}

////////////////////////////////////
///// METHODS OF CLASS LOGFILE /////
////////////////////////////////////

LogFile::LogFile(LogMode mode, string name)
{
    time_t t = time(0);
    struct tm *now = localtime(&t);
    stringstream ss;

    if(mode == RECORDING)
    {
        ss << "../PhiR2Framework/Sensors/sensors-" << -100+now->tm_year
                        << setfill('0') << setw(2) << 1+now->tm_mon
                        << setfill('0') << setw(2) << now->tm_mday << '-'
                        << setfill('0') << setw(2) << now->tm_hour
                        << setfill('0') << setw(2) << now->tm_min
                        << setfill('0') << setw(2) << now->tm_sec << ".txt";
        filename = ss.str();

        file.open(filename.c_str(), std::fstream::out);
    }
    else if(mode == PLAYBACK)
    {
        filename = "../PhiR2Framework/Sensors/"+name;
        cout << filename << endl;
        file.open(filename.c_str(), std::fstream::in);
        if(file.fail()){
            cerr << "Error: " << strerror(errno) << endl;
            exit(1);
        }
    }
}

Pose LogFile::readPose(string info)
{
    string tempStr;
    Pose p;

    file >> tempStr >> p.x >> p.y >> p.theta;
    getline(file,tempStr);

    return p;
}

vector<float> LogFile::readSensors(string info)
{
    int max;
    string tempStr;
    vector<float> sensors;

    file >> tempStr >> max;
    sensors.resize(max);
    for (int i = 0; i < max; i++) {
        file >> sensors[i];
    }
    getline(file,tempStr);

    return sensors;
}

void LogFile::writePose(string info, Pose pose)
{
    file << info << ' ' << pose.x << ' ' << pose.y << ' ' << pose.theta << endl;
}

void LogFile::writeSensors(string info, vector<float> sensors)
{
    file << info << ' ' << sensors.size() << ' ';
    for (int i = 0; i < sensors.size(); i++)
        file << sensors[i] << ' ';
    file << endl;
}

bool LogFile::hasEnded()
{
    return file.peek() == fstream::traits_type::eof();
}

//////////////////////////////////
///// METHODS OF CLASS TIMER /////
//////////////////////////////////

Timer::Timer()
{
    startCounting();
}

void Timer::startCounting()
{
    gettimeofday(&tstart, NULL);
    gettimeofday(&tlapstart, NULL);
}

void Timer::startLap()
{
    gettimeofday(&tlapstart, NULL);
}

void Timer::stopCounting()
{
    gettimeofday(&tnow, NULL);
}

float Timer::getTotalTime()
{
    gettimeofday(&tnow, NULL);

    if (tstart.tv_usec > tnow.tv_usec) {
        tnow.tv_usec += 1000000;
        tnow.tv_sec--;
    }

    return (float)(tnow.tv_sec - tstart.tv_sec) +
           ((float)tnow.tv_usec - (float)tstart.tv_usec)/1000000.0;
}

float Timer::getLapTime()
{
    gettimeofday(&tnow, NULL);

    if (tlapstart.tv_usec > tnow.tv_usec) {
        tnow.tv_usec += 1000000;
        tnow.tv_sec--;
    }
    return (float)(tnow.tv_sec - tlapstart.tv_sec) +
           ((float)tnow.tv_usec - (float)tlapstart.tv_usec)/1000000.0;
}


