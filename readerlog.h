#ifndef READERLOG_H
#define READERLOG_H

#include<string>
#include <unistd.h>
#include<iostream>
#include<fstream>
#include<bits/stdc++.h>
#include<stdio.h>
#include<stdlib.h>
#include<cstdlib>
#include<ctype.h>
#include<vector>
#include<sstream>

using namespace std;

typedef struct IMU{
    enum ValueType
    {
        timeMS, GyrX, GyrY, GyrZ, AccX, AccY, AccZ
    };

    double values[7];
}IMU;

typedef struct ATT{
    enum ValueType
    {
        timeMS, DesRoll, Roll, DesPitch, Pitch, DesYaw, Yaw, ErrRP, ErrYaw
    };

    double values[9];
}ATT;

typedef struct GPS{
    enum ValueType
    {
        Status, timeMS, Week, NSats, HDop, Lat, Lng, RelAlt, Alt, Spd, GCrs, Vz, T
    };

    double values[13];
}GPS;

class ReaderLog
{
public:
    ReaderLog(string endereco);
    void Read();

    vector<IMU> get_sensor_IMU();
    vector<IMU> get_sensor_IMU2();
    vector<GPS> get_sensor_GPS();
    vector<ATT> get_sensor_ATT();

private:

    void sensor_IMU(string input_line);
    void sensor_GPS(string input_line);
    void sensor_IMU2(string input_line);
    void sensor_ATT(string input_line);

    ifstream file_input;
    string input_line;
    string buffer;
    vector<IMU> imu_informations;
    vector<IMU> imu2_informations;
    vector<GPS> gps_informations;
    vector<ATT> att_informations;
};

#endif // READERLOG_H
