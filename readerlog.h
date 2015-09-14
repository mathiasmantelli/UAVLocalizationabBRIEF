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
        // GyrX, GyrY, GyrZ: the raw gyro rotation rates in degrees/second
        // AccX, AccY, AccZ: the raw accelerometer values in m/s/s
        timeMS, GyrX, GyrY, GyrZ, AccX, AccY, AccZ
    };

    double values[7];
}IMU;

typedef struct ATT{
    enum ValueType
    {
        // DesRoll: the pilot’s desired roll angle in centi-degrees (roll left is negative, right is positive)
        // Roll: the vehicle’s actual roll in centi-degrees (roll left is negative, right is positive)
        // DesPitch: the pilot’s desired pitch angle in centi-degrees (pitch forward is negative, pitch back is positive)
        // Pitch: the vehicle’s actual pitch angle in centi-degrees (roll left is negative, right is positive)
        // DesYaw: the pilot’s desired yaw rate as a number from -4500 ~ 4500 (not in deg/sec, clockwise is positive)
        // Yaw: the vehicles actual heading in centi-degrees with 0 = north
        // NavYaw: the desired heading in centi-degrees
        timeMS, DesRoll, Roll, DesPitch, Pitch, DesYaw, Yaw, ErrRP, ErrYaw
    };

    double values[9];
}ATT;

typedef struct GPS{
    enum ValueType
    {
        //Status – 0 = no GPS, 1 = GPS but no fix, 2 = GPS with 2D fix, 3 = GPS with 3D fix
        //imeMS: the GPS reported time since epoch in milliseconds
        //NSats: the number of satellites current being used
        //HDop: a measure of gps precision (1.5 is good, >2.0 is not so good)
        //Lat: Lattitude according to the GPS
        //Lng: Longitude according to the GPS
        //RelAlt: Accelerometer + Baro altitude in meters
        //Alt: GPS reported altitude (not used by the flight controller)
        //SPD: horizontal ground speed in m/s
        //GCrs: ground course in degrees (0 = north)
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
