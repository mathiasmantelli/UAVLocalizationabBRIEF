#ifndef ROBOT_H
#define ROBOT_H

class Robot;

#include <vector>
using namespace std;

#include "Grid.h"
#include "Utils.h"
#include "Mcl.h"

enum MotionMode {MANUAL, WANDER, FOLLOWWALL, ENDING};
enum MovingDirection {STOP, FRONT, BACK, LEFT, RIGHT};
enum ConnectionMode {SIMULATION, SERIAL, WIFI};

class Robot
{
public:
    Robot();
    ~Robot();

    virtual void initialize(ConnectionMode cmode, LogMode lmode, string fname);
    virtual void run();

    virtual void move(MovingDirection dir);
    virtual void draw(double xRobot, double yRobot, double angRobot);

    const Pose& getTruePose();
    const Pose& getPoseEstimate();
    const Pose& getOdometry();

    void drawPath();

    bool isReady();
    bool isRunning();

    Grid* grid;
    MotionMode motionMode_;

    MCL* mcLocalization;

protected:
    Pose odometry_;
    Pose poseEstimate_;
    Pose truePose_;

    bool ready_;
    bool running_;

    vector<Pose> path_;

    LogFile* logFile_;
    LogMode logMode_;

};

#endif // ROBOT_H
