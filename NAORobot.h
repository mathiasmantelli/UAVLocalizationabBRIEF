#ifndef NAOROBOT_H
#define NAOROBOT_H

#include "Robot.h"

#include <string>

#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <almath/types/alpose2d.h>

class NAORobot : public Robot
{
    public:
        NAORobot();

        void initialize(ConnectionMode cmode, LogMode lmode, string fname);
        void run();

        // Navigation stuff
        void move(MovingDirection dir);

        // Sensors stuff
        const vector<float>& getSonarReadings();
        int getNumSonars();
        float getMaxSonarRange();

    private:
        // AL stuff
        string robotIP_;
        AL::ALMotionProxy* motionProxy_;
        AL::ALRobotPostureProxy* robotPostureProxy_;
        AL::Math::Pose2D initRobotPosition_;

        // Sensors stuff
        int numSonars_;
        vector<float> sonars_;
        float maxSonarRange_;
        void readOdometryAndSensors();

        // Navigation stuff
        bool isMoving();
        void resumeMovement();
        void stopMovement();
        void endProcess();
        float vX_, vY_, vTheta_;



};

#endif // NAOROBOT_H
