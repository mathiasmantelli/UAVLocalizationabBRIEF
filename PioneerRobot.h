#ifndef __PIONEERROBOT_H__
#define __PIONEERROBOT_H__

#include <Aria.h>
#include "Robot.h"

class PioneerRobot: public Robot {
	public:
        PioneerRobot();
        ~PioneerRobot();

        void initialize(ConnectionMode cmode, LogMode lmode, string fname);
        void run();
        void draw(double xRobot, double yRobot, double angRobot);

        // Navigation stuff
        void move(MovingDirection dir);
        void moveWheels(float vl, float vr);
        bool isMoving();

        // Sensors stuff
        const vector<float>& getLaserReadings();
        const vector<float>& getSonarReadings();
        int getNumLasers();
        int getNumSonars();
        float getMaxLaserRange();

	private:
        // ARIA stuff
        ArRobot robot_;
        ArSonarDevice sonarDev_;
        ArSick sick_;
        ArLaserConnector *laserConnector_;
        ArRobotConnector *robotConnector_;
        ArArgumentParser *parser_;
        bool resetSimPose_;

        // Sensors stuff
        int numSonars_;
        vector<float> sonars_;
        int numLasers_;
        vector<float> lasers_;
        float maxLaserRange_;
        void readOdometryAndSensors();

        void writeOnLog();
        bool readFromLog();

        // Navigation stuff
        double vLeft_, vRight_;
        float avoidDistance_, smallDistance_, largeDistance_;
        bool closeToWall_;
        void stopMovement();
        void resumeMovement();
        void wanderAvoidingCollisions();
        void wallFollow();

        // Mapping stuff
        int minX_, minY_, maxX_, maxY_;
        bool doWallThickening;
        void updateGridUsingHIMM();
};


#endif /* __PIONEERROBOT_H__ */
