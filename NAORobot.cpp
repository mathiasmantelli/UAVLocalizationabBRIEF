#include "NAORobot.h"

#include <unistd.h>

///////////////////////
///// CONSTRUCTOR /////
///////////////////////

NAORobot::NAORobot() : Robot()
{
    // variables used for navigation
    vX_ = vY_ = vTheta_ = 0.0;

    // sensors variables
    numSonars_ = 2;
    sonars_.resize(numSonars_, 0.0);
    maxSonarRange_ = 2.5;
}

////////////////////////////////////
///// INITIALIZE & RUN METHODS /////
////////////////////////////////////

void NAORobot::initialize(ConnectionMode cmode, LogMode lmode, string fname)
{
    Robot::initialize(cmode,lmode,fname);

    cout << "Initializing NAO!!!" << endl;

    if(logMode_!=PLAYBACK){

        int success=0;

        switch (cmode)
        {
            case WIFI:
            {
                robotIP_="192.168.0.1";
            }
            case SIMULATION:
            {
                robotIP_="127.0.0.1";
            }
        }

        // Initializing proxies
        motionProxy_ = new AL::ALMotionProxy(robotIP_);
        robotPostureProxy_ = new AL::ALRobotPostureProxy(robotIP_);

        motionProxy_->setStiffnesses("Body",1.0);
        robotPostureProxy_->goToPosture("StandInit",1.0);

        initRobotPosition_ = AL::Math::Pose2D(motionProxy_->getRobotPosition(false));
    }

    ready_ = true;
}

void NAORobot::run()
{
    readOdometryAndSensors();

    // Save path traversed by the robot
    if(isMoving() || logMode_==PLAYBACK){
        path_.push_back(odometry_);
    }

    // Mapping

    // Navigation
    switch(motionMode_){
        case ENDING:
            endProcess();
            running_=false;
            break;
        default:
            break;
    }

    resumeMovement();

    sleep(1);
}

//////////////////////////////
///// NAVIGATION METHODS /////
//////////////////////////////

void NAORobot::move(MovingDirection dir)
{
    Robot::move(dir);

    switch(dir){
        case FRONT:
            vX_ = 1.0;
            vY_ = 0.0;
            vTheta_ = 0.0;
            break;
        case BACK:
            vX_ = -1.0;
            vY_ = 0.0;
            vTheta_ = 0.0;
            break;
        case LEFT:
            vX_ = 0.0;
            vY_ = 0.0;
            vTheta_ = M_PI/10.0;
            break;
        case RIGHT:
            vX_ = 0.0;
            vY_ = 0.0;
            vTheta_ = -M_PI/10.0;
            break;
        case STOP:
            vX_ = 0.0;
            vY_ = 0.0;
            vTheta_ = 0.0;
    }
}

bool NAORobot::isMoving()
{
    if(vX_==0.0 && vY_==0.0 && vTheta_==0.0)
        return false;
    return true;
}

void NAORobot::stopMovement()
{
    motionProxy_->moveToward(0.0,0.0,0.0);
}

void NAORobot::resumeMovement()
{
    motionProxy_->moveToward(vX_,vY_,vTheta_);
    //cout << "motionProxy_->moveToward(" << vX_ << ',' << vY_ << ',' << vTheta_ << ")" << endl;
}

void NAORobot::endProcess()
{
    motionProxy_->moveToward(0.0,0.0,0.0);
    robotPostureProxy_->goToPosture("Sit",1.0);
}

///////////////////////////
///// MAPPING METHODS /////
///////////////////////////





////////////////////////////////////////////////////////////////
////// METHODS FOR READING ODOMETRY & SENSORS MEASUREMENTS /////
////////////////////////////////////////////////////////////////

void NAORobot::readOdometryAndSensors()
{
    AL::Math::Pose2D current(motionProxy_->getRobotPosition(false));
    AL::Math::Pose2D displacement = AL::Math::pose2DInverse(initRobotPosition_)*current;

    odometry_.x = displacement.x;
    odometry_.y = displacement.y;
    odometry_.theta = displacement.theta*180.0/M_PI;

    while (odometry_.theta > 180.0)
        odometry_.theta -= 360.0;
    while (odometry_.theta < -180.0)
        odometry_.theta += 360.0;

    cout << "Odometry:" << odometry_ << endl;

//    for(int i=0;i<numSonars_;i++)
//        sonars_[i]=(float)(robot_.getSonarRange(i))/1000.0;
}

int NAORobot::getNumSonars()
{
    return numSonars_;
}

const vector<float>& NAORobot::getSonarReadings()
{
    return sonars_;
}
