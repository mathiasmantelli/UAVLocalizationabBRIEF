#include "PioneerRobot.h"

#include <GL/glut.h>

//////////////////////////////////////
///// CONSTRUCTORS & DESTRUCTORS /////
//////////////////////////////////////

PioneerRobot::PioneerRobot() : Robot()
{
    // reset robot position in simulator
    resetSimPose_ = true;

    // sensors variables
    numSonars_ = 8;
    sonars_.resize(numSonars_, 0.0);
    numLasers_ = 181;
    lasers_.resize(numLasers_, 0.0);
    maxLaserRange_ = 5.0;

    // variables used for navigation
    avoidDistance_ = 0.4;
    largeDistance_ = 0.5;
    smallDistance_ = 0.4;
    closeToWall_ = false;

    // wheels' velocities
    vLeft_ = vRight_ = 0.0;

    doWallThickening=false;
    minX_=minY_=INT_MAX;
    maxX_=maxY_=INT_MIN;
}

PioneerRobot::~PioneerRobot()
{
    robot_.stopRunning(true);
    robot_.disconnect();
    sick_.lockDevice();
    sick_.stopRunning();
    Aria::exit(0);
    Aria::shutdown();
    if(parser_!=NULL)
        delete parser_;
    if(robotConnector_!=NULL)
        delete robotConnector_;
    if(laserConnector_!=NULL)
        delete laserConnector_;
    if(grid!=NULL)
        delete grid;
}

////////////////////////////////////
///// INITIALIZE & RUN METHODS /////
////////////////////////////////////

void PioneerRobot::initialize(ConnectionMode cmode, LogMode lmode, string fname)
{
    Robot::initialize(cmode,lmode,fname);

    if(logMode_!=PLAYBACK){

        int argc=0; char** argv;

        Aria::init();
        int success=0;

        switch (cmode)
        {
            case SERIAL:
            {
                argc=6;
                argv =(char **) new char*[6];

                argv[0]= new char[4];
                argv[1]= new char[13];
                argv[2]= new char[15];
                argv[3]= new char[7];

                argv[4]= new char[4];
                argv[5]= new char[13];

                strcpy(argv[0],"-rp");
                strcpy(argv[1],"/dev/ttyUSB0");

                strcpy(argv[2],"-laserPortType");
                strcpy(argv[3],"serial");
                strcpy(argv[4],"-lp");
                strcpy(argv[5],"/dev/ttyUSB1");

                parser_= new ArArgumentParser(&argc, argv);
                robotConnector_ = new ArRobotConnector(parser_,&robot_);
                success=robotConnector_->connectRobot();
                if (!success){   Aria::shutdown(); break; }
                robot_.addRangeDevice(&sick_);

                laserConnector_ = new ArLaserConnector(parser_, &robot_, robotConnector_);
                laserConnector_->setupLaser(&sick_);
                break;
            }
            case WIFI:
            {
                argc=4;
                argv =(char **) new char*[4];
                argv[0]= new char[4];
                argv[1]= new char[20];
                argv[2]= new char[20];
                argv[3]= new char[7];

                strcpy(argv[0],"-rh");
                strcpy(argv[1],"192.168.1.11");
                strcpy(argv[2],"-remoteLaserTcpPort");
                strcpy(argv[3],"10002");

                parser_= new ArArgumentParser(&argc, argv);
                robotConnector_ = new ArRobotConnector(parser_,&robot_);
                success=robotConnector_->connectRobot();
                if (!success){   Aria::shutdown(); break; }
                robot_.addRangeDevice(&sick_);

                laserConnector_ = new ArLaserConnector(parser_, &robot_, robotConnector_);
                laserConnector_->setupLaser(&sick_);
                break;
            }
            case SIMULATION:
            {
                argc=2;
                argv =(char **) new char[2];

                argv[0]= new char[4];
                argv[1]= new char[20];

                strcpy(argv[0],"-rh");
                strcpy(argv[1],"localhost");

                parser_= new ArArgumentParser(&argc, argv);

                robotConnector_ = new ArRobotConnector(parser_,&robot_);
                success=robotConnector_->connectRobot();
                if (!success){   Aria::shutdown(); break; }
                robot_.addRangeDevice(&sick_);

                laserConnector_ = new ArLaserConnector(parser_, &robot_, robotConnector_);
                laserConnector_->setupLaser(&sick_);

                if(resetSimPose_){
                    ArRobotPacket pkt;
                    pkt.setID(ArCommands::SIM_RESET);
                    pkt.uByteToBuf(0); // argument type: ignored.
                    pkt.finalizePacket();
                    robot_.getDeviceConnection()->write(pkt.getBuf(), pkt.getLength());
                }

                break;
            }
        }

        if (success){
            robot_.addRangeDevice(&sonarDev_);
            if (success){
                sick_.runAsync();
                robot_.setHeading(0);
                robot_.runAsync(true);
                robot_.enableMotors();
                robot_.setRotVelMax(10);
                printf("Connecting...\n");
                if (!laserConnector_->connectLaser(&sick_)){
                    printf("Could not connect to lasers... exiting\n");
                    exit(0);
                }
            }
            else
                 Aria::shutdown();
        }else{
            printf("Could not connect to robot... exiting\n");
            Aria::shutdown();
            exit(0);
        }
    }

    ready_ = true;
}

void PioneerRobot::run()
{
    if(logMode_==PLAYBACK){
        bool hasEnded = readFromLog();
        if(hasEnded){
            cout << "PROCESS COMPLETE. CLOSING PROGRAM." << endl;
            exit(0);
        }
    }else{
        readOdometryAndSensors();
        if(logMode_==RECORDING)
            writeOnLog();
    }

    // Mapping
    updateGridUsingHIMM();

    // Save path traversed by the robot
    if(isMoving() || logMode_==PLAYBACK){
        path_.push_back(odometry_);
    }

    // Navigation
    switch(motionMode_){
        case WANDER:
            wanderAvoidingCollisions();
            break;
        case FOLLOWWALL:
            wallFollow();
            break;
        default:
            break;
    }

    resumeMovement();

    usleep(50000);
}

void PioneerRobot::draw(double xRobot, double yRobot, double angRobot)
{
    double scale = grid->getMapScale();
    glTranslatef(xRobot,yRobot,0.0);
    glRotatef(angRobot,0.0,0.0,1.0);
    glScalef(1.0/scale,1.0/scale,1.0/scale);
    glColor3f(1.0,0.0,0.0);
    glBegin( GL_POLYGON );
    {
        glVertex2f(-20, -8);
        glVertex2f(-13, -15);
        glVertex2f(8, -15);
        glVertex2f(15, -8);
        glVertex2f(15, 8);
        glVertex2f(8, 15);
        glVertex2f(-13, 15);
        glVertex2f(-20, 8);
    }
    glEnd();
    glColor3f(0.0,0.0,0.0);
    glBegin( GL_LINE_STRIP );
    {
        glVertex2f(0, 0);
        glVertex2f(30, 0);
    }
    glEnd();
    glRotatef(-angRobot,0.0,0.0,1.0);
    glScalef(scale,scale,scale);
    glTranslatef(-xRobot,-yRobot,0.0);
}

//////////////////////////////
///// NAVIGATION METHODS /////
//////////////////////////////

void PioneerRobot::move(MovingDirection dir)
{
    Robot::move(dir);

    switch(dir){
        case FRONT:
            vLeft_ = 300;
            vRight_ = 300;
            break;
        case BACK:
            vLeft_ = -300;
            vRight_ = -300;
            break;
        case LEFT:
            vLeft_ = -60;
            vRight_ = 60;
            break;
        case RIGHT:
            vLeft_ = 60;
            vRight_ = -60;
            break;
        case STOP:
            vLeft_ = 0;
            vRight_ = 0;
    }
}

void PioneerRobot::moveWheels(float lv, float rv)
{
    vLeft_ = lv;
    vRight_ = rv;
}

bool PioneerRobot::isMoving()
{
    if(robot_.getRightVel()!=0.0)
        return true;
    if(robot_.getLeftVel()!=0.0)
        return true;
    return false;
}

void PioneerRobot::stopMovement()
{
    robot_.stop();
}

void PioneerRobot::resumeMovement()
{
    robot_.setVel2(vLeft_, vRight_);
}

void PioneerRobot::wanderAvoidingCollisions()
{
    vector<float> s = getSonarReadings();

    float minLeft  = (s[0]<s[1]?(s[0]<s[2]?s[0]:s[2]):(s[1]<s[2]?s[1]:s[2]));
    float minRight = (s[5]<s[6]?(s[5]<s[7]?s[5]:s[7]):(s[6]<s[7]?s[6]:s[7]));
    float minFront = (s[3]<s[4]?s[3]:s[4]);

    if(minFront > avoidDistance_){
        if(minLeft < avoidDistance_ && minRight < avoidDistance_)
            moveWheels(50,50);
        else if(minLeft < avoidDistance_)
            moveWheels(50,20);
        else if(minRight < avoidDistance_)
            moveWheels(20,50);
        else
            moveWheels(100,100);
    }else{
        if(minLeft < minRight)
            moveWheels(15,-15);
        else
            moveWheels(-15,15);
    }
}

void PioneerRobot::wallFollow()
{
    vector<float> s = getSonarReadings();

    float minLeft=s[0];
    float minRight=s[4];
    for(int i=1;i<4;i++){
        if(s[i] < minLeft)
            minLeft = s[i];
        if(s[i+4] < minRight)
            minRight = s[i+4];
    }

    if(!closeToWall_){
        if(minLeft > largeDistance_ && minRight > largeDistance_)
            moveWheels(100,100);
        else{
            if(minLeft < minRight)
                closeToWall_ = true;
            else
                moveWheels(15,-15);
        }
    }else{
        if(minLeft > largeDistance_)
            moveWheels(50,100);
        else{
            if(minLeft > smallDistance_)
                moveWheels(100,100);
            else
                moveWheels(15,-15);
        }
    }
}

///////////////////////////
///// MAPPING METHODS /////
///////////////////////////

void PioneerRobot::updateGridUsingHIMM()
{

    int offset[][8]={{-1,  1},
                     { 0,  1},
                     { 1,  1},
                     { 1,  0},
                     { 1, -1},
                     { 0, -1},
                     {-1, -1},
                     {-1,  0}};

    //update the grid count for all the cells
    //updated cells will have cell->himm_count!= grid->himm_count
    grid->himm_count++;

    int scale = grid->getMapScale();

    int robotX=odometry_.x*scale;
    int robotY=odometry_.y*scale;
    float robotAngle = odometry_.theta;

    Cell* c=grid->getCell(robotX,robotY);
    c->visited=true;

    int rAngle=(int)robotAngle;
    float angle=90+rAngle;

    for (int s=0; s<numLasers_; s++){
        float range = lasers_[s];
        if(range==0.0)
            continue;

        float difX=cos(angle/180*M_PI);
        float difY=sin(angle/180*M_PI);

        float deltaX, deltaY, dist;

        if(tan(angle/180*M_PI)==1 || tan(angle/180*M_PI)==-1){
            deltaX=deltaY=1;
            if(range>maxLaserRange_)
                dist = difX*maxLaserRange_*scale;
            else
                dist = difX*range*scale;
        }else if(difX*difX > difY*difY){
            deltaX=1;
            deltaY=difY/difX;
            if(range>maxLaserRange_)
                dist = difX*maxLaserRange_*scale;
            else
                dist = difX*range*scale;
        }else{
            deltaX=difX/difY;
            deltaY=1;
            if(range>maxLaserRange_)
                dist = difY*maxLaserRange_*scale;
            else
                dist = difY*range*scale;
        }
        if(deltaX*difX < 0)
            deltaX = -deltaX;
        if(deltaY*difY < 0)
            deltaY = -deltaY;
        if(dist < 0)
            dist = -dist;

        // Filling empty space
        float i=robotX;
        float j=robotY;
        for(float k=0;k<(int)dist;k++){
            i+=deltaX;
            j+=deltaY;

            c=grid->getCell((int)i,(int)j);
            c->visited=true;

            if(c->himm>0){
                // Dot not decrease again if the empty space was visited before
                if(c->himm_count!=grid->himm_count)
                {
                    //decrease the HIMM count
                    c->himm--;
                }
            }

            c->himm_count=grid->himm_count;

            if(c->himm==0){
                c->isObstacle = false;
            }
        }

        // Filling Obstacle space do not fill if range is MAX, because there is no obstacle
        if(range<maxLaserRange_){
            i+=deltaX;
            j+=deltaY;

            c=grid->getCell((int)i,(int)j);
            c->visited=true;

            //Not forcing count here, obstacles have priority
            if((c->himm+4)<16)
                c->himm += 4; //changed from 3 to 4
            else
                c->himm = 16; // changed from 15 to 16

            //Set Cell as WALL
            c->isObstacle = true;

            if(doWallThickening){
                // Marking all the neighbors of a wall as WALL too
                for(int n=0;n<8;n++)
                {
                    Cell* neighbor=grid->getCell((int)i+offset[n][0],(int)j+offset[n][1]);
                    if(neighbor->himm<16)
                        neighbor->himm += 4; //changed from 3 to 4
                    else
                        neighbor->himm = 16; // changed from 15 to 16

                    if(neighbor->himm > 0)
                        neighbor->isObstacle = true;
                    else
                        neighbor->isObstacle = false;
                }
            }
        }

        if(i<minX_)
            minX_=i;
        if(i>maxX_)
            maxX_=i;
        if(j<minY_)
            minY_=j;
        if(j>maxY_)
            maxY_=j;

        angle-=1;
    }    
}

////////////////////////////////////////////////////////////////
////// METHODS FOR READING ODOMETRY & SENSORS MEASUREMENTS /////
////////////////////////////////////////////////////////////////

void PioneerRobot::readOdometryAndSensors()
{
    std::vector < ArSensorReading > *readings;
    std::vector < ArSensorReading > ::iterator it;
    ArPose p;
    sick_.lockDevice();
    readings = sick_.getRawReadingsAsVector();
    it = readings->begin();

    if(!readings->empty()){
        p = (*it).getPoseTaken();
        ready_ = true;
    }
    else{
        sick_.unlockDevice();
        return;
    }

    odometry_.x = p.getX()/1000.0;
    odometry_.y = p.getY()/1000.0;
    odometry_.theta = p.getTh();

    while (odometry_.theta > 180.0)
        odometry_.theta -= 360.0;
    while (odometry_.theta < -180.0)
        odometry_.theta += 360.0;

    int i = 0;
    for (it = readings->begin(); it!=readings->end(); it++){
        lasers_[i++] = (float)(*it).getRange()/1000.0;
    }

    sick_.unlockDevice();

    for(int i=0;i<numSonars_;i++)
        sonars_[i]=(float)(robot_.getSonarRange(i))/1000.0;
}

float PioneerRobot::getMaxLaserRange()
{
    return maxLaserRange_;
}

int PioneerRobot::getNumLasers()
{
    return numLasers_;
}

int PioneerRobot::getNumSonars()
{
    return numSonars_;
}

const vector<float>& PioneerRobot::getLaserReadings()
{
    return lasers_;
}

const vector<float>& PioneerRobot::getSonarReadings()
{
    return sonars_;
}

/////////////////////////////////////////////////////
////// METHODS FOR READING & WRITING ON LOGFILE /////
/////////////////////////////////////////////////////

// Prints to file the data that we would normally be getting from sensors, such as the laser and the odometry.
// This allows us to later play back the exact run.
void PioneerRobot::writeOnLog()
{
    logFile_->writePose("Odometry",odometry_);
    logFile_->writeSensors("Sonar",sonars_);
    logFile_->writeSensors("Laser",lasers_);
}

// Reads back into the sensor data structures the raw readings that were stored to file
// While there is still information in the file, it will return 0. When it reaches the end of the file, it will return 1.
bool PioneerRobot::readFromLog() {

    if(logFile_->hasEnded())
        return true;

    odometry_ = logFile_->readPose("Odometry");
    sonars_ = logFile_->readSensors("Sonar");
    lasers_ = logFile_->readSensors("Laser");

    return false;
}
