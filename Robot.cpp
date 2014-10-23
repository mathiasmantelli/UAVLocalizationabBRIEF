#include "Robot.h"

#include <GL/glut.h>
#include <cmath>
#include <iostream>

using namespace std;

//////////////////////////////////////
///// CONSTRUCTORS & DESTRUCTORS /////
//////////////////////////////////////

Robot::Robot()
{
    ready = false;
    running = true;
}

Robot::~Robot()
{

}

///////////////////////////
///// VIRTUAL METHODS /////
///////////////////////////

void Robot::initialize(ConnectionMode cmode, LogMode lmode, string fname)
{
    // initialize logfile
    logMode = lmode;
    logFile = new LogFile(logMode,fname);
}

void Robot::run()
{
    cout << "RUNNING BASE ROBOT" << endl;
}

void Robot::move(MovingDirection dir)
{
    switch(dir){
        case FRONT:
            cout << "moving front" << endl;
            break;
        case BACK:
            cout << "moving back" << endl;
            break;
        case LEFT:
            cout << "turning left" << endl;
            break;
        case RIGHT:
            cout << "turning right" << endl;
            break;
        case STOP:
            cout << "stopping robot" << endl;
    }
}

void Robot::draw(double xRobot, double yRobot, double angRobot)
{
    double scale = grid->getMapScale();
    glTranslatef(xRobot,yRobot,0.0);
    glRotatef(angRobot,0.0,0.0,1.0);
    glScalef(1.0/scale,1.0/scale,1.0/scale);
    glColor3f(0.0,0.8,0.0);

    // draw circle
    int num_segments=80;
    float cx=0;
    float cy=0;
    float r=20;

    float theta = 2 * M_PI / float(num_segments);
    float c = cos(theta);//precalculate the sine and cosine
    float s = sin(theta);
    float t;

    float x = r;//we start at angle = 0
    float y = 0;

    glBegin(GL_POLYGON);
    for(int ii = 0; ii < num_segments; ii++)
    {
        glVertex2f(x + cx, y + cy);//output vertex

        //apply the rotation matrix
        t = x;
        x = c * x - s * y;
        y = s * t + c * y;
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

/////////////////////////
///// OTHER METHODS /////
/////////////////////////

bool Robot::isReady()
{
    return ready;
}

bool Robot::isRunning()
{
    return running;
}

const Pose& Robot::getTruePose()
{
    return truePose_;
}

const Pose& Robot::getPoseEstimate()
{
    return poseEstimate_;
}

const Pose& Robot::getOdometry()
{
    return odometry_;
}

void Robot::drawPath()
{
    double scale = grid->getMapScale();

    if(path.size() > 1){
        glScalef(scale,scale,scale);
        glLineWidth(3);
        glBegin( GL_LINE_STRIP );
        {
            for(unsigned int i=0;i<path.size()-1; i++){
                glColor3f(1.0,0.0,1.0);

                glVertex2f(path[i].x, path[i].y);
                glVertex2f(path[i+1].x, path[i+1].y);
            }
        }
        glEnd();
        glLineWidth(1);
        glScalef(1.0/scale,1.0/scale,1.0/scale);

    }
}
