#include <GL/glut.h>
#include <FreeImage.h>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <cmath>
#include <unistd.h>
using namespace std;

#include "GlutClass.h"

/////////////////////////////////////////////
///// CONSTRUCTOR & CLASS INSTANTIATION /////
/////////////////////////////////////////////

GlutClass::GlutClass(){
}

GlutClass* GlutClass::instance = 0; 

GlutClass* GlutClass::getInstance ()
{
    if (instance == 0){
        instance = new GlutClass;
    }
    return instance;
}

//////////////////////////
///// PUBLIC METHODS /////
//////////////////////////

void GlutClass::initialize()
{
    halfWindowSize = 50;
    x_aux = 0;
    y_aux = 0;
    glutWindowSize = 700;

    // Wait for the robot's initialization
    while(robot_->isReady() == false){
        usleep(100000);
    }

    grid_ = robot_->grid;

	int argc=0;char** argv=0;
    glutInit(&argc, argv);
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize (glutWindowSize,glutWindowSize);

    id_ = glutCreateWindow("Janela");
    lockCameraOnRobot = true;
    drawRobotPath = true;
    frame = 0;

    timer.startCounting();

    glClearColor (1.0, 1.0, 1.0, 0.0);
    glClear (GL_COLOR_BUFFER_BIT);

    glutDisplayFunc(display); 
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKeys);     
}

void GlutClass::process()
{
    glutMainLoop();
}

void GlutClass::terminate()
{
    robot_->motionMode_ = ENDING;
}

void GlutClass::setRobot(Robot *r)
{
    robot_=r;
}

///////////////////////////
///// PRIVATE METHODS /////
///////////////////////////

void GlutClass::render()
{
    if(robot_->isRunning() == false){
        exit(0);
    }

    int mapWidth = grid_->getMapWidth();

    int scale = grid_->getMapScale();

    Pose robotPose;

    robotPose = robot_->getOdometry();

    double xRobot = robotPose.x*scale;
    double yRobot = robotPose.y*scale;
    double angRobot = robotPose.theta;

    double xCenter, yCenter;
    if(lockCameraOnRobot){
        xCenter=xRobot;
        yCenter=yRobot;
    }

    // Update window region
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    glOrtho ((int)(xCenter) + x_aux - halfWindowSize, (int)(xCenter) + x_aux + halfWindowSize-1,
             (int)(yCenter) - y_aux - halfWindowSize, (int)(yCenter) - y_aux + halfWindowSize-1,-1, 50);
    glMatrixMode (GL_MODELVIEW);
    glClearColor(1.0, 1.0, 1.0, 0);
    glClear (GL_COLOR_BUFFER_BIT);

    // Compute limits of visible section of the grid
    int xi, yi, xf, yf;
    int x = xCenter + mapWidth/2 - 1;
    int y = mapWidth/2 - yCenter;

    xi = x + x_aux - halfWindowSize;
    if( xi < 0 ){
        xi = 0;
        xf = halfWindowSize*2 - 1;
    }else{
        xf = x + x_aux + halfWindowSize - 1;
        if( xf > mapWidth - 1){
            xi = mapWidth - 2*halfWindowSize;
            xf = mapWidth - 1;
        }
    }

    yi = y + y_aux - halfWindowSize;
    if( yi < 0 ){
        yi = 0;
        yf = halfWindowSize*2 - 1;
    }else{
        yf = y + y_aux + halfWindowSize - 1;
        if( yf > mapWidth - 1){
            yi = mapWidth - 2*halfWindowSize;
            yf = mapWidth - 1;
        }
    }

    // Draw grid
    grid_->draw(xi, yi, xf, yf);

    // Draw robot path
    if(drawRobotPath){
        robot_->drawPath();
    }

    // Draw robot
    robot_->draw(xRobot,yRobot,angRobot);

    // Take a screenshot per second
    if(timer.getLapTime()>1.0){
        screenshot();
        timer.startLap();
    }

    glutSwapBuffers();
    glutPostRedisplay();

    usleep(5000);
}

void GlutClass::screenshot()
{

    stringstream ss;
    string imgName;
    ss << "../PhiR2Framework/Imgs/frame-" << setfill('0') << setw(6) << frame << ".png";
    ss >> imgName;

    int width = glutWindowSize;
    int height = glutWindowSize;

    // Make the BYTE array, factor of 3 because it's RBG.
    BYTE* pixels = new BYTE[ 3 * width * height];

    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);

    BYTE aux;
    for(int p=0;p<3*width*height;p=p+3){
        aux=pixels[p+2];
        pixels[p+2]=pixels[p];
        pixels[p]=aux;
    }

    // Convert to FreeImage format & save to file
    FIBITMAP* image = FreeImage_ConvertFromRawBits(pixels, width, height, 3 * width, 24, 0xFF0000, 0x0000FF, 0xFF0000, false);
    FreeImage_Save(FIF_PNG, image, imgName.c_str(), 0);

    // Free resources
    FreeImage_Unload(image);
    delete [] pixels;

    frame++;
}

/////////////////////////////////////////////////////
///// STATIC FUNCTIONS PASSED AS GLUT CALLBACKS /////
/////////////////////////////////////////////////////

void GlutClass::display()
{
    instance->render();
}

void GlutClass::reshape(int w, int h)
{
    glViewport (0, 0, (GLsizei) w, (GLsizei) h); 
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    glOrtho (-100,100,-100,100,0, 50);
    glMatrixMode (GL_MODELVIEW);

    glClearColor(0.8, 0.8, 0.8, 0);
    glClear (GL_COLOR_BUFFER_BIT);
}

void GlutClass::keyboard(unsigned char key, int x, int y)
{
    // key: the value of the pressed key
    // x, y: the position of the mouse when the key was pressed

    Pose mousePosition;
    mousePosition.x=x;
    mousePosition.y=y;
    // cout << "Mouse: " << mousePosition.x << ' ' << mousePosition.y << endl;

    switch(key) {
        case 27:
            instance->terminate();
            break;
        case ' ':
            instance->robot_->motionMode_ = MANUAL;
            instance->robot_->move(STOP);
            break;
        case '1':
            instance->robot_->motionMode_ = WANDER;
            break;
        case '2':
            instance->robot_->motionMode_ = FOLLOWWALL;
            break;
        case 'l': //Lock camera
            if(instance->lockCameraOnRobot == true){
                instance->lockCameraOnRobot = false;
                Pose p = instance->robot_->getOdometry();
                instance->x_aux = p.x*instance->grid_->getMapScale();
                instance->y_aux = -p.y*instance->grid_->getMapScale();
            }else{
                instance->lockCameraOnRobot = true;
                instance->x_aux = 0;
                instance->y_aux = 0;
            }
            break;
        case 'f':
            instance->grid_->showArrows=!instance->grid_->showArrows;
            break;
        case 'g':
            instance->grid_->showValues=!instance->grid_->showValues;
            break;
        case 'v': //view mode
            instance->grid_->viewMode++;
            if(instance->grid_->viewMode == instance->grid_->numViewModes)
                instance->grid_->viewMode = 0;
            break;
        case 'b': //view mode
            instance->grid_->viewMode--;
            if(instance->grid_->viewMode == -1)
                instance->grid_->viewMode = instance->grid_->numViewModes-1;
            break;
        case 'w':
            instance->y_aux -= 10;
            cout << "x_aux: " << instance->x_aux << " y_aux: " << instance->y_aux << " halfWindowSize:" << instance->halfWindowSize << endl;
            break;
        case 'd':
            instance->x_aux += 10;
            cout << "x_aux: " << instance->x_aux << " y_aux: " << instance->y_aux << " halfWindowSize:" << instance->halfWindowSize << endl;
            break;
        case 'a':
            instance->x_aux -= 10;
            cout << "x_aux: " << instance->x_aux << " y_aux: " << instance->y_aux << " halfWindowSize:" << instance->halfWindowSize << endl;
            break;
        case 's':
            instance->y_aux += 10;
            cout << "x_aux: " << instance->x_aux << " y_aux: " << instance->y_aux << " halfWindowSize:" << instance->halfWindowSize << endl;
            break;
        case 'm':
            instance->screenshot();
            break;
        case '-':
            instance->halfWindowSize += 10;
            if((unsigned int)instance->halfWindowSize > instance->grid_->getMapWidth()/2)
                instance->halfWindowSize = instance->grid_->getMapWidth()/2;
            break;
        case '+': 
        case '=':
            instance->halfWindowSize -= 10;
            if(instance->halfWindowSize < instance->grid_->getMapScale())
                instance->halfWindowSize = instance->grid_->getMapScale();
            break;
        default:
            break;
    }
}

void GlutClass::specialKeys(int key, int x, int y)
{
    // key: the value of the pressed key
    // x, y: the position of the mouse when the key was pressed

    Pose mousePosition;
    mousePosition.x=x;
    mousePosition.y=y;
    // cout << "Mouse: " << mousePosition.x << ' ' << mousePosition.y << endl;

    switch(key) {
        case GLUT_KEY_UP:
            instance->robot_->move(FRONT);
            break;
        case GLUT_KEY_RIGHT:
            instance->robot_->move(RIGHT);
            break;
        case GLUT_KEY_LEFT:
            instance->robot_->move(LEFT);
            break;
        case GLUT_KEY_DOWN:
            instance->robot_->move(BACK);
            break;
        default:
            break;
    }
}

