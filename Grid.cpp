#include <cstdio>
#include <GL/glut.h>
#include <iostream>
#include <sstream>
#include <iomanip>

using namespace std;

#include "Grid.h"
#include "math.h"

Grid::Grid ()
{
    mapScale_ = 10;
    mapWidth_ = mapHeight_ = 2000;
    numCellsInRow_=mapWidth_;
    halfNumCellsInRow_=mapWidth_/2;

    cells_ = new Cell[mapWidth_*mapHeight_];

    for (unsigned int j = 0; j < numCellsInRow_; ++j)
    {
        for (unsigned int i = 0; i < numCellsInRow_; ++i)
        {
            unsigned int c = j*numCellsInRow_ + i;
            cells_[c].x = -halfNumCellsInRow_ + 1 + i;
            cells_[c].y =  halfNumCellsInRow_ - j;

            cells_[c].visited=false;
            cells_[c].isObstacle=false;

            cells_[c].himm=0;
            cells_[c].laser_count=0;

            cells_[c].distWalls=0.0;
            cells_[c].dirX=0.0;
            cells_[c].dirY=0.0;
        }
    }

    himm_count=0; //keeping the global count of HIMM synchronous

    numViewModes=3;
    viewMode=0;

    showValues=false;
    showArrows=false;
}

Cell* Grid::getCell (int x, int y)
{
    int i=x+halfNumCellsInRow_-1;
    int j=halfNumCellsInRow_-y;
    return &(cells_[j*numCellsInRow_ + i]);
}

int Grid::getMapScale()
{
    return mapScale_;
}

int Grid::getMapWidth()
{
    return mapWidth_;
}

int Grid::getMapHeight()
{
    return mapHeight_;
}

void Grid::draw(int xi, int yi, int xf, int yf)
{
    glLoadIdentity();

    for(int i=xi; i<=xf; ++i){
        for(int j=yi; j<=yf; ++j){
            drawCell(i+j*numCellsInRow_);
        }
    }

    if(showArrows){
        glPointSize(2);
        for(int i=xi; i<=xf; ++i){
            for(int j=yi; j<=yf; ++j){
                drawVector(i+j*numCellsInRow_);
            }
        }
    }

    if(showValues){
        for(int i=xi; i<=xf; i++){
            for(int j=yi; j<=yf; j++){
                drawText(i+j*numCellsInRow_);
            }
        }
    }
}

void Grid::drawCell(unsigned int n)
{
    float aux;

    if(viewMode==0){
        aux=(16.0-cells_[n].himm)/16.0;
        glColor3f(aux,aux,aux);
    }else if(viewMode==1){
        if(cells_[n].isObstacle)
            glColor3f(0.0,0.0,1.0);
        else
            glColor3f(1.0,1.0,1.0);
    }else if(viewMode==2){
        if(cells_[n].himm_count == himm_count)
            glColor3f(0.0,1.0,0.0);
        else
            glColor3f(1.0,1.0,1.0);
    }

    glBegin( GL_QUADS );
    {
        glVertex2f(cells_[n].x+1, cells_[n].y+1);
        glVertex2f(cells_[n].x+1, cells_[n].y  );
        glVertex2f(cells_[n].x  , cells_[n].y  );
        glVertex2f(cells_[n].x  , cells_[n].y+1);
    }
    glEnd();
}

void Grid::drawVector(unsigned int n)
{
    glColor3f(1.0,1.0,0.0);
    glBegin( GL_LINES );
    {
        glVertex2f(cells_[n].x+0.5, cells_[n].y+0.5);
        glVertex2f(cells_[n].x+0.5+cells_[n].dirX, cells_[n].y+0.5+cells_[n].dirY);
    }
    glEnd();
    glBegin( GL_POINTS );
    {
        glVertex2f(cells_[n].x+0.5+cells_[n].dirX, cells_[n].y+0.5+cells_[n].dirY);
    }
    glEnd();
}

void Grid::drawText(unsigned int n)
{
    glRasterPos2f(cells_[n].x+0.25, cells_[n].y+0.25);
    stringstream s;
    glColor3f(0.5f, 0.0f, 0.0f);
    s << cells_[n].distWalls;

    string text=s.str();
    for (unsigned int i=0; i<text.size(); i++)
    {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, text[i]);
    }
}
