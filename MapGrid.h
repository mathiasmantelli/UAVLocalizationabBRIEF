#ifndef MAP_GRID_H
#define MAP_GRID_H

class MapGrid;

#include <stdio.h>
#include "Heuristic.h"

using namespace std;

class Heuristic;

enum occupancyValues { CELL_FREE, CELL_OBSTACLE, CELL_UNKNOWN };

class MapGrid {
    public:
        // Start grid from file info, calculating densities using the heuristic.
        MapGrid(Mat *image, Mat *map, Heuristic *heuristic);
        // Start empty grid. 
        MapGrid(int width, int height, double floor, double ceil, Heuristic *heuristic);
        // Start grid from file with density info.
        MapGrid(FILE *mapFile);

       
        int getWidth();
        int getHeight();
        void draw();
        void drawLine();
        
        double getFloorValue();
        double getCeilValue();
        int convertToMapGridDiscrete(double val);

        void setObstacle(bool isObstacle, int x, int y);
        bool isObstacle(int x, int y);
        bool isKnown(int x, int y);
        void clearCell(int x, int y);
    
        int    getHeuristicValue(int x, int y);
        double getPureHeuristicValue(int x, int y);
        double getOrientation(int x, int y);
        bool   isGradientReliable(int x, int y);

        void   calculateHeuristicFor(int x, int y, Mat* image, Mat* map);
        double calculateGradientFor(int x, int y, Mat *image, Mat* map);
        void   smoothMap(); //incomplete function
        double calculateOrientation(int x, int y); //get orientation from precalculated map (true)
        double calculateSobelOrientation(int x, int y); //get orientation from precalculated map (true)
        bool checkGrad(int x, int y);

        Heuristic *heuristic;
    private:
        occupancyValues *cells;
        double *heuristicValues;
        double *gradients;
        bool *reliableGradients;
        int width;
        int height;

        double floorValue;
        double ceilValue;
        double radius;
};

#endif
