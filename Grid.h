#ifndef __GRID_H__
#define __GRID_H__

class Cell
{
    public:
        int x,y;        
        bool visited, isObstacle;
        int himm, himm_count, laser_count;
        double distWalls, dirX, dirY;
};

class Grid
{
    public:
        Grid();
        Cell* getCell(int x, int y);

        int getMapScale();
        int getMapWidth();
        int getMapHeight();

        void draw(int xi, int yi, int xf, int yf);

        int numViewModes;
        int viewMode;
        bool showValues;
        bool showArrows;

        int himm_count;

    private:
        int mapScale_; // Number of cells per meter
        int mapWidth_, mapHeight_; // in cells
        int numCellsInRow_, halfNumCellsInRow_;

        Cell* cells_;

        void drawCell(unsigned int i);
        void drawVector(unsigned int i);
        void drawText(unsigned int n);
};

#endif // __GRID_H__
