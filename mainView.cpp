#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include "MapGrid.h"

int main(int argc, char **argv)
{
    FILE* f = fopen(argv[1],"r");
    MapGrid* map = new MapGrid(f);

    map->drawLine();

    delete map;

    return 0;
}
