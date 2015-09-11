#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
using namespace cv;

//#include "MapGrid.h"

#include <fstream>
using namespace std;

int main(int argc, char **argv)
{
//    FILE* f = fopen(argv[1],"r");
//    MapGrid* map = new MapGrid(f);

    fstream input;
    input.open("../Datasets/images.txt",std::fstream::in);
    if(!input.is_open()){
        cout << "File: does not exist" << endl;
        exit(0);
    }

    int initial = 22;
    int final = 26;
    string sufixo = "_"+to_string(initial)+"_"+to_string(final);

    vector<Mat> mapas;
    int i=0;
    while(input.peek() != fstream::traits_type::eof()){
        string imagePath;
        getline(input,imagePath);
        if(i<initial || i>final){
            i++;
            continue;
        }
        cout << "image " << i << " " << imagePath << endl;

        Mat img = imread(imagePath,CV_LOAD_IMAGE_COLOR);
        mapas.push_back(img);
        i++;
    }

//    Mat Average = mapas[0]/mapas.size();
//    for(int m=1; m<mapas.size(); ++m)
//    {
//        Average += mapas[m]/mapas.size();
//    }

    Mat Sum(mapas[0].rows, mapas[0].cols, CV_32FC3, Scalar(0.0, 0.0, 0.0));
    Mat SquaredSum(mapas[0].rows, mapas[0].cols, CV_32FC3, Scalar(0.0, 0.0, 0.0));
    for(int m=0; m<mapas.size(); ++m)
    {
        accumulate(mapas[m],Sum);
        accumulateSquare(mapas[m],SquaredSum);
    }

    Mat Average = Sum/mapas.size();
    Mat Variance(mapas[0].rows, mapas[0].cols, CV_32FC3, Scalar(0.0, 0.0, 0.0));
    cv::pow(SquaredSum/mapas.size() - Average.mul(Average),0.5,Variance);

    Average /= 255.0;
    Variance /= 255.0;

//    Mat Variance(Average.rows, Average.cols, CV_32FC3, Scalar(0.0, 0.0, 0.0));
//    Mat diff(Average.rows, Average.cols, CV_32FC3, Scalar(0.0, 0.0, 0.0));
//    for(int m=1; m<mapas.size(); ++m)
//    {
//        subtract(mapas[m],Average,diff);
////        diff = mapas[m]-Average;
//        imshow("diff",diff);
////        multiply(diff,diff,diff);
//        cv::pow(diff,2.0,diff);
//        Variance += diff;
//    }
//    Variance /= mapas.size();
//    normalize(Variance,Variance);

    Average.convertTo(Average, CV_8UC3, 255.0);
    imwrite("../Datasets/average"+sufixo+".png",Average);
    resize(Average,Average,Size(0,0),0.2,0.2);
    imshow("Average", Average);

    Variance.convertTo(Variance, CV_8UC3, 255.0);
    imwrite("../Datasets/variance"+sufixo+".png",Variance);
    resize(Variance,Variance,Size(0,0),0.2,0.2);
    imshow("Variance", Variance);

//    vector<Mat> spl;
//    split(Variance,spl);
//    imshow("spl1",spl[0]);//b
//    imshow("spl2",spl[1]);//g
//    imshow("spl3",spl[2]);//r

    waitKey(0);
//    map->drawLine();

//    delete map;

    return 0;
}
