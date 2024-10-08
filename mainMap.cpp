#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
using std::cout;
using std::endl;
using std::cerr;
#include <cstring>
#include <fstream>
#include <stdexcept>      // std::invalid_argument

#include "SomeKernels.h"
#include "densityheuristic.h"
#include "miheuristic.h"

using namespace cv;

int main( int argc, char** argv )
{

    //data to collect
    Mat image;
    Mat map;
    std::string kernelType;
    std::string outputName;
    std::string diffcolor;
    int radius;
    STRATEGY strategy;
    int color_difference=-1; // -1 to set undefined
    double color_limiar=-1.0;
    int numBins = -1;
    std::vector<bool> validInput(8,false);
    COLOR_NAME colorName;

    int p=1;
    while(p<argc)
    {
        // print help
        if(!strncmp(argv[p], "-h", 2) || !strncmp(argv[p], "-H", 2))
        {
            cout << "Usage: MapfromImage -i <path/image> -o <path/outputFile> -s strategy -r radius -k kernel -d <diff-intensity|diff-rgb|diff-cie1976|diff-cmc1984|diff-cie1994|diff-cie2000|diff-cie1994mix|diff-cie2000mix> -m <path/mapimage>" << endl;
            return 0;
        }
        // Check if this is the filename part -- step 1
        if(!strncmp(argv[p], "-i", 2) || !strncmp(argv[p], "-I", 2))
        {
            // check if there is a file name
            if(argc>p+1)
            {
                // store file name
                std::string imageName(argv[p+1]);
                cout << "Opening image name: " << imageName << endl;
                // extract image
                image = imread( imageName, 1);

                // check if image is valid
                if(!image.data)
                {
                    cerr<<"Image is empty"<< endl;
                    return -1;
                }                

                //increasing the string position
                validInput[0]=true;
                p+=2;
            }
            else
            {
                cerr << "Failed to load image.\nUsage: MapfromImage -i <path/image> -o <path/outputFile> -r radius -k kernel -m <path/mapimage>" << endl;
                return -1;
            }
        }
        // Check if the is the filename for the map is working -- step 2
        else if(!strncmp(argv[p], "-m", 2) || !strncmp(argv[p], "-M", 2))
        {
            // check if there is a file name
            if(argc>p+1)
            {
                // store file name
                std::string imageName(argv[p+1]);
                cout << imageName <<endl;
                // extract image
                map = imread( imageName, 1);

                // check if image is valid
                if(!map.data)
                {
                    cerr<<"Map Image is empty"<< endl;
                    return -1;
                }

                //increasing the string position
                validInput[1]=true;
                p+=2;
            }
            else
            {
                cerr << "Failed to load map image.\nUsage: MapfromImage -i <path/image> -o <path/outputFile> -r radius -k kernel" << endl;
                return -1;
            }
        }
        // Check if the filename for the output file is valid -- step 3
        else if(!strncmp(argv[p], "-o", 2) || !strncmp(argv[p], "-O", 2))
        {
            // check if there is a file name
            if(argc>p+1)
            {
                // store file name
                outputName=argv[p+1];
                validInput[2]=true;
                p+=2;
            }
            else
            {
                cerr << "Failed to find a valid file name for the output map.\nUsage: MapfromImage -i <path/image> -o <path/outputFile> -r radius -k kernel" << endl;
                return -1;
            }
        }
        // Check if this is the kernel type -- step 4
        else if(!strncmp(argv[p], "-k", 2) || !strncmp(argv[p], "-K", 2))
        {
            // check if there is an appropriate kernel type
            if(argc>p+1)
            {
                // store kernel type
                std::string kernel(argv[p+1]);
                if(kernel.compare("Gaussian")==0 || kernel.compare("gaussian")==0 ||
                   kernel.compare("Inverted")==0 || kernel.compare("inverted")==0 ||
                   kernel.compare("Circular")==0 || kernel.compare("circular")==0){
                    // store kernel type
                    kernelType = kernel;
                }
                else
                {
                    cerr << "Invalid kernel type.\nUsage: MapfromImage -k <gaussian,inverted,circular> " << endl;
                    exit(1);
                }
                //increasing the string position
                validInput[3]=true;
                p+=2;
            }
            else
            {
                cerr << "Missing Kernel type:\nUsage: MapfromImage -i <path/image> -o <path/outputFile> -r radius -k kernel -d <diff-intensity|diff-rgb|diff-cie1976|diff-cmc1984|diff-cie1994|diff-cie2000|diff-cie1994mix|diff-cie2000mix> -m <path/mapimage>" << endl;
                return -1;
            }
        }
        // Check if this is the kernel radius -- step 5
        else if(!strncmp(argv[p], "-r", 2) || !strncmp(argv[p], "-R", 2))
        {
            // check if there is an appropriate kernel radius
            if(argc>p+1)
            {
                // store file name
                //std::string r(argv[p+1]);
                try {
                    radius = atoi(argv[p+1]);
                }
                catch (const std::invalid_argument& ia) {
                    cerr << "Invalid argument: " << ia.what() << '\n';
                    cerr << "Last parameter is an int: -k <Gaussian|Inverted|Circular> -r <int>\n";
                    exit(0);
                }

                // Catching silly radius error
                if(radius<=0)
                {
                    cerr << "Missing kernel radius:\nUsage: MapfromImage -i <path/image> -o <path/outputFile> -r radius -k kernel -d <diff-intensity|diff-rgb|diff-cie1976|diff-cmc1984|diff-cie1994|diff-cie2000|diff-cie1994mix|diff-cie2000mix> -m <path/mapimage>" << endl;
                    return -1;
                }

                //increasing the string position
                validInput[4]=true;
                p+=2;
            }
            else
            {
                cerr << "Missing kernel radius:\nUsage: MapfromImage -i <path/image> -o <path/outputFile> -r radius -k kernel" << endl;
                return -1;
            }
        }
        // Check if this is the threshold for color difference is valid -- step 6
        else if(!strncmp(argv[p], "-t", 2) || !strncmp(argv[p], "-T", 2))
        {
            // check if there is an appropriate kernel radius
            if(argc>p+1)
            {
                try {
                    color_limiar = atof(argv[p+1]);
                }
                catch (const std::invalid_argument& ia) {
                    cerr << "Invalid argument: " << ia.what() << '\n';
                    cerr << "Last parameter is a double, example: -t 2.3\n";
                    exit(1);
                }

                // Catching silly color threshold
                if(color_limiar<0)
                {
                    cerr << "Color limiar should not be negative:\nUsage -t <float number>" << endl;
                    return -1;
                }

                numBins = color_limiar;

                //increasing the string position
                validInput[5]=true;
                p+=2;
            }
            else
            {
                cerr << "Missing kernel radius:\nUsage: MapfromImage -i <path/image> -o <path/outputFile> -r radius -k kernel" << endl;
                return -1;
            }
        }
        // Check if this is the color difference is valid -- step 7
        else if(!strncmp(argv[p], "-d", 2) || !strncmp(argv[p], "-D", 2))
        {
            // check if there is an appropriate color difference
            if(argc>p+1)
            {
                // store color difference type
                std::string color_diff(argv[p+1]);
                if(color_diff.compare("diff-intensity")==0 || color_diff.compare("DIFF-INTENSITY")==0)
                    color_difference = INTENSITYC;
                if (color_diff.compare("diff-rgb")==0 || color_diff.compare("DIFF-RGB")==0)
                    color_difference = RGBNORMA;
                if(color_diff.compare("diff-cie1976")==0   || color_diff.compare("DIFF-CIE1976")==0)
                    color_difference = CIELAB1976;
                if(color_diff.compare("diff-cmc1984")==0   || color_diff.compare("DIFF-CMC1984")==0)
                    color_difference = CMCLAB1984;
                if(color_diff.compare("diff-cie1994")==0   || color_diff.compare("DIFF-CIE1994")==0)
                    color_difference = CIELAB1994;
                if(color_diff.compare("diff-cie2000")==0   || color_diff.compare("DIFF-CIE2000")==0)
                    color_difference = CIELAB2000;
                if(color_diff.compare("diff-cie1994mix")==0|| color_diff.compare("DIFF-CIE1994MIX")==0)
                    color_difference = CIELAB1994MIX;
                if(color_diff.compare("diff-cie2000mix")==0|| color_diff.compare("DIFF-CIE2000MIX")==0)
                    color_difference = CIELAB2000MIX;

                //Check for correct initialization
                if(color_difference == -1)
                {
                    cerr << "Invalid color difference.\nUsage: MapfromImage -i <path/image> -o <path/outputFile> -r radius -k kernel -d <diff-intensity|diff-rgb|diff-cie1976|diff-cmc1984|diff-cie1994|diff-cie2000|diff-cie1994mix|diff-cie2000mix> -m <path/mapimage>" << endl;
                    exit(1);
                }
                diffcolor=color_diff;
                //increasing the string position
                validInput[6]=true;
                p+=2;
            }
            else
            {
                cerr << "Missing Kernel type:\nUsage: MapfromImage -i <path/image> -o <path/outputFile> -r radius -k kernel" << endl;
                return -1;
            }
        }
        // Check if this is the strategy type -- step 8
        else if(!strncmp(argv[p], "-s", 2) || !strncmp(argv[p], "-S", 2))
        {
            // check if there is a file name
            if(argc>p+1)
            {
                // store strategy
                std::string s(argv[p+1]);
                if(s.compare("DENSITY")==0 || s.compare("density")==0)
                    strategy=DENSITY;
                else if(s.compare("ENTROPY")==0 || s.compare("entropy")==0)
                    strategy=ENTROPY;
                else if(s.compare("SDENSITY")==0 || s.compare("sdensity")==0)
                    strategy=SINGLE_COLOR_DENSITY;
                else
                     cerr << "Invalid strategy: " << s << endl;

                //increasing the string position
                validInput[7]=true;
                p+=2;

                if(strategy==SINGLE_COLOR_DENSITY){
                    std::string color(argv[p]);
                    if(color.compare("WHITE")==0 || color.compare("white")==0)
                        colorName=WHITE;
                    else if(color.compare("BLACK")==0 || color.compare("black")==0)
                        colorName=BLACK;
                    else if(color.compare("RED")==0 || color.compare("red")==0)
                        colorName=RED;
                    else if(color.compare("BLUE")==0 || color.compare("blue")==0)
                        colorName=BLUE;
                    else if(color.compare("GREEN")==0 || color.compare("green")==0)
                        colorName=GREEN;
                    else
                        cerr << "Invalid color name: " << color << endl;
                }
            }
            else
            {
                cerr << "Missing Strategy type.\nUsage: MapfromImage -i <path/image> -o <path/outputFile> -r radius -k kernel -m <path/mapimage>" << endl;
                return -1;
            }
        }
        else
            p++;
    }

    // Check if all parameters are valid
    for(int i=0;i<validInput.size();++i)
        if(validInput[i]==false)
        {
            cerr << "Input is incorrect at step " << i+1 << endl;
            exit(1);
        }

    double r = double(radius);
    CGaussianC     g;
    CCircular      c;
    CAntiEllipsoid a;

    KernelHeuristic *h;
    std::string kernelT;

    // Try to create the kernel
    if(kernelType.compare("Gaussian")==0 || kernelType.compare("gaussian")==0 )
    {
        kernelT="GAUSSIAN";
        g.initializeKernel(&r);
        if(strategy==DENSITY)
            h = new DensityHeuristic(strategy, 0, g.m_kernelMask, g.width(), g.height(), radius, color_limiar, color_difference);
        else if(strategy==ENTROPY)
            h = new EntropyHeuristic(strategy, 0, g.m_kernelMask, g.width(), g.height(), radius, color_limiar, color_difference, color_limiar); // color_limiar=numBins
        else if(strategy==SINGLE_COLOR_DENSITY)
            h = new SingleColorDensityHeuristic(strategy, 0, g.m_kernelMask, g.width(), g.height(), radius, color_limiar, color_difference, colorName); // color_limiar=numBins
    }
    else if(kernelType.compare("Inverted")==0 || kernelType.compare("inverted")==0)
    {
        kernelT="ANTIELIP";
        a.initializeKernel(&r);
        if(strategy==DENSITY)
            h = new DensityHeuristic(strategy, 0, a.m_kernelMask, a.width(), a.height(), radius, color_limiar, color_difference);
        else if(strategy==ENTROPY)
            h = new EntropyHeuristic(strategy, 0, a.m_kernelMask, a.width(), a.height(), radius, color_limiar, color_difference, color_limiar); // color_limiar=numBins
        else if(strategy==SINGLE_COLOR_DENSITY)
            h = new SingleColorDensityHeuristic(strategy, 0, a.m_kernelMask, a.width(), a.height(), radius, color_limiar, color_difference, colorName); // color_limiar=numBins
    }
    else if(kernelType.compare("Circular")==0 || kernelType.compare("circular")==0)
    {
        kernelT="CIRCULAR";
        c.initializeKernel(&r);
        if(strategy==DENSITY)
            h = new DensityHeuristic(strategy, 0, c.m_kernelMask, c.width(), c.height(), radius, color_limiar, color_difference);
        else if(strategy==ENTROPY)
            h = new EntropyHeuristic(strategy, 0, c.m_kernelMask, c.width(), c.height(), radius, color_limiar, color_difference, color_limiar); // color_limiar=numBins
        else if(strategy==SINGLE_COLOR_DENSITY)
            h = new SingleColorDensityHeuristic(strategy, 0, c.m_kernelMask, c.width(), c.height(), radius, color_limiar, color_difference, colorName); // color_limiar=numBins
    }

    // Convert image to the appropriate format
    Mat used;
    switch(color_difference)
    {
    case INTENSITYC:
        cvtColor(image,used,CV_BGR2GRAY);
        cvtColor(used,used,CV_GRAY2BGR);
        break;
    case CIELAB1976:
    case CIELAB1994:
    case CMCLAB1984:
    case CIELAB2000:
    case CIELAB1994MIX:
    case CIELAB2000MIX:
        image.convertTo(used, CV_32F);
        used*=1/255.0;
        cvtColor(used, used, CV_BGR2Lab);
        break;
    default:
        // RGBNORMA
        used = image.clone();
        break;
    }

    // initialize heuristics vector
    std::vector<double> heuristics_vector(used.rows*used.cols,HEURISTIC_UNDEFINED);
    fstream stream;

    if(strategy==DENSITY){
        cout << "Generating density map file..." << endl;

        cout << "0. Setting the limiar..." << endl;
        cout << "Output name: " << outputName << endl;

        DensityHeuristic* dh = (DensityHeuristic*) h;
    //    if(dh->getColorDifference()==INTENSITYC)
            dh->setLimiarAsMeanDifference(used, map);
        //dh->setLimiar(2.3);
        cout << h->getLimiar() << endl;

        // Create output file
        outputName+=std::string("DENSITY") +
                "_" + colorDifferenceName(color_difference) +
                "_" + kernelT +
                "_R" + std::to_string(int(radius)) +
                "_T" + std::to_string(h->getLimiar()) + ".txt";

    }else if(strategy==ENTROPY){
        cout << "Generating entropy map file..." << endl;

        // Create output file
        outputName+=std::string("ENTROPY") +
                "_" + colorDifferenceName(color_difference) +
                "_" + kernelT +
                "_R" + std::to_string(int(radius)) +
                "_T" + std::to_string(numBins) + ".txt";
    }else if(strategy==SINGLE_COLOR_DENSITY){
        cout << "Generating sdensity map file..." << endl;

        cout << "0. Setting the limiar..." << endl;
        cout << "Output name: " << outputName << endl;

//        DensityHeuristic* dh = (DensityHeuristic*) h;
//    //    if(dh->getColorDifference()==INTENSITYC)
//            dh->setLimiarAsMeanDifference(used, map);
//        //dh->setLimiar(2.3);
//        cout << h->getLimiar() << endl;

        // Create output file
        outputName+=std::string("SDENSITY") +
                "_" + colorDifferenceName(color_difference) +
                "_" + kernelT +
                "_R" + std::to_string(int(radius)) +
                "_T" + std::to_string(h->getLimiar()) + "_C" + getColorName(colorName) + ".txt";
    }

    cout << "1. computing values: this will take a very long time for large kernels..." << endl;

//    resize(used,used,Size(0,0),0.2,0.2);
//    resize(map,map,Size(0,0),0.2,0.2);

    for(int y=h->getRadius();y<used.rows-h->getRadius();++y){
        for(int x=h->getRadius();x<used.cols-h->getRadius();++x)
        {
            // check free region
            vec3 pos(h->getValuefromPixel(x,y,&map));
            vec3 free(0.0, 0.0, 0.0);
            if(pos==free)
                heuristics_vector[y*used.cols+x]=h->calculateValue(x,y,&used,&map);
            else
                heuristics_vector[y*used.cols+x]=HEURISTIC_UNDEFINED;
        }
        if(y%8==0)
            cout << "\r" << y*100/used.rows << "%" << flush;
    }
    cout << "\r100%" << endl;

    if(strategy==DENSITY){
        cout << "2. Storing width, height, kernel type, radius, color-difference, and limiar" << endl;
        stream.open(outputName.data(), std::fstream::out);
        stream << used.cols << " " << used.rows << " " << kernelType << " "
               << h->getRadius() << " " << color_difference << " " << h->getLimiar() << endl;
    }else if(strategy==ENTROPY){
        cout << "2. Storing width, height, kernel type, radius, color-difference, and numBins" << endl;
        stream.open(outputName.data(), std::fstream::out);
        stream << used.cols << " " << used.rows << " " << kernelType << " "
               << h->getRadius() << " " << color_difference << " " << numBins << endl;
    }else if(strategy==SINGLE_COLOR_DENSITY){
        cout << "2. Storing width, height, kernel type, radius, color-difference, limiar, and color" << endl;
        stream.open(outputName.data(), std::fstream::out);
        stream << used.cols << " " << used.rows << " " << kernelType << " "
               << h->getRadius() << " " << color_difference << " " << h->getLimiar() << " " << getColorName(colorName) << endl;
    }

    // Storing data file...
    for(int yI = 0; yI < used.rows; ++yI)
    {
        for(int xI = 0; xI < used.cols; ++xI)
        {
            if(heuristics_vector[yI*used.cols+xI]!=HEURISTIC_UNDEFINED){
                stream << "0/" << heuristics_vector[yI*used.cols+xI] << " ";
            }else
                stream << "-/0.0 ";

            // pro R ler
//            if(heuristics_vector[yI*used.cols+xI]!=HEURISTIC_UNDEFINED)
//                stream << heuristics_vector[yI*used.cols+xI] << " ";
        }
        stream << endl;
    }
    stream.close();

// Stores a gigantic image for some unknown reason
//    // Write to file!CV_WINDOW_AUTOSIZE
    Mat values(used.rows, used.cols, CV_64FC1, &heuristics_vector[0]);
//    //resize(density, density, Size(used.cols, used.rows));

//    // Declare what you need
//    cv::FileStorage file(outputName, cv::FileStorage::WRITE);
//    file << "density" << density;

    Mat window;
    resize(values,window,Size(0,0),0.2,0.2);
    if(strategy==DENSITY){
        namedWindow( "Density Map", CV_WINDOW_KEEPRATIO );
        imshow( "Density Map", window );
    }else if(strategy==ENTROPY){
        namedWindow( "Entropy Map", CV_WINDOW_KEEPRATIO );
        imshow( "Entropy Map", window );
    }else if(strategy==SINGLE_COLOR_DENSITY){
        namedWindow( "Single Color Density Map", CV_WINDOW_KEEPRATIO );
        imshow( "Single Color Density Map", window );
    }
    namedWindow( "Original", CV_WINDOW_KEEPRATIO );
    imshow( "Original", image );
    namedWindow( "Map", CV_WINDOW_KEEPRATIO );
    imshow( "Map", map );

 waitKey(0);

 return 0;
}
