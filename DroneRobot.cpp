#include "DroneRobot.h"

#include "SomeKernels.h"

#include <unistd.h>
#include <fstream>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

DroneRobot::DroneRobot()
{

}

DroneRobot::DroneRobot(string& mapPath, string& trajectoryName, vector< heuristicType* > &heuristicTypes) :
    mapsColorConverted(3)
{
    // Read fullMap
    Mat originalMap = imread(mapPath+"/globalmap.jpg",CV_LOAD_IMAGE_COLOR);
    if(! originalMap.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find local map" << std::endl ;
        return;
    }
    globalMaps.push_back(originalMap);

    Mat grayMap;
    cvtColor(originalMap,grayMap,CV_BGR2GRAY);
    globalMaps.push_back(grayMap);

    Mat labMap;
    originalMap.convertTo(labMap, CV_32F);
    labMap*=1/255.0;
    cvtColor(labMap, labMap, CV_BGR2Lab);
    globalMaps.push_back(labMap);

    for(int i=0;i<heuristicTypes.size();++i)
    {
        heuristicType* h = heuristicTypes[i];

        switch(h->strategy)
        {
        case SSD:

            break;
        case DENSITY:

            DensityHeuristic* dh;
            double r = h->radius;

            // Try to create the kernel
            if(h->kernelType == KGAUSSIAN)
            {
                CGaussianC     g;
                g.initializeKernel(&r);
                dh = new DensityHeuristic(g.m_kernelMask, g.width(), g.height(), h->radius, h->threshold, h->colorDifference);
            }
            else if(h->kernelType == KANTIELIP)
            {
                CAntiEllipsoid a;
                a.initializeKernel(&r);
                dh = new DensityHeuristic(a.m_kernelMask, a.width(), a.height(), h->radius, h->threshold, h->colorDifference);
            }
            else if(h->kernelType == KCIRCULAR)
            {
                CCircular      c;
                c.initializeKernel(&r);
                dh = new DensityHeuristic(c.m_kernelMask, c.width(), c.height(), h->radius, h->threshold, h->colorDifference);
            }

            heuristics.push_back(dh);

            string densityfilename = mapPath + "/" + strategyName(h->strategy) +
                                    "_" + colorDifferenceName(h->colorDifference) +
                                    "_" + kernelName(h->kernelType) +
                                    "_R" + std::to_string(int(h->radius)) +
                                    "_T" + std::to_string(h->threshold) + ".txt";

            cout << "Open denity file: " << densityfilename << endl;
            FILE* f = fopen(densityfilename.c_str(),"r");
            MapGrid* mg = new MapGrid(f);
            maps.push_back(mg);

            break;
        }
    }

    // Read images names
    fstream input;
    input.open(trajectoryName,std::fstream::in);
    while(input.peek() != fstream::traits_type::eof()){
        string tempStr;
        getline(input,tempStr);
        imagesNames.push_back(tempStr);
    }
    cout << "Num images " << imagesNames.size() << endl;

}

DroneRobot::~DroneRobot()
{
}

void DroneRobot::initialize(ConnectionMode cmode, LogMode lmode, string fname)
{
    // Create windows for the global map and the local map
//    namedWindow( "Global Map", WINDOW_KEEPRATIO );
    namedWindow( "Local Map", WINDOW_KEEPRATIO );

//    namedWindow("Red",1);
//    namedWindow("Green",1);
//    namedWindow("Blue",1);

    // Create Matrices (make sure there is an image in input!)

//     Show global map (resized to 20% of normal size).
//    Mat window;
//    resize(globalMaps[0],window,Size(0,0),0.2,0.2);
//    imshow( "Global Map", window );

//    waitKey(1000);                                    // Wait for a keystroke in the window

    // Initialize MCL
    string locTechnique = "density";
    mcLocalization = new MCL(maps, globalMaps, locTechnique);

    step = 0;

    ready_ = true;
}

void DroneRobot::run()
{
    if(step >= imagesNames.size())
        return;

    // Read image
    Mat currentMap = imread(imagesNames[step++],CV_LOAD_IMAGE_COLOR);
    if(! currentMap.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find local map" << std::endl ;
        return;
    }

    // Create different versions of the input image
    createColorVersions(currentMap);

//    Mat window;
//    resize(currentMap,window,Size(0,0),0.2,0.2);
    imshow( "Local Map", currentMap );
    waitKey(100);

    if(step>1)
        odometry_ = findOdometry(prevMap,currentMap);
    cout << "Odometry: " << odometry_ << endl;
    prevMap = currentMap;

    Mat mask(currentMap.cols, currentMap.rows, CV_8SC3,Scalar(0,0,0));

    vector<int> densities(heuristics.size());
    vector<double> gradients(heuristics.size());
    for(int i=0;i<heuristics.size();++i)
    {
        densities[i]=heuristics[i]->calculateValue(
                    currentMap.cols/2, currentMap.rows/2,
                    &mapsColorConverted[heuristics[i]->getColorDifference()], &mask);
        // create discrete density value according to the corresonding mapgrid
        densities[i]=maps[i]->convertToMapGridDiscrete(densities[i]);
        gradients[i]=heuristics[i]->calculateGradientSobelOrientation(
                    currentMap.cols/2, currentMap.rows/2,
                    &mapsColorConverted[heuristics[i]->getColorDifference()], &mask);

    }
    // Obtain
    double time=0;


    mcLocalization->run(odometry_, currentMap, densities, gradients,time);

    // Navigation
    switch(motionMode_){
        case ENDING:
            cout << "Ending program" << endl;
            exit(0);
            break;
        default:
            break;
    }
}

Pose DroneRobot::findOdometry(Mat& prevImage, Mat& curImage)
{
    Mat im1_gray, im2_gray;

    // Reduce size of images
    resize(prevImage,im1_gray,Size(0,0),0.2,0.2);
    resize(curImage,im2_gray,Size(0,0),0.2,0.2);

    // Convert images to gray scale;
    cvtColor(im1_gray, im1_gray, CV_BGR2GRAY);
    cvtColor(im2_gray, im2_gray, CV_BGR2GRAY);

    // Define the motion model
    const int warp_mode = MOTION_EUCLIDEAN;

    // Set a 2x3 or 3x3 warp matrix depending on the motion model.
    Mat warp_matrix;

    // Initialize the matrix to identity
    if ( warp_mode == MOTION_HOMOGRAPHY )
        warp_matrix = Mat::eye(3, 3, CV_32F);
    else
        warp_matrix = Mat::eye(2, 3, CV_32F);

    // Specify the number of iterations.
    int number_of_iterations = 500;

    // Specify the threshold of the increment
    // in the correlation coefficient between two iterations
    double termination_eps = 1e-10;

    // Define termination criteria
    TermCriteria criteria (TermCriteria::COUNT+TermCriteria::EPS, number_of_iterations, termination_eps);

    // Run the ECC algorithm. The results are stored in warp_matrix.
    findTransformECC(
                     im1_gray,
                     im2_gray,
                     warp_matrix,
                     warp_mode,
                     criteria
                 );

    // Storage for warped image.
    Mat im2_aligned;

    if (warp_mode != MOTION_HOMOGRAPHY)
        // Use warpAffine for Translation, Euclidean and Affine
        warpAffine(curImage, im2_aligned, warp_matrix, prevImage.size(), INTER_LINEAR + WARP_INVERSE_MAP);
    else
        // Use warpPerspective for Homography
        warpPerspective (curImage, im2_aligned, warp_matrix, prevImage.size(),INTER_LINEAR + WARP_INVERSE_MAP);

    // Show final result
//    imshow("Image 1", prevImage);
//    imshow("Image 2", curImage);
//    imshow("Image 2 Aligned", im2_aligned);
//    waitKey(0);

//    cout << warp_matrix << endl;

    Pose p;
    p.x = warp_matrix.at<float>(0,2)/0.2;
    p.y = warp_matrix.at<float>(1,2)/0.2;
    p.theta = acos(warp_matrix.at<float>(0,0));

    return p;
}
void DroneRobot::createColorVersions(Mat& imageRGB)
{
    // RGB
    mapsColorConverted[0]=imageRGB.clone();

    // INTENSITYC:
    cvtColor(imageRGB, mapsColorConverted[1],CV_BGR2GRAY);

    // CIELAB1976 || CIELAB1994 || CMCLAB1984 || CIELAB2000 || CIELAB1994MIX || CIELAB2000MIX
    imageRGB.convertTo(mapsColorConverted[2], CV_32F);
    mapsColorConverted[2]*=1/255.0;
    cvtColor(imageRGB, mapsColorConverted[2], CV_BGR2Lab);
}
int DroneRobot::selectMapID(int colorDiff)
{
    switch(colorDiff)
    {
    /// index of mapsColorConverted
    /// 0: RGB
    /// 1: Intensity
    /// 2: LAB
    case INTENSITYC:
        return 1;
    case CIELAB1976:
    case CIELAB1994:
    case CMCLAB1984:
    case CIELAB2000:
    case CIELAB1994MIX:
    case CIELAB2000MIX:
        return 2;
        break;
    default:
        return 0;
    }
}

