#include "DroneRobot.h"

#include <unistd.h>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

DroneRobot::DroneRobot()
{

}

DroneRobot::~DroneRobot()
{
}

void DroneRobot::initialize(ConnectionMode cmode, LogMode lmode, string fname)
{
    dataset = fname;

    // Read fullMap
    globalMap = imread(fname+"/globalmap.jpg",CV_LOAD_IMAGE_COLOR);
    if(! globalMap.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find local map" << std::endl ;
        return;
    }

    // Create windows for the global map and the local map
//    namedWindow( "Global Map", WINDOW_AUTOSIZE );
    namedWindow( "Local Map", WINDOW_AUTOSIZE );

//    namedWindow("Red",1);
//    namedWindow("Green",1);
//    namedWindow("Blue",1);

    // Create Matrices (make sure there is an image in input!)

//     Show global map (resized to 20% of normal size).
    Mat window;
    resize(globalMap,window,Size(0,0),0.2,0.2);
//    imshow( "Global Map", window );

    waitKey(1000);                                    // Wait for a keystroke in the window

    // Initialize MCL
    string locTechnique = "ssd";
    vector<MapGrid*> tempVec;
    mcLocalization = new MCL(tempVec, globalMap, locTechnique);

    // Read images names
    fstream input;
    input.open(fname+"/input.txt",std::fstream::in);
    while(input.peek() != fstream::traits_type::eof()){
        string tempStr;
        getline(input,tempStr);
        imagesNames.push_back(fname+"/"+tempStr);
    }
    cout << "Num images " << imagesNames.size() << endl;

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
    Mat window;
    resize(currentMap,window,Size(0,0),0.2,0.2);
    imshow( "Local Map", window );
    waitKey(100);

    if(step>1)
        odometry_ = findOdometry(prevMap,currentMap);
    cout << "Odometry: " << odometry_ << endl;
    prevMap = currentMap;

    vector<int> densities;
    vector<double> gradients;
    double time=0;
    mcLocalization->run(odometry_,currentMap,densities,gradients,time);

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
    p.x = warp_matrix.at<float>(0,2);
    p.y = warp_matrix.at<float>(1,2);
    p.theta = acos(warp_matrix.at<float>(0,0));

    return p;
}
