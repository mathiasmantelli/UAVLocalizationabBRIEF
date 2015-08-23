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
        originalMap = imread(mapPath+"/globalmap.png",CV_LOAD_IMAGE_COLOR);
        if(! originalMap.data )                              // Check for invalid input
        {
            cout <<  "Could not open or find local map" << std::endl ;
            return;
        }
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

    // Open odometry file, if available
    string rawname = trajectoryName.substr(0, trajectoryName.find_last_of("."));
    odomFile.open(rawname+"_odom.txt",std::fstream::in);
    if(!odomFile.is_open()){
        offlineOdom = false;
    }else{
        offlineOdom = true;
        readRawOdometryFromFile(prevRawOdom);
    }


    for(int i=0;i<heuristicTypes.size();++i)
    {
        heuristicType* h = heuristicTypes[i];

        switch(h->strategy)
        {
        case CREATE_OBSERVATIONS:
            generateObservations(rawname+"/");
            exit(0);
            break;
        case SSD:
        {
            // Create a semi void class for ssd heuristics
            ColorHeuristic *ssdh = new ColorHeuristic(h->colorDifference, h->threshold);
            ssdHeuristics.push_back(ssdh);
            break;
        }
        case COLOR_ONLY:
        {
            // Create and store color heuristic
            ColorHeuristic *ch = new ColorHeuristic(h->colorDifference, h->threshold);
            colorHeuristics.push_back(ch);
            break;
        }
        case DENSITY:
        {
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

            densityHeuristic.push_back(dh);

            string densityfilename = mapPath + "/" + strategyName(h->strategy) +
                                    "_" + colorDifferenceName(h->colorDifference) +
                                    "_" + kernelName(h->kernelType) +
                                    "_R" + std::to_string(int(h->radius)) +
                                    "_T" + std::to_string(h->threshold) + ".txt";

            cout << "Open density file: " << densityfilename << endl;
            FILE* f = fopen(densityfilename.c_str(),"r");
            MapGrid* mg = new MapGrid(f);
            maps.push_back(mg);
            break;
        }
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

void DroneRobot::generateObservations(string imagePath)
{
    // Show global map (resized to 20% of normal size).
    Mat window;
    double scale = 1.0/5.0;
    resize(globalMaps[0],window,Size(0,0),scale,scale);
    imshow( "Global Map", window );

    stringstream ss;
    int id=0;

    // Read odom
    Pose p;
    while(readRawOdometryFromFile(p)){
        cout << "x:" << p.x << " y:" << p.y << " th:" << RAD2DEG(p.theta) << endl;

        circle(window,Point2f(p.x*scale,p.y*scale),10,Scalar(0,0,255));
        imshow( "Trajectory", window );

        Mat z = MCL::getParticleObservation(p,Size2f(320,240),globalMaps[0]);
        imshow( "Observation", z );

        // Save image
        ss.str("");
        ss << imagePath << setfill('0') << setw(6) << id << ".png";
        cout << "Image: " << ss.str() << endl;
        imwrite(ss.str(), z );

        waitKey(100);
        id++;
    }

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

    STRATEGY locTechnique;
    // Initialize MCL
    if(ssdHeuristics.size()>0)
        locTechnique = SSD;
    if(densityHeuristic.size()>0)
        locTechnique = DENSITY;
    if(colorHeuristics.size()>0)
        locTechnique = COLOR_ONLY;

    mcLocalization = new MCL(maps, globalMaps, locTechnique, prevRawOdom,
                             ssdHeuristics, colorHeuristics, densityHeuristic);

    step = 0;

    ready_ = true;
}

void DroneRobot::run()
{
    if(step >= imagesNames.size())
        return;

    // Read image
    Mat currentMap = imread(imagesNames[step],CV_LOAD_IMAGE_COLOR);
    if(! currentMap.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find local map" << std::endl ;
        return;
    }

    cout << "Image" << step;
    step += 1;

    // Create different versions of the input image
    createColorVersions(currentMap);


//    Mat window;
//    resize(currentMap,window,Size(0,0),0.2,0.2);
    imshow( "Local Map", currentMap );
    waitKey(100);

    if(step>1){
        if(offlineOdom)
            odometry_ = readOdometry();
        else
            odometry_ = findOdometry(prevMap,currentMap);
    }else{
        odometry_ = Pose(0,0,0);
    }
    cout << "Odometry: " << odometry_ << endl;

    prevMap = currentMap;

    // Defaut all free binary map
    Mat mask(currentMap.cols, currentMap.rows, CV_8SC3,Scalar(0,0,0));

    vector<int> densities(densityHeuristic.size());
    vector<double> gradients(densityHeuristic.size());
    double val=0;
    cout << "Heuristics: ";
    for(int i=0;i<densityHeuristic.size();++i)
    {
        // Choose appropriate color scheme
        int mapID = selectMapID(densityHeuristic[i]->getColorDifference());
        // create discrete density value according to the corresonding mapgrid
        val=densityHeuristic[i]->calculateValue(
                    currentMap.cols/2, currentMap.rows/2,
                    &mapsColorConverted[mapID], &mask);
        densities[i]=maps[i]->convertToMapGridDiscrete(val);
        // and do the same for the angles
        gradients[i]=densityHeuristic[i]->calculateGradientSobelOrientation(
                    currentMap.cols/2, currentMap.rows/2,
                    &mapsColorConverted[mapID], &mask);
        cout << densities[i] << " ";
    }
    cout << endl;

    // Obtain
    double time=0;


    // Color ONLY
    for(int i=0; i<colorHeuristics.size();++i)
    {
        int mapID = selectMapID(colorHeuristics[i]->getColorDifference());
        colorHeuristics[i]->setBaselineColor(currentMap.cols/2, currentMap.rows/2, &mapsColorConverted[mapID]);
    }

    mcLocalization->run(odometry_, currentMap, densities, gradients,time, prevRawOdom);

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

Pose DroneRobot::readOdometry()
{
    Pose newRawOdom;
    readRawOdometryFromFile(newRawOdom);

    double deltaX = newRawOdom.x - prevRawOdom.x;
    double deltaY = newRawOdom.y - prevRawOdom.y;
    Pose odom;
    odom.x = cos(-prevRawOdom.theta)*deltaX - sin(-prevRawOdom.theta)*deltaY;
    odom.y = sin(-prevRawOdom.theta)*deltaX + cos(-prevRawOdom.theta)*deltaY;
    odom.theta = getDiffAngle(newRawOdom.theta,prevRawOdom.theta);
//    cout << "RAW " << newRawOdom.theta << " - " << prevRawOdom.theta << " = " << odom.theta << endl;

    prevRawOdom = newRawOdom;

    return odom;
}

bool DroneRobot::readRawOdometryFromFile(Pose& p)
{
   if(odomFile.peek() == fstream::traits_type::eof())
       return false;

   string tempStr;
   odomFile >> p.x >> p.y >> p.theta;
   getline(odomFile,tempStr);

   p.theta = DEG2RAD(p.theta);
   while(p.theta > M_PI)
       p.theta -= M_PI;
   while(p.theta < -M_PI)
       p.theta += M_PI;

   return true;
}

Pose DroneRobot::findOdometry(Mat& prevImage, Mat& curImage)
{
    Mat im1_gray, im2_gray;

//    // Reduce size of images
//    resize(prevImage,im1_gray,Size(0,0),0.2,0.2);
//    resize(curImage,im2_gray,Size(0,0),0.2,0.2);

    // Convert images to gray scale;
    cvtColor(prevImage, im1_gray, CV_BGR2GRAY);
    cvtColor(curImage, im2_gray, CV_BGR2GRAY);

//    Canny(im1_gray,im1_gray,10,30,5);
//    Canny(im2_gray,im2_gray,10,30,5);

//    imshow("Canny 1", im1_gray);
//    imshow("Canny 2", im2_gray);

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


    vector<Point2f> yourPoints;
    yourPoints.push_back(Point2f(0,0));
    yourPoints.push_back(Point2f(curImage.cols,0));
    yourPoints.push_back(Point2f(curImage.cols,curImage.rows));
    yourPoints.push_back(Point2f(0,curImage.rows));
    yourPoints.push_back(Point2f(curImage.cols/2,curImage.rows/2));
    yourPoints.push_back(Point2f(curImage.cols,curImage.rows/2));
    yourPoints.push_back(Point2f(curImage.cols/2,0));

    vector<Point2f> transformedPoints;
    transformedPoints.resize(yourPoints.size());

    Mat transf = Mat::eye(3, 3, CV_32F);
    Mat aux = transf.colRange(0,3).rowRange(0,2);
    warp_matrix.copyTo(aux);

    perspectiveTransform(yourPoints, transformedPoints, transf.inv());

    vector<Point2f> totalPoints;
    totalPoints.reserve( yourPoints.size() + transformedPoints.size() ); // preallocate memory
    totalPoints.insert( totalPoints.end(), yourPoints.begin(), yourPoints.end() );
    totalPoints.insert( totalPoints.end(), transformedPoints.begin(), transformedPoints.end() );

    Rect r = boundingRect(totalPoints);

//    cout << "warp:" << warp_matrix << endl;
//    cout << "transf:" << transf << endl;
//    cout << "r:" << r.x << ' ' << r.y << endl;

    Mat displ = Mat::eye(3, 3, CV_32F);
    displ.at<float>(0,2) = -r.x;
    displ.at<float>(1,2) = -r.y;
    perspectiveTransform(yourPoints, yourPoints, displ);
    perspectiveTransform(transformedPoints, transformedPoints, displ);

    Mat blank(r.height, r.width, CV_8UC3, Scalar(0));
//    waitKey(0);


    // Storage for warped image.
    Mat im2_aligned;
    warp_matrix.at<float>(0,2) += r.x;
    warp_matrix.at<float>(1,2) += r.y;
//    cout << "warp2:" << warp_matrix << endl;

    if (warp_mode != MOTION_HOMOGRAPHY)
        // Use warpAffine for Translation, Euclidean and Affine
        warpAffine(curImage, im2_aligned, warp_matrix, blank.size(), INTER_LINEAR + WARP_INVERSE_MAP);
    else
        // Use warpPerspective for Homography
        warpPerspective (curImage, im2_aligned, warp_matrix, blank.size(),INTER_LINEAR + WARP_INVERSE_MAP);

    // Draw aligned curImage in blank
    blank += im2_aligned*0.5;

    // Draw prevImage in blank
    Mat aux1 = blank.colRange(-r.x,-r.x+prevImage.cols).rowRange(-r.y,-r.y+prevImage.rows);
    Mat im1 = prevImage*0.5 + aux1;
    im1.copyTo(aux1);

    // Draw rectangles
    for (int i = 0; i < 4; i++)
        line(blank, yourPoints[i], yourPoints[(i+1)%4], Scalar(0,255,0));
    for (int i = 0; i < 4; i++)
        line(blank, transformedPoints[i], transformedPoints[(i+1)%4], Scalar(0,0,255));

    // Draw arrow
    rectangle(blank,Rect(yourPoints[4]-Point2f(2.5,2.5), Size2f(5,5)), Scalar(0,255,0));
    circle(blank,transformedPoints[4],10,Scalar(0,0,255));
    line(blank, yourPoints[4], transformedPoints[4], Scalar(0,0,255));
    line(blank, transformedPoints[4], transformedPoints[5], Scalar(255,255,255));
    line(blank, transformedPoints[4], transformedPoints[6], Scalar(255,255,255));


    // Show final result
//    namedWindow( "Image 1", WINDOW_KEEPRATIO );
//    namedWindow( "Image 2", WINDOW_KEEPRATIO );
//    namedWindow( "Image 2 Aligned", WINDOW_KEEPRATIO );
//    namedWindow( "Image Blank", WINDOW_KEEPRATIO );
    imshow("Image Blank", blank);
    imshow("Image 1", im1_gray);
    imshow("Image 2", im2_gray);
//    imshow("Image 2 Aligned", im2_aligned);
//    waitKey(10);

    char k = waitKey(0);
    if (k=='g' || k=='G')
        cout << " GOOD odometry! :)" << endl;
    else if (k=='b' || k=='B')
        cout << " BAD odometry! :(" << endl;


    Pose p;
    p.x = warp_matrix.at<float>(0,2);
    p.y = warp_matrix.at<float>(1,2);
    p.theta = acos(warp_matrix.at<float>(0,0));

    return p;
}

void DroneRobot::createColorVersions(Mat& imageRGB)
{
    // RGB
    mapsColorConverted[0]=imageRGB.clone();

    // INTENSITYC:
    cvtColor(imageRGB, mapsColorConverted[1],CV_BGR2GRAY);
    cvtColor(mapsColorConverted[1], mapsColorConverted[1],CV_GRAY2BGR);

    // CIELAB1976 || CIELAB1994 || CMCLAB1984 || CIELAB2000 || CIELAB1994MIX || CIELAB2000MIX
    imageRGB.convertTo(mapsColorConverted[2], CV_32F);
    mapsColorConverted[2]*=1/255.0;
    cvtColor(imageRGB, mapsColorConverted[2], CV_BGR2Lab);
}

int selectMapID(int colorDiff)
{
    switch(colorDiff)
    {
    /// index of mapsColorConverted
    /// 0: RGB, 1: Intensity, 2: LAB
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
