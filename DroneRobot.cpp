#include "DroneRobot.h"

#include "SomeKernels.h"

#include <unistd.h>
#include <fstream>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"

DroneRobot::DroneRobot()
{

}

DroneRobot::DroneRobot(string& mapPath, string& trajectoryName, vector< heuristicType* > &heuristicTypes)
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


    // extract binary map
     Mat map= imread(mapPath+"/globalmap_Mapa.png",CV_LOAD_IMAGE_COLOR);

    // check if image is valid
    if(!map.data)
    {
        cerr<<"DroneRobot: Map Global Map Binary Image is empty"<< endl;
        exit(-1);
    }
    globalMaps.push_back(map);


    // Open odometry file, if available
    string rawname = trajectoryName.substr(0, trajectoryName.find_last_of("."));
    odomFile.open(rawname+"_odom.txt",std::fstream::in);
    if(!odomFile.is_open()){
        offlineOdom = false;
    }else{
        offlineOdom = true;
        readRawOdometryFromFile(prevRawOdom);
    }

    // Initialize heuristics
    for(int i=0;i<heuristicTypes.size();++i)
    {
        heuristicType* hT = heuristicTypes[i];
        Heuristic* heur = NULL;
        MapGrid* mg = NULL;

        int id=getNextProperHeuristicID(hT->strategy);

        if(hT->strategy == TEMPLATE_MATCHING)
        {
            locTechnique = TEMPLATE_MATCHING;
            break;
        }
        else if(hT->strategy == CREATE_OBSERVATIONS)
        {
            generateObservations(rawname+"/");
            exit(0);
        }
        else if(hT->strategy == SSD)
        {
            // Create a semi void class for ssd heuristics
            heur = new ColorHeuristic(SSD, id, hT->colorDifference, hT->threshold);
        }
        else if(hT->strategy == COLOR_ONLY)
        {
            // Create color heuristic
            heur = new ColorHeuristic(COLOR_ONLY, id, hT->colorDifference, hT->threshold);
        }
        else if(hT->strategy == DENSITY || hT->strategy == ENTROPY || hT->strategy == MUTUAL_INFORMATION )
        {
            double r = hT->radius;
            double* kMask;
            int kWidth, kHeight;

            // Try to create the kernel
            if(hT->kernelType == KGAUSSIAN)
            {
                CGaussianC g;
                g.initializeKernel(&r);
                kMask = g.m_kernelMask;
                kWidth = g.width();
                kHeight = g.height();
            }
            else if(hT->kernelType == KANTIELIP)
            {
                CAntiEllipsoid a;
                a.initializeKernel(&r);
                kMask = a.m_kernelMask;
                kWidth = a.width();
                kHeight = a.height();
            }
            else if(hT->kernelType == KCIRCULAR)
            {
                CCircular c;
                c.initializeKernel(&r);
                kMask = c.m_kernelMask;
                kWidth = c.width();
                kHeight = c.height();
            }

            // Create kernel heuristic
            if(hT->strategy == DENSITY)
                heur = new DensityHeuristic(DENSITY, id, kMask, kWidth, kHeight, hT->radius, hT->threshold, hT->colorDifference);
            else if(hT->strategy == ENTROPY)
                heur = new EntropyHeuristic(ENTROPY, id, kMask, kWidth, kHeight, hT->radius, 2.3, hT->colorDifference, hT->threshold);
            else if(hT->strategy == MUTUAL_INFORMATION)
                heur = new MIHeuristic(MUTUAL_INFORMATION, id, kMask, kWidth, kHeight, hT->radius, 2.3, hT->colorDifference, hT->threshold);

            // Open cached map
            string mapfilename = mapPath + "/";
            if(hT->strategy == DENSITY)
                mapfilename += "DENSITY";
            else
                mapfilename += "ENTROPY";
            mapfilename += "_" + colorDifferenceName(hT->colorDifference) +
                           "_" + kernelName(hT->kernelType) +
                           "_R" + std::to_string(int(hT->radius));
            if(hT->strategy == DENSITY)
                mapfilename += "_T" + std::to_string(hT->threshold) + ".txt";
            else
            {
                // Entropy Heuistic receives bins from threshold
                mapfilename += "_T" + std::to_string(int(hT->threshold)) + ".txt";
            }

            cout << "Open file: " << mapfilename << endl;
            FILE* f = fopen(mapfilename.c_str(),"r");
            mg = new MapGrid(f);
        }

        heuristics.push_back(heur);
        cachedMaps.push_back(mg);
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

int DroneRobot::getNextProperHeuristicID(STRATEGY type)
{
    int id=0;
    for(int s=0; s<heuristics.size(); s++)
        if(heuristics[s]->getType() == type)
            id++;
    return id;
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

        Mat z = Utils::getRotatedROIFromImage(p,Size2f(320,240),globalMaps[0]);
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

//    if(ssdHeuristics.size()>0)
//        locTechnique = SSD;
//    if(densityHeuristic.size()>0)
//        locTechnique = DENSITY;
//    if(colorHeuristics.size()>0)
//        locTechnique = COLOR_ONLY;

    // Initialize MCL
    mcLocalization = new MCL(heuristics, cachedMaps, globalMaps, prevRawOdom);

    step = 0;

    ready_ = true;
}

void computeEntropyMap(Mat& image_orig, Mat& mask)
{
    Mat image;
    image = image_orig;
//    resize(image_orig,image,Size(0,0),0.5,0.5);

    image.convertTo(image,CV_32F);
    image *= 1.0/255.0;
    cvtColor(image, image, CV_BGR2Lab);

//    cvtColor(image, image, CV_BGR2GRAY);
//    cvtColor(image, image, CV_GRAY2BGR);
    imshow("input",image);
//    waitKey(0);

    Mat entropyMap(image.rows, image.cols, CV_64F, Scalar(0.0));

    double r = 10;
    double l = 2.3;
    int cd = CIELAB1976;
    unsigned int bins = 22;
    CGaussianC     g;
    g.initializeKernel(&r);
    EntropyHeuristic *eih = new EntropyHeuristic(ENTROPY, 0, g.m_kernelMask, g.width(), g.height(), r, l, cd, bins);

    cout << "Computing entropy " << endl;

    for(int x=0; x<image.cols; ++x){
        for(int y=0; y<image.rows; ++y){
            entropyMap.at<double>(y,x) = eih->calculateValue(x,y,&image,&mask);
        }
        if(x%8==0)
            cout << "\r" << x*100/image.cols << "%" << flush;
    }
    cout << "\r100%" << endl;

    imshow("entropy",entropyMap);

    waitKey(0);
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

//    Mat window;
//    resize(currentMap,window,Size(0,0),0.2,0.2);
    imshow( "Local Map", currentMap );
    waitKey(100);

    cout << "Image" << step << ' ';
    step += 1;

//    computeEntropyMap(currentMap, mask);
//    return;

    if(locTechnique == TEMPLATE_MATCHING){
        localizeWithTemplateMatching(currentMap);
        return;
    }

    // Compute or read Odometry
    if(step>1){
        if(offlineOdom)
            odometry_ = readOdometry();
        else
//            odometry_ = findOdometry(prevMap,currentMap);
            odometry_ = findOdometryUsingFeatures(prevMap,currentMap);
    }else{
        odometry_ = Pose(0,0,0);
    }
    cout << "Odometry: " << odometry_ << endl;

    prevMap = currentMap;

    // Obtain
    double time=0;

    // Run Monte Carlo Localization
    mcLocalization->run(odometry_, currentMap, time, prevRawOdom);

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

void DroneRobot::localizeWithTemplateMatching(Mat& currentMap)
{
    cout << "Templatão" << endl;
    Mat whiteMask(currentMap.rows, currentMap.cols, CV_8UC3, Scalar(255,255,255));

    for(int angle=360; angle>=0; angle-=10){

        Mat templ = Utils::rotateImage(currentMap,angle);
        Mat mask = Utils::rotateImage(whiteMask,angle);
        imshow( "Local Map", templ );
        imshow( "Mask", mask );

        /// Source image to display
        Mat img_display, result;
        globalMaps[0].copyTo( img_display );

        Point matchLoc = Utils::templateMatching(globalMaps[0],templ,result,CV_TM_CCORR_NORMED,mask);

        /// Show me what you got
        rectangle( img_display, matchLoc, Point( matchLoc.x + currentMap.cols , matchLoc.y + currentMap.rows ), Scalar::all(0), 2, 8, 0 );
        rectangle( result, matchLoc, Point( matchLoc.x + currentMap.cols , matchLoc.y + currentMap.rows ), Scalar::all(0), 2, 8, 0 );

        resize(img_display,img_display,Size(0,0),0.2,0.2);
        imshow( "image_window", img_display );
        resize(result,result,Size(0,0),0.2,0.2);
        imshow( "result", result);
        waitKey(0);
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
    odom.theta = Utils::getDiffAngle(newRawOdom.theta,prevRawOdom.theta);
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

void DroneRobot::drawMatchedImages(Mat& prevImage, Mat& curImage, Mat& warp_matrix, const int warp_mode)
{
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

    Mat transf;
//    if (warp_mode != MOTION_HOMOGRAPHY){
        transf = Mat::eye(3, 3, CV_32F);
        Mat aux = transf.colRange(0,3).rowRange(0,2);
        warp_matrix.copyTo(aux);
//    }else{
//        transf = warp_matrix;
//    }

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
//    imshow("Image 2 Aligned", im2_aligned);
    waitKey(0);
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

//    imshow("Image 1", im1_gray);
//    imshow("Image 2", im2_gray);

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

    // Check quality of matching
    Mat whiteMask(curImage.rows, curImage.cols, CV_8UC3, Scalar(255,255,255));
    Mat mask;
    warpAffine(whiteMask, mask, warp_matrix, whiteMask.size(), INTER_LINEAR + WARP_INVERSE_MAP);

    Mat templ(curImage.rows, curImage.cols, CV_8UC3, Scalar(255,255,255));
    warpAffine(curImage, templ, warp_matrix, curImage.size(), INTER_LINEAR + WARP_INVERSE_MAP);
//    imshow("Mask", mask);
//    imshow("Aligned", templ);

    /// "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
    int match_method = CV_TM_CCORR_NORMED;

    Mat result;
    result.create( 1, 1, CV_32FC1 );

    matchTemplate( prevImage, templ, result, match_method, mask );
    double w = result.at<float>(0,0);
    cout << "ODOMETRIA " << (w>0.96?"BOA":"RUIM") << " MANO (" << w << ")" << endl;

    drawMatchedImages(prevImage,curImage,warp_matrix);

    char k = waitKey(20);
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


Pose DroneRobot::findOdometryUsingFeatures(Mat& prevImage, Mat& curImage)
{
    Mat img_1, img_2;

    // Convert images to gray scale;
    cvtColor(prevImage, img_1, CV_BGR2GRAY);
    cvtColor(curImage, img_2, CV_BGR2GRAY);

    //-- Step 1 & 2: Detect the keypoints using Detector & Calculate descriptors (feature vectors)

    int minHessian = 400;
    Ptr<Feature2D> detector=xfeatures2d::SIFT::create(minHessian);
    Ptr<Feature2D> extractor=xfeatures2d::SIFT::create(minHessian);


    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;

//    detector->detectAndCompute(img_1,noArray(),keypoints_1,descriptors_1);
//    detector->detectAndCompute(img_2,noArray(),keypoints_2,descriptors_2);

    detector->detect(img_1,keypoints_1);
    extractor->compute(img_1,keypoints_1,descriptors_1);

    detector->detect(img_2,keypoints_2);
    extractor->compute(img_2,keypoints_2,descriptors_2);

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist )
            min_dist = dist;
        if( dist > max_dist )
            max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );

    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //-- small)
    //-- PS.- radiusMatch can also be used here.
    std::vector< DMatch > good_matches;

    for( int i = 0; i < descriptors_1.rows; i++ )
    {
        if( matches[i].distance <= max(2*min_dist, 0.02) ){
            good_matches.push_back( matches[i]);
        }
    }

    //-- Draw only "good" matches
    Mat img_matches;
    drawMatches( img_1, keypoints_1, img_2, keypoints_2,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    for( int i = 0; i < (int)good_matches.size(); i++ )
        printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx );

    int minNumPoints = 4;

    // Find transformation
    std::vector<Point2f> points1, points2;
    for( int i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        points1.push_back( keypoints_1[ good_matches[i].queryIdx ].pt );
        points2.push_back( keypoints_2[ good_matches[i].trainIdx ].pt );
    }

    Mat H;
    if(good_matches.size()>=minNumPoints)
        H = findHomography( points1, points2, CV_RANSAC );

//    //-- Get the corners from the image_1 ( the object to be "detected" )
//    std::vector<Point2f> obj_corners(4);
//    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_1.cols, 0 );
//    obj_corners[2] = cvPoint( img_1.cols, img_1.rows ); obj_corners[3] = cvPoint( 0, img_1.rows );
//    std::vector<Point2f> scene_corners(4);

//    perspectiveTransform( obj_corners, scene_corners, H);

//    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
//    line( img_matches, scene_corners[0] + Point2f( img_1.cols, 0), scene_corners[1] + Point2f( img_1.cols, 0), Scalar(0, 255, 0), 4 );
//    line( img_matches, scene_corners[1] + Point2f( img_1.cols, 0), scene_corners[2] + Point2f( img_1.cols, 0), Scalar( 0, 255, 0), 4 );
//    line( img_matches, scene_corners[2] + Point2f( img_1.cols, 0), scene_corners[3] + Point2f( img_1.cols, 0), Scalar( 0, 255, 0), 4 );
//    line( img_matches, scene_corners[3] + Point2f( img_1.cols, 0), scene_corners[0] + Point2f( img_1.cols, 0), Scalar( 0, 255, 0), 4 );

    //-- Show detected matches
    imshow( "Good Matches", img_matches );
//    if(good_matches.size()>=minNumPoints)
    if(!H.empty())
        drawMatchedImages(prevImage,curImage,H,MOTION_HOMOGRAPHY);
    else
        waitKey(0);

    Pose p;
//    p.x = warp_matrix.at<float>(0,2);
//    p.y = warp_matrix.at<float>(1,2);
//    p.theta = acos(warp_matrix.at<float>(0,0));

    return p;
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
