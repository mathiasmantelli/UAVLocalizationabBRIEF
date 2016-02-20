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

DroneRobot::DroneRobot(string& mapPath, string& trajectoryName, vector< heuristicType* > &heuristicTypes, bool quiet, string& oName, int startVal, int finishVal)
{
    runQuiet = quiet;
    if(runQuiet){
        cout << "Initializing" << flush;
        disableCout();
    }

    start=startVal;
    finish=finishVal;
    current=start;
    outputName=oName;

    // Read fullMap
    cv::Mat originalMap = cv::imread(mapPath+"/globalmap.jpg",CV_LOAD_IMAGE_COLOR);
    if(! originalMap.data )                              // Check for invalid input
    {
        originalMap = cv::imread(mapPath+"/globalmap.png",CV_LOAD_IMAGE_COLOR);
        cout<<"Endereco:"<<mapPath+"globalmap.png"<<endl;

        if(! originalMap.data )                              // Check for invalid input
        {
            cout <<  "Could not open or find local map" << std::endl ;
            cout<<"Endereco:"<<mapPath+"globalmap.png"<<endl;
            return;
        }
    }
    globalMaps.push_back(originalMap);

    cv::Mat grayMap;
    cvtColor(originalMap,grayMap,CV_BGR2GRAY);
    cvtColor(grayMap,grayMap,CV_GRAY2BGR);
    globalMaps.push_back(grayMap);

    cv::Mat labMap;
    originalMap.convertTo(labMap, CV_32FC3);
    labMap*=1/255.0;
    cvtColor(labMap, labMap, CV_BGR2Lab);
    globalMaps.push_back(labMap);

    cv::Mat hsvMap;
    cvtColor(originalMap, hsvMap, CV_BGR2HSV);
    globalMaps.push_back(hsvMap);

    if(runQuiet){
        enableCout();
        cout << "." << flush;
        disableCout();
    }

    // extract binary map
    cv::Mat map= cv::imread(mapPath+"/globalmap_Mapa.png",CV_LOAD_IMAGE_COLOR);

    // check if image is valid
    if(!map.data)
    {
        cerr<<"DroneRobot: Map Global Map Binary Image is empty"<< endl;
        exit(-1);
    }
    globalMaps.push_back(map);

    // Open ground truth file, if available
    rawname = trajectoryName.substr(0, trajectoryName.find_last_of("."));
    truthFile.open(rawname+"_truth.txt",std::fstream::in);
    if(!truthFile.is_open()){
        availableGTruth = false;
    }else{
        availableGTruth = true;
        realPose = readGroundTruth();
    }

    // Open odometry file, if available
    odomFile.open(rawname+"_odom.txt",std::fstream::in);
    if(!odomFile.is_open()){

        // Check if there are available offline odoms
        vector<string> opaths = Utils::getListOfFiles(rawname);
        cout << "Num offline odoms " << opaths.size() << endl;

        if(!opaths.empty()){
            std::default_random_engine generator;
            std::uniform_int_distribution<int> randomVal(0,opaths.size());
            int id=randomVal(generator);

            odomFile.open(rawname+"/"+opaths[id],std::fstream::in);
            offlineOdom = true;
            isRawOdom = false;
        }else{
            offlineOdom = false;

            /******************** Prepare odom file *****************************/
            time_t t = time(0);
            struct tm *now = localtime(&t);
            stringstream odomName;

            odomName << rawname << "/odom-" << -100+now->tm_year
                            << setfill('0') << setw(2) << 1+now->tm_mon
                            << setfill('0') << setw(2) << now->tm_mday << '-'
                            << setfill('0') << setw(2) << now->tm_hour
                            << setfill('0') << setw(2) << now->tm_min
                            << setfill('0') << setw(2) << now->tm_sec << ".txt";
            cout << odomName.str() << endl; cout.flush();

            odomFile.open(odomName.str(),std::fstream::out);
        }
    }else{
        offlineOdom = true;
        isRawOdom = true;
        readRawOdometryFromFile(prevRawOdom);
    }

    if(runQuiet){
        enableCout();
        cout << "." << flush;
        disableCout();
    }

    slowMethod=false;
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
        if(hT->strategy == BRIEF)
        {
            heur = new BriefHeuristic(BRIEF, id, hT->colorDifference, hT->threshold);
            if(hT->lowThreshold != -1) ((BriefHeuristic*)heur)->lowThreshold = hT->lowThreshold;
            if(hT->multiplierThreshold != -1) ((BriefHeuristic*)heur)->multiplierThreshold = hT->multiplierThreshold;
            if(hT->margin != -1) ((BriefHeuristic*)heur)->margin = hT->margin;
            if(hT->numberPairs != -1) ((BriefHeuristic*)heur)->totalPairs = hT->numberPairs;
            ((BriefHeuristic*)heur)->printInfo();
        }
        if(hT->strategy == FEATURE_MATCHING)
        {
            locTechnique = FEATURE_MATCHING;
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
        else if(hT->strategy == UNSCENTED_COLOR)
        {
            // Create unscented color heuristic
            heur = new UnscentedColorHeuristic(UNSCENTED_COLOR, id, hT->colorDifference, hT->threshold);
        }
        else if(   hT->strategy == DENSITY
                || hT->strategy == ENTROPY
                || hT->strategy == MEAN_SHIFT
                || hT->strategy == SIFT_MCL
                || hT->strategy == MUTUAL_INFORMATION
                || hT->strategy == HISTOGRAM_MATCHING)
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

            if(hT->strategy == MEAN_SHIFT)
            {
                heur = new MeanShiftHeuristic(MEAN_SHIFT, id, kMask, kWidth, kHeight, hT->radius, 2.3, hT->colorDifference);
            }
            else if(hT->strategy == SIFT_MCL)
            {
                    heur = new SIFTHeuristic(SIFT_MCL, id, grayMap, kMask, kWidth, kHeight, hT->radius, 2.3, hT->colorDifference);
            }
            else if(hT->strategy == HISTOGRAM_MATCHING)
                    heur = new HistogramHeuristic(HISTOGRAM_MATCHING, id, kMask, kWidth, kHeight, hT->radius, 2.3, hT->colorDifference, hT->threshold);
            else
            {
                // Create kernel heuristic
                if(hT->strategy == DENSITY)
                    heur = new DensityHeuristic(DENSITY, id, kMask, kWidth, kHeight, hT->radius, hT->threshold, hT->colorDifference);
                else if(hT->strategy == ENTROPY)
                    heur = new EntropyHeuristic(ENTROPY, id, kMask, kWidth, kHeight, hT->radius, 2.3, hT->colorDifference, hT->threshold);
                else if(hT->strategy == MUTUAL_INFORMATION){
                    heur = new MIHeuristic(MUTUAL_INFORMATION, id, kMask, kWidth, kHeight, hT->radius, 2.3, hT->colorDifference, hT->threshold);
                    slowMethod=true;
                }

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
        }

        if(runQuiet){
            enableCout();
            cout << "." << flush;
            disableCout();
        }

        heuristics.push_back(heur);
        cachedMaps.push_back(mg);
    }

    if(runQuiet){
        enableCout();
        cout << "." << flush;
        disableCout();
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

    if(heuristicTypes[0]->strategy == FEATURE_MATCHING)
        initializeFeatureMatching();

    if(runQuiet){
        enableCout();
        cout << " Running " << endl;
        disableCout();
    }
}

void DroneRobot::reinitialize()
{
    // Open ground truth file, if available
    if(truthFile.is_open())
        truthFile.close();
    truthFile.open(rawname+"_truth.txt",std::fstream::in);
    if(!truthFile.is_open()){
        availableGTruth = false;
    }else{
        availableGTruth = true;
        realPose = readGroundTruth();
    }

    // Open odometry file, if available
    if(odomFile.is_open())
        odomFile.close();
    odomFile.open(rawname+"_odom.txt",std::fstream::in);
    if(!odomFile.is_open()){

        // Check if there are available offline odoms
        vector<string> opaths = Utils::getListOfFiles(rawname);
        cout << "Num offline odoms " << opaths.size() << endl;

        if(!opaths.empty()){
            std::default_random_engine generator;
            std::uniform_int_distribution<int> randomVal(0,opaths.size());
            int id=randomVal(generator);

            odomFile.open(rawname+"/"+opaths[id],std::fstream::in);
            offlineOdom = true;
            isRawOdom = false;
        }else{
            offlineOdom = false;

            /******************** Prepare odom file *****************************/
            time_t t = time(0);
            struct tm *now = localtime(&t);
            stringstream odomName;

            odomName << rawname << "/odom-" << -100+now->tm_year
                            << setfill('0') << setw(2) << 1+now->tm_mon
                            << setfill('0') << setw(2) << now->tm_mday << '-'
                            << setfill('0') << setw(2) << now->tm_hour
                            << setfill('0') << setw(2) << now->tm_min
                            << setfill('0') << setw(2) << now->tm_sec << ".txt";
            cout << odomName.str() << endl; cout.flush();

            odomFile.open(odomName.str(),std::fstream::out);
        }
    }else{
        offlineOdom = true;
        isRawOdom = true;
        readRawOdometryFromFile(prevRawOdom);
    }

    string oName;
    if(!outputName.empty())
        oName = outputName+to_string(current)+".txt";
    mcLocalization->restart(realPose,oName);

    step = 0;

    if(runQuiet){
        enableCout();
        cout << " Running " << endl;
        disableCout();
    }

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
    cv::Mat window;
    double scale = 1.0/5.0;
    resize(globalMaps[0],window,cv::Size(0,0),scale,scale);
    imshow( "Global Map", window );

    stringstream ss;
    int id=0;

    // Read odom
    Pose p;
    while(readRawOdometryFromFile(p)){
        cout << "x:" << p.x << " y:" << p.y << " th:" << RAD2DEG(p.theta) << endl;

        circle(window,cv::Point2f(p.x*scale,p.y*scale),10,cv::Scalar(0,0,255));
        imshow( "Trajectory", window );

        cv::Mat z = Utils::getRotatedROIFromImage(p,cv::Size2f(320,240),globalMaps[0]);
        imshow( "Observation", z );

        // Save image
        ss.str("");
        ss << imagePath << setfill('0') << setw(6) << id << ".png";
        cout << "Image: " << ss.str() << endl;
        imwrite(ss.str(), z );

        cv::waitKey(100);
        id++;
    }

}

void DroneRobot::initialize(ConnectionMode cmode, LogMode lmode, string fname)
{
    // Create windows for the global map and the local map
//    cv::namedWindow( "Global Map", cv::WINDOW_KEEPRATIO );
    cv::namedWindow( "Local Map", cv::WINDOW_KEEPRATIO );

//    cv::namedWindow("Red",1);
//    cv::namedWindow("Green",1);
//    cv::namedWindow("Blue",1);

    // Create Matrices (make sure there is an image in input!)

//     Show global map (resized to 20% of normal size).
//    cv::Mat window;
//    resize(globalMaps[0],window,cv::Size(0,0),0.2,0.2);
//    imshow( "Global Map", window );

//    cv::waitKey(1000);                                    // Wait for a keystroke in the window

//    if(ssdHeuristics.size()>0)
//        locTechnique = SSD;
//    if(densityHeuristic.size()>0)
//        locTechnique = DENSITY;
//    if(colorHeuristics.size()>0)
//        locTechnique = COLOR_ONLY;

//    Pose initialPose( 1324,486,DEG2RAD(20.0));

    // Initialize MCL
//    mcLocalization = new MCL(heuristics, cachedMaps, globalMaps, prevRawOdom);
    string oName;
    if(!outputName.empty())
        oName = outputName+to_string(current)+".txt";
    mcLocalization = new MCL(heuristics, cachedMaps, globalMaps, realPose, oName);

    step = 0;

    ready_ = true;
}

void DroneRobot::initializeFeatureMatching()
{
    cv::Mat globalMap = globalMaps[1]; // Grayscale

    int H = globalMap.rows;
    int W = globalMap.cols;
    cout << "H:" << H << " W:" << W << endl;

    feature_detector=cv::xfeatures2d::SIFT::create(100000,10);
//    feature_detector=cv::xfeatures2d::SURF::create();
    feature_extractor=feature_detector;
    feature_detector->detect(globalMap,keypoints_globalMap);
    feature_extractor->compute(globalMap,keypoints_globalMap,descriptors_globalMap);
    feature_matcher.add(descriptors_globalMap);

    int descriptorsType = descriptors_globalMap.type();

    cout << "NumKeyPoints " << keypoints_globalMap.size() <<
            " Descriptors " << descriptors_globalMap.cols << ' ' << descriptors_globalMap.rows << ' '
                            << Utils::opencvtype2str(descriptorsType) << endl;

    cv::Mat localMap = cv::imread(imagesNames[0],CV_LOAD_IMAGE_COLOR);
    if(! localMap.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find local map" << std::endl ;
        cout<<"Endereco:"<<imagesNames[0]<<endl;
        return;
    }
    int h = localMap.rows;
    int w = localMap.cols;
    int size = sqrt(h*h+w*w);
    int halfSize = size/2;

    cout << "h:" << h << " w:" << w << " size:" << size << endl;

    int numRows = H/halfSize;
    int numCols = W/halfSize;
    cout << "numRows:" << numRows << " numCols:" << numCols << endl;

    likelihood = cv::Mat(numRows,numCols,descriptorsType,1.0);

    // Allocate matrices
    hMatcher.resize(numCols);
    idKeypoints.resize(numCols);
    for(int x=0; x<hMatcher.size(); ++x){
        hMatcher[x].resize(numRows);
        idKeypoints[x].resize(numRows);
        for(int y=0; y<hMatcher[x].size(); ++y){
            hMatcher[x][y] = new cv::FlannBasedMatcher();
            idKeypoints[x][y] = new vector<unsigned int>();
        }
    }

    int sum=0;
    cout << "keyPoints " << endl;
    // Fill matrices
    for(int x=0; x<hMatcher.size(); ++x){
        for(int y=0; y<hMatcher[x].size(); ++y){
            cv::FlannBasedMatcher* matcher = hMatcher[x][y];
            vector<unsigned int>* kPoints = idKeypoints[x][y];

            if(x==0 && y==0)
                cout << matcher << endl;

            // Find features inside the (x,y) block
            for(int k=0; k<keypoints_globalMap.size(); ++k){
                cv::KeyPoint& kp = keypoints_globalMap[k];
                if(kp.pt.x >= x*halfSize &&
                   kp.pt.x < x*halfSize + size &&
                   kp.pt.y >= y*halfSize &&
                   kp.pt.y < y*halfSize + size)
                    kPoints->push_back(k);
            }

//            cout << "(" << x*halfSize << "-" << x*halfSize + halfSize << "; "
//                        << y*halfSize << "-" << y*halfSize + halfSize << ") ";
            cout << kPoints->size() << ' ';
            sum += kPoints->size();

            // Copy descriptors associated to kPoints
            cv::Mat descriptors(kPoints->size(),128,descriptorsType);
            for(int k=0; k<kPoints->size(); ++k){
                descriptors_globalMap.row(kPoints->at(k)).copyTo(descriptors.row(k));
            }

            matcher->add(descriptors);
        }
        cout << endl;
    }

//    cout << "TOTAL:" << sum << endl;
//    exit(0);

}

void computeEntropyMap(cv::Mat& image_orig, cv::Mat& mask)
{
    cv::Mat image;
    image = image_orig;
//    resize(image_orig,image,cv::Size(0,0),0.5,0.5);

    image.convertTo(image,CV_32F);
    image *= 1.0/255.0;
    cvtColor(image, image, CV_BGR2Lab);

//    cvtColor(image, image, CV_BGR2GRAY);
//    cvtColor(image, image, CV_GRAY2BGR);
    imshow("input",image);
//    cv::waitKey(0);

    cv::Mat entropyMap(image.rows, image.cols, CV_64F, cv::Scalar(0.0));

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

    cv::waitKey(0);
}

void DroneRobot::run()
{
    if(step >= imagesNames.size()){
        if(runQuiet){
            enableCout();
            cout << "\r" << step*100/imagesNames.size() << "%" << endl;
            disableCout();
        }

        if(current==finish)
            exit(0);
        else{
            current++;
            reinitialize();
        }
    }

    // Read image
    cv::Mat currentMap = cv::imread(imagesNames[step],CV_LOAD_IMAGE_COLOR);
    if(! currentMap.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find local map" << std::endl ;
        cout<<"Endereco:"<<imagesNames[step]<<endl;
        return;
    }

//    cv::Mat window;
//    resize(currentMap,window,cv::Size(0,0),0.2,0.2);

    if(!runQuiet){
        imshow( "Local Map", currentMap );
        cv::waitKey(100);
    }

    cout << "Image" << step << ' ';
    step += 1;

    if((slowMethod || step%5==0) && runQuiet){
        enableCout();
        cout << "\r" << step*100/imagesNames.size() << "%" << flush;
        disableCout();
    }

//    computeEntropyMap(currentMap, mask);
//    return;

    if(locTechnique == TEMPLATE_MATCHING){
        localizeWithTemplateMatching(currentMap);
        return;
    }else if(locTechnique == FEATURE_MATCHING){
        localizeWithHierarchicalFeatureMatching(currentMap);
//        localizeWithFeatureMatching(currentMap);
        return;
    }

    bool odom_reliable = true;

    // Compute or read Odometry
    if(step>1){
        if(offlineOdom){
            if(isRawOdom){
                odometry_ = readOdometry();
            }else{
                pair<Pose,bool> od = readOdometryNew();
                odometry_ = od.first;
                odom_reliable = od.second;
            }
//            cv::waitKey(0);
        }else{
//            odometry_ = findOdometry(prevMap,currentMap);

            pair<Pose,bool> od = findOdometryUsingCorrelativeSM(prevMap,currentMap);
//            pair<Pose,bool> od = findOdometry(prevMap,currentMap);
            odom_reliable = od.second;
            if(odom_reliable)
                odometry_ = od.first;
            else
                odometry_ = Pose(0,0,0.0);
//                odometry_ = prevOdometry;
            odomFile << odometry_.x << ' ' << odometry_.y << ' ' << odometry_.theta << ' ' << odom_reliable << endl;
        }
    }else{
        odometry_ = Pose(0,0,0.0);
    }
    cout << "Odometry: " << odometry_ << endl;
    prevOdometry = odometry_;

    prevMap = currentMap;

    if(availableGTruth){
        realPose = readGroundTruth();
    }else{
        realPose = prevRawOdom;
    }

    // Obtain
    double time=0;

    // Run Monte Carlo Localization
    mcLocalization->run(odometry_, odom_reliable, currentMap, time, realPose);
    if(availableGTruth){
        mcLocalization->writeErrorLogFile(realPose.x,realPose.y,realPose.theta);
    }

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

void DroneRobot::localizeWithTemplateMatching(cv::Mat& currentMap)
{
    cout << "Templatão" << endl;
    cv::Mat whiteMask(currentMap.rows, currentMap.cols, CV_8UC3, cv::Scalar(255,255,255));

    for(int angle=360; angle>=0; angle-=10){

        cv::Mat templ = Utils::rotateImage(currentMap,angle);
        cv::Mat mask = Utils::rotateImage(whiteMask,angle);
        imshow( "Local Map", templ );
        imshow( "Mask", mask );

        /// Source image to display
        cv::Mat img_display, result;
        globalMaps[0].copyTo( img_display );

        cv::Point matchLoc = Utils::templateMatching(globalMaps[0],templ,result,CV_TM_CCORR_NORMED,mask);

        /// Show me what you got
        rectangle( img_display, matchLoc, cv::Point( matchLoc.x + currentMap.cols , matchLoc.y + currentMap.rows ), cv::Scalar::all(0), 2, 8, 0 );
        rectangle( result, matchLoc, cv::Point( matchLoc.x + currentMap.cols , matchLoc.y + currentMap.rows ), cv::Scalar::all(0), 2, 8, 0 );

        resize(img_display,img_display,cv::Size(0,0),0.2,0.2);
        imshow( "image_window", img_display );
        resize(result,result,cv::Size(0,0),0.2,0.2);
        imshow( "result", result);
        cv::waitKey(0);
    }
}

void DroneRobot::localizeWithHierarchicalFeatureMatching(cv::Mat& currentMap)
{
    cv::Mat current;

    // Convert image to gray scale;
    cvtColor(currentMap, current, CV_BGR2GRAY);

    //-- Step 1 & 2: Detect the keypoints using Detector & Calculate descriptors (feature vectors)

    std::vector<cv::KeyPoint> keypoints_currentMap;
    cv::Mat descriptors_currentMap;

    feature_detector->detect(current,keypoints_currentMap);
    feature_extractor->compute(current,keypoints_currentMap,descriptors_currentMap);

    cv::Mat H;
    double sum=0;
    double maxMatches=-1;
    pair<int,int> best;
    std::vector< cv::DMatch > best_matches;
    cv::Mat curlikelihood = cv::Mat(likelihood.rows,likelihood.cols,likelihood.type());

    for(int x=0; x<hMatcher.size(); ++x){
        for(int y=0; y<hMatcher[x].size(); ++y){
            //-- Step 3: Matching descriptor vectors using FLANN matcher
            cv::FlannBasedMatcher* matcher = hMatcher[x][y];
            vector<unsigned int>* kPoints = idKeypoints[x][y];

            std::vector< cv::DMatch > matches;
            matcher->match( descriptors_currentMap, matches );

            double max_dist = 0; double min_dist = 100;

            //-- Quick calculation of max and min distances between keypoints
            for( int i = 0; i < matches.size(); i++ )
            {
                double dist = matches[i].distance;
                if( dist < min_dist )
                    min_dist = dist;
                if( dist > max_dist )
                    max_dist = dist;
            }
//            printf("-- Max dist : %f \n", max_dist );
//            printf("-- Min dist : %f \n", min_dist );

            //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
            //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
            //-- small)
            //-- PS.- radiusMatch can also be used here.
            std::vector< cv::DMatch > good_matches;
            for( int i = 0; i < matches.size(); i++ )
            {
                if( matches[i].distance <= max(2*min_dist, 0.02) ){
                    good_matches.push_back( matches[i]);
                }
            }

            curlikelihood.at<float>(y,x) = good_matches.size();
            sum += good_matches.size();
            if(int(good_matches.size()) > maxMatches){
                maxMatches = good_matches.size();
                best = pair<int,int>(x,y);
                best_matches = good_matches;
//                if(maxMatches > 10){
//                    // Find transformation
//                    std::vector<cv::Point2f> points1, points2;
//                    for( int i = 0; i < good_matches.size(); i++ )
//                    {
//                        //-- Get the keypoints from the good matches
//                        points1.push_back( keypoints_globalMap[ kPoints->at(good_matches[i].trainIdx) ].pt );
//                        points2.push_back( keypoints_currentMap[ good_matches[i].queryIdx ].pt );
//                    }

//                    H = findHomography( points1, points2, CV_RANSAC );
//                }
            }
        }
    }

    // Normalize likelihood
//    if(sum>0.0)
//        likelihood /= sum;
//    likelihood /= maxMatches;
//    curlikelihood /= maxMatches;

    normalize(curlikelihood,curlikelihood);
    cv::Mat noisinho = cv::Mat(likelihood.rows,likelihood.cols,likelihood.type(),0.1/float(likelihood.rows*likelihood.cols));

    likelihood = likelihood.mul(curlikelihood);
    likelihood += noisinho;
    normalize(likelihood,likelihood);


    cout << "Total: " << sum << " Max Num: " << maxMatches << endl;

    vector<unsigned int>* kPoints = idKeypoints[best.first][best.second];

    // Compute boundaries of the best association
    int minX, minY, maxX, maxY;
    minX = maxX = keypoints_globalMap[kPoints->at(best_matches[0].trainIdx)].pt.x;
    minY = maxY = keypoints_globalMap[kPoints->at(best_matches[0].trainIdx)].pt.y;
    for(int k=0; k<best_matches.size(); ++k)
    {
        int x=keypoints_globalMap[kPoints->at(best_matches[k].trainIdx)].pt.x;
        int y=keypoints_globalMap[kPoints->at(best_matches[k].trainIdx)].pt.y;
        if(x<minX) minX=x;
        else if(x>maxX) maxX=x;
        if(y<minY) minY=y;
        else if(y>maxY) maxY=y;
    }
//    cout << "min " << minX << "," << minY << " max " << maxX << "," << maxY << endl;
//    cout << "min " << minX << "," << minY << " delta " << maxX-minX << "," << maxY-minY << endl;

    // Draw bounding box of the best association
    cv::Mat gMap = globalMaps[0].clone();
    line( gMap, cv::Point2f(minX,minY), cv::Point2f(minX,maxY), cv::Scalar(0, 0, 255), 10 );
    line( gMap, cv::Point2f(minX,maxY), cv::Point2f(maxX,maxY), cv::Scalar(0, 0, 255), 10 );
    line( gMap, cv::Point2f(maxX,maxY), cv::Point2f(maxX,minY), cv::Scalar(0, 0, 255), 10 );
    line( gMap, cv::Point2f(maxX,minY), cv::Point2f(minX,minY), cv::Scalar(0, 0, 255), 10 );

    // Draw tranformed image
    if(!H.empty()){
        //-- Get the corners from currentMap
        std::vector<cv::Point2f> obj_corners(7);
        obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( currentMap.cols, 0 );
        obj_corners[2] = cvPoint( currentMap.cols, currentMap.rows ); obj_corners[3] = cvPoint( 0, currentMap.rows );

        obj_corners[4] = cvPoint( currentMap.cols/2, currentMap.rows/2 ); // center
        obj_corners[5] = cvPoint( currentMap.cols, currentMap.rows/2 ); obj_corners[6] = cvPoint( currentMap.cols/2, 0 );

        std::vector<cv::Point2f> scene_corners(7);

        //-- Transform the corners using the homography
        perspectiveTransform( obj_corners, scene_corners, H);

        cv::Mat img_matches = globalMaps[0].clone();
        //-- Draw lines between the corners
        line( gMap, scene_corners[0] + cv::Point2f( currentMap.cols, 0), scene_corners[1] + cv::Point2f( currentMap.cols, 0), cv::Scalar(0, 255, 0), 4 );
        line( gMap, scene_corners[1] + cv::Point2f( currentMap.cols, 0), scene_corners[2] + cv::Point2f( currentMap.cols, 0), cv::Scalar( 0, 255, 0), 4 );
        line( gMap, scene_corners[2] + cv::Point2f( currentMap.cols, 0), scene_corners[3] + cv::Point2f( currentMap.cols, 0), cv::Scalar( 0, 255, 0), 4 );
        line( gMap, scene_corners[3] + cv::Point2f( currentMap.cols, 0), scene_corners[0] + cv::Point2f( currentMap.cols, 0), cv::Scalar( 0, 255, 0), 4 );

        line( gMap, scene_corners[4] + cv::Point2f( currentMap.cols, 0), scene_corners[5] + cv::Point2f( currentMap.cols, 0), cv::Scalar( 0, 0, 255), 4 );
        line( gMap, scene_corners[4] + cv::Point2f( currentMap.cols, 0), scene_corners[6] + cv::Point2f( currentMap.cols, 0), cv::Scalar( 255, 0, 0), 4 );
//        resize(gMap,img_matches,cv::Size(0,0),0.2,0.2);
//        imshow("Matches", gMap);
    }

    //-- Draw only "good" matches
    vector<cv::KeyPoint> keypoints_1;
    for(int k=0; k<kPoints->size(); ++k)
        keypoints_1.push_back(keypoints_globalMap[kPoints->at(k)]);
    cv::Mat img_matches;
    drawMatches( currentMap, keypoints_currentMap, gMap, keypoints_1,
               best_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
               vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    resize(img_matches,img_matches,cv::Size(0,0),0.2,0.2);
    imshow("Matches", img_matches);

//    likelihood *= 0;
//    likelihood.at<float>(3,2) = 1.0;

    cv::Mat window;
    resize(likelihood,window,cv::Size(0,0),50,50,cv::INTER_NEAREST);
    imshow("likelihood",window);
    cv::waitKey(10);
}

void DroneRobot::localizeWithFeatureMatching(cv::Mat& currentMap)
{
    cv::Mat current;

    // Convert image to gray scale;
    cvtColor(currentMap, current, CV_BGR2GRAY);

    //-- Step 1 & 2: Detect the keypoints using Detector & Calculate descriptors (feature vectors)

    std::vector<cv::KeyPoint> keypoints_currentMap;
    cv::Mat descriptors_currentMap;

    feature_detector->detect(current,keypoints_currentMap);
    feature_extractor->compute(current,keypoints_currentMap,descriptors_currentMap);

    //-- Step 3: Matching descriptor vectors using FLANN matcher
//    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
//    matcher.match( descriptors_currentMap, descriptors_globalMap, matches );
    feature_matcher.match( descriptors_currentMap, matches );

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < matches.size(); i++ )
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
    std::vector< cv::DMatch > good_matches;

    for( int i = 0; i < matches.size(); i++ )
    {
        if( matches[i].distance <= max(2*min_dist, 0.02) ){
            good_matches.push_back( matches[i]);
        }
    }

    //-- Draw only "good" matches
    cv::Mat img_matches;
    drawMatches( currentMap, keypoints_currentMap, globalMaps[0], keypoints_globalMap,
               good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
               vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    for( int i = 0; i < (int)good_matches.size(); i++ )
        printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx );

    int minNumPoints = 4;

    // Find transformation
    std::vector<cv::Point2f> points1, points2;
    for( int i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        points1.push_back( keypoints_globalMap[ good_matches[i].trainIdx ].pt );
        points2.push_back( keypoints_currentMap[ good_matches[i].queryIdx ].pt );
    }

    cv::Mat H;
    if(good_matches.size()>=minNumPoints)
        H = findHomography( points1, points2, CV_RANSAC );

//    //-- Get the corners from the image_1 ( the object to be "detected" )
//    std::vector<cv::Point2f> obj_corners(4);
//    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_1.cols, 0 );
//    obj_corners[2] = cvPoint( img_1.cols, img_1.rows ); obj_corners[3] = cvPoint( 0, img_1.rows );
//    std::vector<cv::Point2f> scene_corners(4);

//    perspectiveTransform( obj_corners, scene_corners, H);

//    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
//    line( img_matches, scene_corners[0] + cv::Point2f( img_1.cols, 0), scene_corners[1] + cv::Point2f( img_1.cols, 0), cv::Scalar(0, 255, 0), 4 );
//    line( img_matches, scene_corners[1] + cv::Point2f( img_1.cols, 0), scene_corners[2] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 255, 0), 4 );
//    line( img_matches, scene_corners[2] + cv::Point2f( img_1.cols, 0), scene_corners[3] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 255, 0), 4 );
//    line( img_matches, scene_corners[3] + cv::Point2f( img_1.cols, 0), scene_corners[0] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 255, 0), 4 );

    //-- Show detected matches
    cv::Mat resized;
    resize(img_matches,resized,cv::Size(0,0),0.4,0.4);
    imshow( "Good Matches", resized );
//    if(good_matches.size()>=minNumPoints)
//    if(!H.empty())
//        drawMatchedImages(globalMaps[0],currentMap,H,cv::MOTION_HOMOGRAPHY);
//    else
        cv::waitKey(0);

}

pair<Pose,bool> DroneRobot::readOdometryNew()
{
    if(odomFile.peek() == fstream::traits_type::eof())
        return pair<Pose,bool>(Pose(),false);

    Pose odom;
    bool reliable;
    string tempStr;
    odomFile >> odom.x >> odom.y >> odom.theta >> reliable;
    getline(odomFile,tempStr);

    return pair<Pose,bool>(odom,reliable);
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
    cout << "RAW " << RAD2DEG(newRawOdom.theta)
         << " - "  << RAD2DEG(prevRawOdom.theta)
         << " = "  << RAD2DEG(odom.theta) << endl;

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
       p.theta -= 2*M_PI;
   while(p.theta < -M_PI)
       p.theta += 2*M_PI;

   return true;
}

Pose DroneRobot::readGroundTruth()
{
   if(truthFile.peek() == fstream::traits_type::eof()) // keep the last good pose
       return realPose;

   string tempStr;
   Pose p;
   truthFile >> p.x >> p.y >> p.theta;
   getline(truthFile,tempStr);

   p.theta = DEG2RAD(p.theta);
   while(p.theta > M_PI)
       p.theta -= 2*M_PI;
   while(p.theta < -M_PI)
       p.theta += 2*M_PI;


   return p;
}

void DroneRobot::drawMatchedImages(cv::Mat& prevImage, cv::Mat& curImage, const cv::Mat& wm, const int warp_mode)
{
    vector<cv::Point2f> yourPoints;
    yourPoints.push_back(cv::Point2f(0,0));
    yourPoints.push_back(cv::Point2f(curImage.cols,0));
    yourPoints.push_back(cv::Point2f(curImage.cols,curImage.rows));
    yourPoints.push_back(cv::Point2f(0,curImage.rows));
    yourPoints.push_back(cv::Point2f(curImage.cols/2,curImage.rows/2));
    yourPoints.push_back(cv::Point2f(curImage.cols,curImage.rows/2));
    yourPoints.push_back(cv::Point2f(curImage.cols/2,0));

    cv::Mat warp_matrix = wm.clone();

    vector<cv::Point2f> transformedPoints;
    transformedPoints.resize(yourPoints.size());

    cv::Mat transf;
//    if (warp_mode != cv::MOTION_HOMOGRAPHY){
        transf = cv::Mat::eye(3, 3, CV_32F);
        cv::Mat aux = transf.colRange(0,3).rowRange(0,2);
        warp_matrix.copyTo(aux);
//    }else{
//        transf = warp_matrix;
//    }

    perspectiveTransform(yourPoints, transformedPoints, transf.inv());

    vector<cv::Point2f> totalPoints;
    totalPoints.reserve( yourPoints.size() + transformedPoints.size() ); // preallocate memory
    totalPoints.insert( totalPoints.end(), yourPoints.begin(), yourPoints.end() );
    totalPoints.insert( totalPoints.end(), transformedPoints.begin(), transformedPoints.end() );

    cv::Rect r = boundingRect(totalPoints);

//    cout << "warp:" << warp_matrix << endl;
//    cout << "transf:" << transf << endl;
//    cout << "r:" << r.x << ' ' << r.y << endl;

    cv::Mat displ = cv::Mat::eye(3, 3, CV_32F);
    displ.at<float>(0,2) = -r.x;
    displ.at<float>(1,2) = -r.y;
    perspectiveTransform(yourPoints, yourPoints, displ);
    perspectiveTransform(transformedPoints, transformedPoints, displ);

    cv::Mat blank(r.height, r.width, CV_8UC3, cv::Scalar(0));
//    cv::waitKey(0);


    // Storage for warped image.
    cv::Mat im2_aligned;
    warp_matrix.at<float>(0,2) += r.x;
    warp_matrix.at<float>(1,2) += r.y;
//    cout << "warp2:" << warp_matrix << endl;

    if (warp_mode != cv::MOTION_HOMOGRAPHY)
        // Use warpAffine for Translation, Euclidean and Affine
        warpAffine(curImage, im2_aligned, warp_matrix, blank.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
    else
        // Use warpPerspective for Homography
        warpPerspective (curImage, im2_aligned, warp_matrix, blank.size(),cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);

    // Draw aligned curImage in blank
    blank += im2_aligned*0.5;

    // Draw prevImage in blank
    cv::Mat aux1 = blank.colRange(-r.x,-r.x+prevImage.cols).rowRange(-r.y,-r.y+prevImage.rows);
    cv::Mat im1 = prevImage*0.5 + aux1;
    im1.copyTo(aux1);

    // Draw rectangles
    for (int i = 0; i < 4; i++)
        line(blank, yourPoints[i], yourPoints[(i+1)%4], cv::Scalar(0,255,0));
    for (int i = 0; i < 4; i++)
        line(blank, transformedPoints[i], transformedPoints[(i+1)%4], cv::Scalar(0,0,255));

    // Draw arrow
    rectangle(blank,cv::Rect(yourPoints[4]-cv::Point2f(2.5,2.5), cv::Size2f(5,5)), cv::Scalar(0,255,0));
    circle(blank,transformedPoints[4],10,cv::Scalar(0,0,255));
    line(blank, yourPoints[4], transformedPoints[4], cv::Scalar(0,0,255));
    line(blank, transformedPoints[4], transformedPoints[5], cv::Scalar(255,255,255));
    line(blank, transformedPoints[4], transformedPoints[6], cv::Scalar(255,255,255));


    // Show final result
//    cv::namedWindow( "Image 1", cv::WINDOW_KEEPRATIO );
//    cv::namedWindow( "Image 2", cv::WINDOW_KEEPRATIO );
//    cv::namedWindow( "Image 2 Aligned", cv::WINDOW_KEEPRATIO );
//    cv::namedWindow( "Image Blank", cv::WINDOW_KEEPRATIO );
    imshow("Image Blank", blank);
//    imshow("Image 2 Aligned", im2_aligned);
//    cv::waitKey(0);
}

pair<Pose,bool> DroneRobot::findOdometryUsingECC(cv::Mat& prevImage, cv::Mat& curImage)
{
    cv::Mat im1_gray, im2_gray;

//    // Reduce size of images
//    resize(prevImage,im1_gray,cv::Size(0,0),0.2,0.2);
//    resize(curImage,im2_gray,cv::Size(0,0),0.2,0.2);

    // Convert images to gray scale;
    cvtColor(prevImage, im2_gray, CV_BGR2GRAY);
    cvtColor(curImage, im1_gray, CV_BGR2GRAY);

//    Canny(im1_gray,im1_gray,10,30,5);
//    Canny(im2_gray,im2_gray,10,30,5);

//    imshow("Image 1", im1_gray);
//    imshow("Image 2", im2_gray);

    // Define the motion model
    const int warp_mode = cv::MOTION_EUCLIDEAN;

    // Set a 2x3 or 3x3 warp matrix depending on the motion model.
    cv::Mat warp_matrix;

    // Initialize the matrix to identity
    if ( warp_mode == cv::MOTION_HOMOGRAPHY )
        warp_matrix = cv::Mat::eye(3, 3, CV_32F);
    else
        warp_matrix = cv::Mat::eye(2, 3, CV_32F);

    // Specify the number of iterations.
    int number_of_iterations = 1000;

    // Specify the threshold of the increment
    // in the correlation coefficient between two iterations
    double termination_eps = 1e-10;

    // Define termination criteria
    cv::TermCriteria criteria (cv::TermCriteria::COUNT+cv::TermCriteria::EPS, number_of_iterations, termination_eps);

    try
    {
    // Run the ECC algorithm. The results are stored in warp_matrix.
    findTransformECC(
                     im1_gray,
                     im2_gray,
                     warp_matrix,
                     warp_mode,
                     criteria
                 );
    }
    catch(cv::Exception e)
    {
        if (e.code == cv::Error::StsNoConv)
        {
            cout << "findTransformECC did not converge";
            return pair<Pose,bool>(odometry_,false);
        }
    }

    // Check quality of matching
    cv::Mat whiteMask(curImage.rows, curImage.cols, CV_8UC3, cv::Scalar(255,255,255));
    cv::Mat mask;
    warpAffine(whiteMask, mask, warp_matrix, whiteMask.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);

    cv::Mat templ(curImage.rows, curImage.cols, CV_8UC3, cv::Scalar(255,255,255));
    warpAffine(curImage, templ, warp_matrix, curImage.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
//    imshow("Mask", mask);
//    imshow("Aligned", templ);

    /// "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
    int match_method = CV_TM_CCORR_NORMED;

    cv::Mat result;
    result.create( 1, 1, CV_32FC1 );

    matchTemplate( prevImage, templ, result, match_method, mask );
    double w = result.at<float>(0,0);
    cout << "ODOMETRIA " << (w>0.96?"BOA":"RUIM") << " MANO (" << w << ")" << endl;

    if(!runQuiet)
        drawMatchedImages(curImage,prevImage,warp_matrix);

    char k = cv::waitKey(20);
    if (k=='g' || k=='G')
        cout << " GOOD odometry! :)" << endl;
    else if (k=='b' || k=='B')
        cout << " BAD odometry! :(" << endl;

    Pose p;
    p.x = warp_matrix.at<float>(0,2);
    p.y = warp_matrix.at<float>(1,2);
    p.theta = acos(warp_matrix.at<float>(0,0));

    return pair<Pose,bool>(p,true);
}

pair<Pose,bool> DroneRobot::findOdometry(cv::Mat& prevImage, cv::Mat& curImage)
{
    pair<Pose,bool> odom;
    odom.second = false;

    double cT=0.04;
//    while(odom.second==false && cT<0.06){
        odom = findOdometryUsingFeatures(prevImage, curImage, cT);
//        odom = findOdometryUsingICP(prevImage, curImage);
//        cT += 0.01;
//    }

//    cout << "cT " << cT-0.01 << endl;

    if(odom.second == false){
//        cv::waitKey(10);
//        cout << "ECC" << endl;
//        odom = findOdometryUsingECC(prevImage,curImage);
    }
    if(!runQuiet)
        cv::waitKey();
    return odom;
}

pair<Pose,bool> DroneRobot::findOdometryUsingICP(cv::Mat& prevImage, cv::Mat& curImage)
{
    cv::Mat edges_prev, edges_cur;

    // Convert images to gray scale;
    cvtColor(prevImage, edges_prev, CV_BGR2GRAY);
    cvtColor(curImage, edges_cur, CV_BGR2GRAY);

    blur( edges_prev, edges_prev, cv::Size(21,21) );
    blur( edges_prev, edges_prev, cv::Size(21,21) );
    blur( edges_cur, edges_cur, cv::Size(21,21) );
    blur( edges_cur, edges_cur, cv::Size(21,21) );

    cv::resize(edges_prev,edges_prev,cv::Size(0,0),0.5,0.5);
    cv::resize(edges_cur,edges_cur,cv::Size(0,0),0.5,0.5);

    cv::Canny(edges_cur,edges_cur,5000,5000,7);
    cv::Canny(edges_prev,edges_prev,5000,5000,7);
//    imshow("Canny",canny_img_2);
//    cv::waitKey(0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_prev (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cur (new pcl::PointCloud<pcl::PointXYZ>);

    int minX, minY, maxX, maxY;
    minX = minY = 100000000;
    maxX = maxY = -111111111;

    // Extract edge pixels
    for(int x=0;x<edges_prev.cols; ++x){
        for(int y=0;y<edges_prev.rows; ++y){
            float color = edges_prev.at<float>(y,x);
            if(color>0){
                pcl::PointXYZ p;
                p.x = x;
                p.y = y;
                p.z = 0;
                cloud_prev->points.push_back(p);

                if(x<minX) minX=x;
                if(x>maxX) maxX=x;
                if(y<minY) minY=y;
                if(y>maxY) maxY=y;
            }
        }
    }
    cloud_prev->width    = cloud_prev->points.size();
    cloud_prev->height   = 1;
    cloud_prev->is_dense = false;

    cout << "NUM " << cloud_prev->width
         << " minX:" << minX << " maxX:" << maxX
         << " minY:" << minY << " maxY:" << maxY << endl;

    minX = minY = 100000000;
    maxX = maxY = -111111111;
    for(int x=0;x<edges_cur.cols; ++x){
        for(int y=0;y<edges_cur.rows; ++y){
            float color = edges_cur.at<float>(y,x);
            if(color>0){
                pcl::PointXYZ p;
                p.x = x;
                p.y = y;
                p.z = 0;
                cloud_cur->points.push_back(p);

                if(x<minX) minX=x;
                if(x>maxX) maxX=x;
                if(y<minY) minY=y;
                if(y>maxY) maxY=y;
            }
        }
    }
    cloud_cur->width    = cloud_cur->points.size();
    cloud_cur->height   = 1;
    cloud_cur->is_dense = false;

    cout << "NUM " << cloud_prev->width
         << " minX:" << minX << " maxX:" << maxX
         << " minY:" << minY << " maxY:" << maxY << endl;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(100);
    //icp.setRANSACIterations(3);
//    icp.setRANSACOutlierRejectionThreshold(0.3);
//    icp.setMaxCorrespondenceDistance(100000.0);
    icp.setTransformationEpsilon(0.000001);
    icp.setEuclideanFitnessEpsilon(0.000001);

    cout << "Max correspondence Distance: " << icp.getMaxCorrespondenceDistance()
         << " Rejecttion Threshold: " << icp.getRANSACOutlierRejectionThreshold()
         << "EuclideanFitnessEpsilon: " << icp.getEuclideanFitnessEpsilon() << endl;
    //icp.setEuclideanFitnessEpsilon(0.00001);

    std::cout << "ICP:" << icp.getMaximumIterations() << ' ' << icp.getRANSACIterations() << endl;
    icp.setInputTarget(cloud_prev);
    icp.setInputCloud(cloud_cur);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    cv::Vec3b green(0,100,0);
    cv::Vec3b red(0,0,100);

    cv::Mat edges_final(edges_prev.rows,edges_prev.cols,CV_8UC3,cv::Scalar(0,0,0));

    edges_final=edges_prev;
    //    cvtColor(edges_final, edges_final, CV_GRAY2BGR);

    minX = minY = 100000000;
    maxX = maxY = -111111111;

    for(int p=0;p<Final.points.size(); ++p){
        int x=(Final.points[p].x>=floor(Final.points[p].x)+0.5?Final.points[p].x+1:Final.points[p].x);
        int y=(Final.points[p].y>=floor(Final.points[p].y)+0.5?Final.points[p].y+1:Final.points[p].y);
        if(x>=0 && y>=0 && x<edges_final.cols && y < edges_final.rows){
            edges_final.at<float>(y,x) = 0.5;
        }

        if(x<minX) minX=x;
        if(x>maxX) maxX=x;
        if(y<minY) minY=y;
        if(y>maxY) maxY=y;
    }
    cout << "NUM " << Final.width
         << " minX:" << minX << " maxX:" << maxX
         << " minY:" << minY << " maxY:" << maxY << endl;

//    minX = minY = 100000000;
//    maxX = maxY = -111111111;

//    for(int p=0;p<cloud_prev->points.size(); ++p){
//        int x=cloud_prev->points[p].x;
//        int y=cloud_prev->points[p].y;
//        if(x>=0 && y>=0 && x < edges_final.cols && y < edges_final.rows){
//            edges_final.at<cv::Vec3b>(y,x) = red;
//            edges_final.at<float>(y,x) = 0.5;

//        }

//        if(x<minX) minX=x;
//        if(x>maxX) maxX=x;
//        if(y<minY) minY=y;
//        if(y>maxY) maxY=y;
//    }

//    cout << "NUM " << cloud_prev->width
//         << " minX:" << minX << " maxX:" << maxX
//         << " minY:" << minY << " maxY:" << maxY << endl;

    imshow("ICP",edges_final);
    imshow("PREV",edges_prev);
    imshow("CUR",edges_cur);
    cv::waitKey(0);

    pair<Pose,bool> odom;
    odom.second = true;
//    return score;

}


pair<Pose, bool> DroneRobot::findOdometryUsingCorrelativeSM(cv::Mat& prevImage, cv::Mat& curImage){
    cv::Mat edges_prev, edges_cur;

    // Convert images to gray scale;
    cvtColor(prevImage, edges_prev, CV_BGR2GRAY);
    cvtColor(curImage, edges_cur, CV_BGR2GRAY);

    blur( edges_prev, edges_prev, cv::Size(21,21) );
    blur( edges_prev, edges_prev, cv::Size(21,21) );
    blur( edges_cur, edges_cur, cv::Size(21,21) );
    blur( edges_cur, edges_cur, cv::Size(21,21) );

    cv::resize(edges_prev,edges_prev,cv::Size(0,0),0.5,0.5);
    cv::resize(edges_cur,edges_cur,cv::Size(0,0),0.5,0.5);

    cv::Canny(edges_cur,edges_cur,5000,5000,7);
    cv::Canny(edges_prev,edges_prev,5000,5000,7);
//    imshow("Canny",canny_img_2);
//    cv::waitKey(0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_prev (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cur (new pcl::PointCloud<pcl::PointXYZ>);


    int minX, minY, maxX, maxY;
    minX = minY = 100000000;
    maxX = maxY = -111111111;

    // Extract edge pixels
    for(int x=0;x<edges_prev.cols; ++x){
        for(int y=0;y<edges_prev.rows; ++y){
            float color = edges_prev.at<float>(y,x);
            if(color>0){
                pcl::PointXYZ p;
                p.x = x;
                p.y = y;
                p.z = 0;
                cloud_prev->points.push_back(p);

                if(x<minX) minX=x;
                if(x>maxX) maxX=x;
                if(y<minY) minY=y;
                if(y>maxY) maxY=y;
            }
        }
    }
    cloud_prev->width    = cloud_prev->points.size();
    cloud_prev->height   = 1;
    cloud_prev->is_dense = false;

    cout << "NUM " << cloud_prev->width
         << " minX:" << minX << " maxX:" << maxX
         << " minY:" << minY << " maxY:" << maxY << endl;

    minX = minY = 100000000;
    maxX = maxY = -111111111;
    for(int x=0;x<edges_cur.cols; ++x){
        for(int y=0;y<edges_cur.rows; ++y){
            float color = edges_cur.at<float>(y,x);
            if(color>0){
                pcl::PointXYZ p;
                p.x = x;
                p.y = y;
                p.z = 0;
                cloud_cur->points.push_back(p);

                if(x<minX) minX=x;
                if(x>maxX) maxX=x;
                if(y<minY) minY=y;
                if(y>maxY) maxY=y;
            }
        }
    }
    cloud_cur->width    = cloud_cur->points.size();
    cloud_cur->height   = 1;
    cloud_cur->is_dense = false;

    cout << "NUM " << cloud_prev->width
         << " minX:" << minX << " maxX:" << maxX
         << " minY:" << minY << " maxY:" << maxY << endl;

    CorrelativeScanMatching csm(prevImage.rows, prevImage.cols);
    Pose outPose;
    Matrix3d outCov;
    csm.compute(cloud_prev, cloud_cur, outPose, outCov);

    cout<<"X: "<<outPose.x<<" Y: "<<outPose.y<<"Th: "<< outPose.theta << endl;
    pair<Pose,bool> p;
    p.first = outPose;
    p.second = true;
    return p;

}


pair<Pose,bool> DroneRobot::findOdometryUsingFeatures(cv::Mat& prevImage, cv::Mat& curImage, double cT)
{
    cv::Mat img_1, img_2;

    // Convert images to gray scale;
    cvtColor(prevImage, img_2, CV_BGR2GRAY);
    cvtColor(curImage, img_1, CV_BGR2GRAY);

//    blur( img_2, img_2, cv::Size(21,21) );
//    blur( img_2, img_2, cv::Size(21,21) );


//    cv::Mat canny_img_2;
//    cv::Canny(img_2,canny_img_2,5000,5000,7);
//    imshow("Canny",canny_img_2);
//    cv::waitKey(0);

    //-- Step 1 & 2: Detect the keypoints using Detector & Calculate descriptors (feature vectors)

    cv::Ptr<cv::Feature2D> detector=cv::xfeatures2d::SURF::create();                    //create(20000,8,cT);
    cv::Ptr<cv::Feature2D> extractor=cv::xfeatures2d::SURF::create();        //create(20000,8,cT);

    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;

//    detector->detectAndCompute(img_1,cv::noArray(),keypoints_1,descriptors_1);
//    detector->detectAndCompute(img_2,cv::noArray(),keypoints_2,descriptors_2);

    detector->detect(img_1,keypoints_1);
    extractor->compute(img_1,keypoints_1,descriptors_1);

    detector->detect(img_2,keypoints_2);
    extractor->compute(img_2,keypoints_2,descriptors_2);

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < matches.size(); i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist )
            min_dist = dist;
        if( dist > max_dist )
            max_dist = dist;
    }

    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //-- small)
    //-- PS.- radiusMatch can also be used here.
    std::vector< cv::DMatch > good_matches;

    for( int i = 0; i < matches.size(); i++ )
    {
        if( matches[i].distance <= max(2*min_dist, 0.02) ){
            good_matches.push_back( matches[i]);
        }
    }

    //-- Draw only "good" matches
    cv::Mat img_matches;
    drawMatches( img_1, keypoints_1, img_2, keypoints_2,
               good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
               vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

//    for( int i = 0; i < (int)good_matches.size(); i++ )
//        printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx );

    int minNumPoints = 6;

    // Find transformation
    std::vector<cv::Point2f> points1, points2;
    for( int i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        points1.push_back( keypoints_1[ good_matches[i].queryIdx ].pt );
        points2.push_back( keypoints_2[ good_matches[i].trainIdx ].pt );
    }

    cv::Mat H;
    if(good_matches.size()>=minNumPoints)
        H = findHomography( points1, points2, CV_RANSAC );


    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<cv::Point2f> obj_corners(7);
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_1.cols, 0 );
    obj_corners[2] = cvPoint( img_1.cols, img_1.rows ); obj_corners[3] = cvPoint( 0, img_1.rows );

    obj_corners[4] = cvPoint( img_1.cols/2, img_1.rows/2 ); // center
    obj_corners[5] = cvPoint( img_1.cols, img_1.rows/2 ); obj_corners[6] = cvPoint( img_1.cols/2, 0 );

    std::vector<cv::Point2f> scene_corners(7);

    if(!H.empty()){
        perspectiveTransform( obj_corners, scene_corners, H);

        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        line( img_matches, scene_corners[0] + cv::Point2f( img_1.cols, 0), scene_corners[1] + cv::Point2f( img_1.cols, 0), cv::Scalar(0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + cv::Point2f( img_1.cols, 0), scene_corners[2] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + cv::Point2f( img_1.cols, 0), scene_corners[3] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + cv::Point2f( img_1.cols, 0), scene_corners[0] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 255, 0), 4 );

        line( img_matches, scene_corners[4] + cv::Point2f( img_1.cols, 0), scene_corners[5] + cv::Point2f( img_1.cols, 0), cv::Scalar( 0, 0, 255), 4 );
        line( img_matches, scene_corners[4] + cv::Point2f( img_1.cols, 0), scene_corners[6] + cv::Point2f( img_1.cols, 0), cv::Scalar( 255, 0, 0), 4 );
    }

    Pose p;
    bool isGood = false;

//    if(good_matches.size()>=minNumPoints)
    if(!H.empty()){
        H = H.inv();
        if(!runQuiet)
            drawMatchedImages(prevImage,curImage,H,cv::MOTION_HOMOGRAPHY);

        // Compute approximate translation and rotation
        p.y = scene_corners[4].x - obj_corners[4].x;
        p.x = -(scene_corners[4].y - obj_corners[4].y);
        p.theta = atan2(scene_corners[5].y-scene_corners[4].y,scene_corners[5].x-scene_corners[4].x);
    //    p.theta += M_PI/2.0;
        isGood = true;

        // Check if it is a good transformation
        // 1st - the displacement must be smaller than half diagonal of the image (times 0.8)
        double origDiag = Utils::getNorm(obj_corners[1]-obj_corners[3]);
        double ratioDisp = Utils::getNorm(scene_corners[4]-obj_corners[4])/(origDiag*0.5);
        if(ratioDisp > 0.8){
            cout << "OPS - displacement larger than half diagonal of the image (" << ratioDisp << ")" << endl;
            isGood = false;
        }
        // 2nd - diag sizes must be similar between themselves and similar to the original image
        double diag1 = Utils::getNorm(scene_corners[1]-scene_corners[3]);
        double diag2 = Utils::getNorm(scene_corners[0]-scene_corners[2]);
        double ratioModDiag, ratioDiag;
        if(diag1>diag2){
            ratioModDiag = diag1/diag2;
            ratioDiag = diag1/origDiag;
        }else{
            ratioModDiag = diag2/diag1;
            ratioDiag = diag2/origDiag;
        }
        if(ratioModDiag > 1.4 || fabs(ratioDiag-1.0) > 0.4){
            cout << "OPS - diag sizes not similar between themselves (" << ratioModDiag
                 <<  ") or to the original image (" << ratioDiag << ")" << endl;
            isGood = false;
        }
        // 3rd - the angle between the axes must be around 90 degrees
        double angle = Utils::getDiffAngle(scene_corners[5]-scene_corners[4], scene_corners[6]-scene_corners[4]);
        if(fabs(angle - 90.0) > 25.0){
            cout << "OPS - angle between the axes different than 90° (" << angle << ")" << endl;
            isGood = false;
        }
        // 4th - the ratio between oposed sides of the projected image must be around 1.0
        double a = Utils::getNorm(scene_corners[1]-scene_corners[0]);
        double b = Utils::getNorm(scene_corners[2]-scene_corners[1]);
        double c = Utils::getNorm(scene_corners[3]-scene_corners[2]);
        double d = Utils::getNorm(scene_corners[0]-scene_corners[3]);
        double ratioX = std::max(a/c,c/a);
        double ratioY = std::max(b/d,d/b);
        if(ratioX > 1.8 || ratioY > 1.8){
            cout << "OPS - ratio between oposed sides of the projected image not around 1.0 (" << ratioX << "," << ratioY << ")" << endl;
            isGood = false;
        }
        cout << "1st " << ratioDisp << " 2nd " << ratioModDiag << ' ' << ratioDiag << " 3rd " << angle << " 4th " << ratioX << ' ' << ratioY << endl;
    }

    //-- Show detected matches
    if(!runQuiet)
        imshow( "Good Matches", img_matches );

    return pair<Pose,bool>(p,isGood);
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
