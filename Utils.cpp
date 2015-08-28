#include "Utils.h"

#include <sstream>
#include <iomanip>
#include <errno.h>
#include <string.h>
#include <cmath>

double Utils::getDiffAngle(double ang1, double ang2)
{
    // Compute diff between angles in -PI --> PI
    double dif1 =  ang1 - ang2;
    while(dif1 > M_PI)
        dif1 -= M_PI;
    while(dif1 < -M_PI)
        dif1 += M_PI;

    // Compute diff between angles in 0 --> 2*PI
    double new360 = (ang1<0?ang1+2*M_PI:ang1);
    double prev360 = (ang2<0?ang2+2*M_PI:ang2);
    double dif2 = new360 - prev360;
    while(dif2 > M_PI)
        dif2 -= M_PI;
    while(dif2 < -M_PI)
        dif2 += M_PI;

    if(fabs(dif1) < fabs(dif2))
        return dif1;
    else
        return dif2;
}

Mat Utils::rotateImage(Mat& input, double angle)
{
    Point2f origCenter(input.cols/2,input.rows/2);
    RotatedRect rRect = RotatedRect(origCenter, input.size(), angle);
    Rect bRect = rRect.boundingRect();

    vector<Point2f> boundaries;
    boundaries.push_back(Point2f(bRect.x,bRect.y));
    boundaries.push_back(Point2f(bRect.x+bRect.width,bRect.y+bRect.height));
    boundaries.push_back(Point2f(0,0));
    boundaries.push_back(Point2f(input.cols,input.rows));
    bRect = boundingRect(boundaries);

    Mat source(bRect.height, bRect.width, CV_8UC3, Scalar(0));
    Point2f center(source.cols/2, source.rows/2);

    Mat aux = source.colRange(center.x - origCenter.x, center.x + origCenter.x).rowRange(center.y - origCenter.y, center.y + origCenter.y);
    input.copyTo(aux);

    // get the rotation matrix
    Mat M = getRotationMatrix2D(center, angle, 1.0);

    // perform the affine transformation
    Mat rotated;
    warpAffine(source, rotated, M, source.size(), INTER_CUBIC);

    return rotated;
}

Mat Utils::getRotatedROIFromImage(Pose p, Size2f s, Mat& largeMap)
{
    Mat& globalMap = largeMap;

    RotatedRect rRect = RotatedRect(Point2f(p.x,p.y), s, RAD2DEG(p.theta)+90.0);
    Rect bRect = rRect.boundingRect();

    vector<Point2f> boundaries;
    boundaries.push_back(Point2f(bRect.x,bRect.y));
    boundaries.push_back(Point2f(bRect.x+bRect.width,bRect.y+bRect.height));
    boundaries.push_back(Point2f(p.x-s.width/2,p.y-s.height/2));
    boundaries.push_back(Point2f(p.x+s.width/2,p.y+s.height/2));
    bRect = boundingRect(boundaries);

    Mat source(bRect.height, bRect.width, CV_8UC3, Scalar(0));
    Point2f center(source.cols/2, source.rows/2);

    int xi=0;
    int xf=source.cols;
    int yi=0;
    int yf=source.rows;

    // Correct brect
    if(bRect.x < 0){
        bRect.width += bRect.x;
        bRect.x = 0;
        xi = source.cols - bRect.width;
    }else if(bRect.x+bRect.width > globalMap.cols){
        bRect.width = globalMap.cols - bRect.x;
        xf = bRect.width;
    }
    if(bRect.y < 0){
        bRect.height += bRect.y;
        bRect.y = 0;
        yi = source.rows - bRect.height;
    }else if(bRect.y+bRect.height > globalMap.rows){
        bRect.height = globalMap.rows - bRect.y;
        yf = bRect.height;
    }

    Mat image_roi = globalMap(bRect).clone();

    Mat aux = source.colRange(xi,xf).rowRange(yi,yf);
    image_roi.copyTo(aux);

    // get angle and size from the bounding box
    float angle = rRect.angle;
    Size rect_size = rRect.size;
//    if (rRect.angle < -45.) {
//        angle += 90.0;
//        swap(rect_size.width, rect_size.height);
//    }

    // get the rotation matrix
    Mat M = getRotationMatrix2D(center, angle, 1.0);

    // perform the affine transformation
    Mat rotated;
    warpAffine(source, rotated, M, source.size(), INTER_CUBIC);

    // crop the resulting image
    Mat cropped;
    getRectSubPix(rotated, rect_size, center, cropped);

//    if (rRect.angle < -45.) {
//        transpose(cropped,cropped);
//        flip(cropped,cropped,0);
//    }


////    imshow("image_roi", image_roi);
//    imshow("rotated", rotated);
////    imshow("cropped", cropped);

//    Point2f start(bRect.x,bRect.y);
//    Point2f vertices[4];
//    rRect.points(vertices);
//    for (int i = 0; i < 4; i++)
//        line(image_roi, vertices[i]-start, vertices[(i+1)%4]-start, Scalar(0,255,0));
//    line(image_roi, rRect.center-start, (vertices[1]+vertices[2])/2-start, Scalar(0,255,0));

//    Rect a(Point2f(p.x,p.y)-Point2f(s.width/2,s.height/2)-start, s);
//    rectangle(image_roi, a, Scalar(255,0,0));
//    imshow("rectangles", image_roi);
//    waitKey(0);

    return cropped;
}

Point Utils::templateMatching(Mat& image, Mat& templ, Mat& result, int match_method, InputArray &templ_mask)
{
    /// Create the result matrix
    int result_cols =  image.cols - templ.cols + 1;
    int result_rows = image.rows - templ.rows + 1;

    /// match_method
    /// CV_TM_SQDIFF = 0,
    /// CV_TM_SQDIFF_NORMED = 1,
    /// CV_TM_CCORR = 2,
    /// CV_TM_CCORR_NORMED = 3,
    /// CV_TM_CCOEFF = 4,
    /// CV_TM_CCOEFF_NORMED = 5

    result = Mat( result_rows, result_cols, CV_32FC1 );

    matchTemplate( image, templ, result, match_method, templ_mask );

    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;

    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
      { matchLoc = minLoc; }
    else
      { matchLoc = maxLoc; }

//    matchLoc.x += templ.cols - 1;
//    matchLoc.y += templ.rows - 1;

    return matchLoc;
}

double Utils::matchImages(Mat& im_1, Mat& im_2, int match_method, InputArray &im2_mask)
{
    Mat result;

    Point p = templateMatching(im_1,im_2,result,match_method,im2_mask);
    return result.at<double>(p);
}

/////////////////////////////////
///// METHODS OF CLASS POSE /////
/////////////////////////////////

Pose::Pose(){
    x=y=theta=0.0;
    up=false;
}

Pose::Pose(double a, double b, bool c){
    x=a; y=b; theta=0.0; up=c;
}

ostream& operator<<(ostream& os, const Pose& p)
{
    os << "(" << p.x << ',' << p.y << ',' << p.theta << ")";
    return os;
}

////////////////////////////////////
///// METHODS OF CLASS LOGFILE /////
////////////////////////////////////

LogFile::LogFile(LogMode mode, string name)
{
    time_t t = time(0);
    struct tm *now = localtime(&t);
    stringstream ss;

    if(mode == RECORDING)
    {
        ss << "../PhiR2Framework/Sensors/sensors-" << -100+now->tm_year
                        << setfill('0') << setw(2) << 1+now->tm_mon
                        << setfill('0') << setw(2) << now->tm_mday << '-'
                        << setfill('0') << setw(2) << now->tm_hour
                        << setfill('0') << setw(2) << now->tm_min
                        << setfill('0') << setw(2) << now->tm_sec << ".txt";
        filename = ss.str();

        file.open(filename.c_str(), std::fstream::out);
    }
    else if(mode == PLAYBACK)
    {
        filename = "../PhiR2Framework/Sensors/"+name;
        cout << filename << endl;
        file.open(filename.c_str(), std::fstream::in);
        if(file.fail()){
            cerr << "Error: " << strerror(errno) << endl;
            exit(1);
        }
    }
}

Pose LogFile::readPose(string info)
{
    string tempStr;
    Pose p;

    file >> tempStr >> p.x >> p.y >> p.theta;
    getline(file,tempStr);

    return p;
}

vector<float> LogFile::readSensors(string info)
{
    int max;
    string tempStr;
    vector<float> sensors;

    file >> tempStr >> max;
    sensors.resize(max);
    for (int i = 0; i < max; i++) {
        file >> sensors[i];
    }
    getline(file,tempStr);

    return sensors;
}

void LogFile::writePose(string info, Pose pose)
{
    file << info << ' ' << pose.x << ' ' << pose.y << ' ' << pose.theta << endl;
}

void LogFile::writeSensors(string info, vector<float> sensors)
{
    file << info << ' ' << sensors.size() << ' ';
    for (int i = 0; i < sensors.size(); i++)
        file << sensors[i] << ' ';
    file << endl;
}

bool LogFile::hasEnded()
{
    return file.peek() == fstream::traits_type::eof();
}

//////////////////////////////////
///// METHODS OF CLASS TIMER /////
//////////////////////////////////

Timer::Timer()
{
    startCounting();
}

void Timer::startCounting()
{
    gettimeofday(&tstart, NULL);
    gettimeofday(&tlapstart, NULL);
}

void Timer::startLap()
{
    gettimeofday(&tlapstart, NULL);
}

void Timer::stopCounting()
{
    gettimeofday(&tnow, NULL);
}

float Timer::getTotalTime()
{
    gettimeofday(&tnow, NULL);

    if (tstart.tv_usec > tnow.tv_usec) {
        tnow.tv_usec += 1000000;
        tnow.tv_sec--;
    }

    return (float)(tnow.tv_sec - tstart.tv_sec) +
           ((float)tnow.tv_usec - (float)tstart.tv_usec)/1000000.0;
}

float Timer::getLapTime()
{
    gettimeofday(&tnow, NULL);

    if (tlapstart.tv_usec > tnow.tv_usec) {
        tnow.tv_usec += 1000000;
        tnow.tv_sec--;
    }
    return (float)(tnow.tv_sec - tlapstart.tv_sec) +
           ((float)tnow.tv_usec - (float)tlapstart.tv_usec)/1000000.0;
}


