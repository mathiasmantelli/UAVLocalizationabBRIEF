#include "Utils.h"

#include <sstream>
#include <iomanip>
#include <errno.h>
#include <string.h>
#include <cmath>
#include <dirent.h>
#include <algorithm>
#include <GL/glut.h>

vector<string> Utils::getListOfFiles(string dirname){
    vector<string> files;
    DIR* dirp = opendir(dirname.c_str());
    if(dirp==NULL)
        return files;
    dirent* dp;
    while ((dp = readdir(dirp)) != NULL){
        string f = dp->d_name;
        if(f.compare(".") != 0 && f.compare("..")  != 0)
            files.push_back(dp->d_name);
    }
    return files;
}

string Utils::opencvtype2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

double Utils::getNorm(cv::Point2f p)
{
    return sqrt(p.x*p.x + p.y*p.y);
}

double Utils::getDiffAngle(cv::Point2f p1, cv::Point2f p2)
{
    return RAD2DEG(acos( (p1.x*p2.x + p1.y*p2.y) / (sqrt(p1.x*p1.x + p1.y*p1.y)*sqrt(p2.x*p2.x + p2.y*p2.y)) ));
}

double Utils::getDiffAngle(double ang1, double ang2)
{
    // Compute diff between angles in -PI --> PI
    double dif1 =  ang1 - ang2;
    while(dif1 > M_PI)
        dif1 -= 2*M_PI;
    while(dif1 < -M_PI)
        dif1 += 2*M_PI;

    // Compute diff between angles in 0 --> 2*PI
    double new360 = (ang1<0?ang1+2*M_PI:ang1);
    double prev360 = (ang2<0?ang2+2*M_PI:ang2);
    double dif2 = new360 - prev360;
    while(dif2 > M_PI)
        dif2 -= 2*M_PI;
    while(dif2 < -M_PI)
        dif2 += 2*M_PI;

    if(fabs(dif1) < fabs(dif2))
        return dif1;
    else
        return dif2;
}

cv::Mat Utils::rotateImage(cv::Mat& input, double angle)
{
    cv::Point2f origCenter(input.cols/2,input.rows/2);
    cv::RotatedRect rRect = cv::RotatedRect(origCenter, input.size(), angle);
    cv::Rect bRect = rRect.boundingRect();

    vector<cv::Point2f> boundaries;
    boundaries.push_back(cv::Point2f(bRect.x,bRect.y));
    boundaries.push_back(cv::Point2f(bRect.x+bRect.width,bRect.y+bRect.height));
    boundaries.push_back(cv::Point2f(0,0));
    boundaries.push_back(cv::Point2f(input.cols,input.rows));
    bRect = boundingRect(boundaries);

    cv::Mat source(bRect.height, bRect.width, CV_8UC3, cv::Scalar(0));
    cv::Point2f center(source.cols/2, source.rows/2);

    cv::Mat aux = source.colRange(center.x - origCenter.x, center.x + origCenter.x).rowRange(center.y - origCenter.y, center.y + origCenter.y);
    input.copyTo(aux);

    // get the rotation matrix
    cv::Mat M = getRotationMatrix2D(center, angle, 1.0);

    // perform the affine transformation
    cv::Mat rotated;
    warpAffine(source, rotated, M, source.size(), cv::INTER_CUBIC);

    return rotated;
}

cv::Mat Utils::getRotatedROIFromImage(Pose p, cv::Size2f s, cv::Mat& largeMap)
{
    cv::Mat& globalMap = largeMap;

    cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(p.x,p.y), s, RAD2DEG(p.theta)+90.0);
    cv::Rect bRect = rRect.boundingRect();

    vector<cv::Point2f> boundaries;
    boundaries.push_back(cv::Point2f(bRect.x,bRect.y));
    boundaries.push_back(cv::Point2f(bRect.x+bRect.width,bRect.y+bRect.height));
    boundaries.push_back(cv::Point2f(p.x-s.width/2,p.y-s.height/2));
    boundaries.push_back(cv::Point2f(p.x+s.width/2,p.y+s.height/2));
    bRect = boundingRect(boundaries);

    cv::Mat source(bRect.height, bRect.width, CV_8UC3, cv::Scalar(0));
    cv::Point2f center(source.cols/2, source.rows/2);

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

    cv::Mat image_roi = globalMap(bRect).clone();

    cv::Mat aux = source.colRange(xi,xf).rowRange(yi,yf);
    image_roi.copyTo(aux);

    // get angle and size from the bounding box
    float angle = rRect.angle;
    cv::Size rect_size = rRect.size;
//    if (rRect.angle < -45.) {
//        angle += 90.0;
//        swap(rect_size.width, rect_size.height);
//    }

    // get the rotation matrix
    cv::Mat M = getRotationMatrix2D(center, angle, 1.0);

    // perform the affine transformation
    cv::Mat rotated;
    cv::warpAffine(source, rotated, M, source.size(), cv::INTER_CUBIC);

    // crop the resulting image
    cv::Mat cropped;
    cv::getRectSubPix(rotated, rect_size, center, cropped);

//    if (rRect.angle < -45.) {
//        transpose(cropped,cropped);
//        flip(cropped,cropped,0);
//    }


////    imshow("image_roi", image_roi);
//    imshow("rotated", rotated);
////    imshow("cropped", cropped);

//    cv::Point2f start(bRect.x,bRect.y);
//    cv::Point2f vertices[4];
//    rRect.points(vertices);
//    for (int i = 0; i < 4; i++)
//        line(image_roi, vertices[i]-start, vertices[(i+1)%4]-start, cv::Scalar(0,255,0));
//    line(image_roi, rRect.center-start, (vertices[1]+vertices[2])/2-start, cv::Scalar(0,255,0));

//    cv::Rect a(cv::Point2f(p.x,p.y)-cv::Point2f(s.width/2,s.height/2)-start, s);
//    rectangle(image_roi, a, cv::Scalar(255,0,0));
//    imshow("rectangles", image_roi);
//    cv::waitKey(0);

    return cropped;
}

cv::Point Utils::templateMatching(cv::Mat& image, cv::Mat& templ, cv::Mat& result, int match_method, cv::InputArray &templ_mask)
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

    result = cv::Mat( result_rows, result_cols, CV_32FC1 );

    matchTemplate( image, templ, result, match_method, templ_mask );

    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
    cv::Point matchLoc;

    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
      { matchLoc = minLoc; }
    else
      { matchLoc = maxLoc; }

//    matchLoc.x += templ.cols - 1;
//    matchLoc.y += templ.rows - 1;

    return matchLoc;
}

double Utils::matchImages(cv::Mat& im_1, cv::Mat& im_2, int match_method, cv::InputArray &im2_mask)
{
    cv::Mat result;

    cv::Point p = templateMatching(im_1,im_2,result,match_method,im2_mask);
    return result.at<double>(p);
}


/////////////////////////////////////////
///// METHODS OF CLASS TRIGONOMETRY /////
/////////////////////////////////////////

Trigonometry::Trigonometry(double res):
resolution(res)
{
    double numValues = 90.0/resolution;
    values.resize(int(numValues)+1);

    double step = DEG2RAD(resolution);

    for(double i=0.0; i<=numValues; i+=1.0){
        values[i] = std::sin(i*step);
    }
}

double Trigonometry::degSin(double a)
{
    while(a > 180.0)
        a -= 360.0;
    while(a <= -180.0)
        a += 360.0;

    //    sin(x) = sin(pi - x), to map from quadrant II to I
    //    sin(x) = - sin (pi + x), to map from quadrant III to I
    //    sin(-x) = - sin(x), to map from quadrant IV to I

    if(a >= 0.0){
        if( a <= 90.0){
            // quadrant I [0,90]
            return values[a/resolution];
        }else{
            // quadrant II (90,180]
            return values[(180.0-a)/resolution];
        }
    }else{
        if(a < -90.0){
            // quadrant III (-180,-90)
            return -values[(180.0+a)/resolution];
        }else{
            // quadrant IV [-90,0)
            return -values[(-a)/resolution];
        }
    }
}

double Trigonometry::degCos(double a)
{
    return Trigonometry::degSin(a+90.0);
}

Pose Trigonometry::getRelativePose(const Pose& p1, const Pose& p2)
{
    double cosine = cos(DEG2RAD(p1.theta));
    double sine = sin(DEG2RAD(p1.theta));
    Pose rel;
    rel.x = cosine*(p2.x-p1.x) - sine*(p2.y-p1.y);
    rel.y = sine*(p2.x-p1.x) + cosine*(p2.y-p1.y);
    rel.theta = p2.theta - p1.theta;
    return rel;
}

Pose Trigonometry::addRelativePose(const Pose& p1, const Pose& rel)
{
    double cosine = cos(DEG2RAD(p1.theta));
    double sine = sin(DEG2RAD(p1.theta));
    Pose pos;
    pos.x = p1.x + cosine*(rel.x) - sine*(rel.y);
    pos.y = p1.y + sine*(rel.x) + cosine*(rel.y);
    pos.theta = p1.theta + rel.theta;
    return pos;
}

void Trigonometry::checkValues()
{
    cout << "Start checking" << endl;
    for(double a=0.0; a<=360.0; a += resolution)
    {
        if(fabs(std::sin(DEG2RAD(a))-Trigonometry::degSin(a)) > 1e-2)
            cout << "A:" << a << " sin " << std::sin(DEG2RAD(a)) << ' ' << Trigonometry::degSin(a) << endl;
        if(fabs(std::cos(DEG2RAD(a))-Trigonometry::degCos(a)) > 1e-2)
            cout << "A:" << a << " cos " << std::cos(DEG2RAD(a)) << ' ' << Trigonometry::degCos(a) << endl;
    }
    cout << "Complete" << endl;
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

Pose& Pose::operator+=(const Pose& p)
{
    this->x += p.x;
    this->y += p.y;
    this->theta += p.theta;
    while(this->theta > 180.0)
        this->theta -= 360.0;
    while(this->theta < -180.0)
        this->theta += 360.0;
    return *this;
}

Pose Pose::operator+(const Pose& p)
{
    Pose pose = *this;
    pose += p;
    return pose;
}

Pose& Pose::operator-=(const Pose& p)
{
    this->x -= p.x;
    this->y -= p.y;
    this->theta -= p.theta;
    while(this->theta > 180.0)
        this->theta -= 360.0;
    while(this->theta < -180.0)
        this->theta += 360.0;
    return *this;
}

Pose Pose::operator-(const Pose& p)
{
    Pose pose = *this;
    pose -= p;
    return pose;
}

Pose::Pose(double a, double b, double c){
    x=a; y=b; theta=c; up=false;
}

ostream& operator<<(ostream& os, const Pose& p)
{
    os << "(" << p.x << ',' << p.y << ',' << p.theta << ")";
    return os;
}

///////////////////////////////////
///// METHODS OF CLASS POSE3D /////
///////////////////////////////////

Pose3d::Pose3d()
{
    x=y=z=roll=pitch=yaw=0.0;
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

///////////////////////////////////
///// METHODS OF CLASS COLORS /////
///////////////////////////////////

void Colors::setHSVColor(double i)
{
    i = i*2.0 - 1.0; // to let between [-1,1]
    glColor3d(red(i),green(i),blue(i));
}

void Colors::setColor(int i)
{
    int id = i%16;
//    cout << "COLOR:" << id << endl;

    switch(id){
        case 0:
            glColor3ub(0,0,0); // Black
            break;
        case 1:
            glColor3ub(225,225,225); // White
            break;
        case 2:
            glColor3ub(255,0,0); // Red
            break;
        case 3:
            glColor3ub(0,255,0); // Lime
            break;
        case 4:
            glColor3ub(0,0,255); // Blue
            break;
        case 5:
            glColor3ub(255,255,0); // Yellow
            break;
        case 6:
            glColor3ub(0,255,255); // Cyan
            break;
        case 7:
            glColor3ub(255,0,255); // Magenta
            break;
        case 8:
            glColor3ub(192,192,192); // Silver
            break;
        case 9:
            glColor3ub(128,128,128); // Gray
            break;
        case 10:
            glColor3ub(128,0,0); // Maroon
            break;
        case 11:
            glColor3ub(128,128,0); // Olive
            break;
        case 12:
            glColor3ub(0,128,0); // Green
            break;
        case 13:
            glColor3ub(128,0,128); // Purple
            break;
        case 14:
            glColor3ub(0,128,128); // Teal
            break;
        case 15:
            glColor3ub(0,0,128); // Navy
            break;
    }
}

double Colors::interpolate( double val, double y0, double x0, double y1, double x1 ) {
    return (val-x0)*(y1-y0)/(x1-x0) + y0;
}

double Colors::base( double val ) {
    if ( val <= -0.75 ) return 0;
    else if ( val <= -0.25 ) return interpolate( val, 0.0, -0.75, 1.0, -0.25 );
    else if ( val <= 0.25 ) return 1.0;
    else if ( val <= 0.75 ) return interpolate( val, 1.0, 0.25, 0.0, 0.75 );
    else return 0.0;
}

double Colors::red( double gray ) {
    return base( gray - 0.5 );
}
double Colors::green( double gray ) {
    return base( gray );
}
double Colors::blue( double gray ) {
    return base( gray + 0.5 );
}
