#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <string>
using namespace std;

#include "Utils.h"
#include "UTMConverter.h"
#include "readerlog.h"

#include <Eigen/Dense>
using namespace Eigen;

vector<double> getImagesHeadings(vector<ATT>& att, vector<double>& attTimes, vector<double>& imagesTimes, double deltaTime)
{
    vector<double> imagesHeadings;

    bool flagExit=false;
    int g=1;
    for(int i=0; i<imagesTimes.size(); ++i){
        double t = imagesTimes[i]+deltaTime;
        while(t > attTimes[g]){
            g++;
            if(g>=attTimes.size()){
                flagExit=true;
                break;
            }
        }
        if(flagExit){
            break;
        }

        // interpolate value
        double alpha = 1.0;
        if(attTimes[g]>attTimes[g-1])
            alpha = (imagesTimes[i]+deltaTime-attTimes[g-1])/(attTimes[g]-attTimes[g-1]);

        double th = (1.0-alpha)*att[g-1].values[ATT::Yaw] + (alpha)*att[g].values[ATT::Yaw];
        th -= 90;
        while(th > 180.0)
            th -= 360.0;
        while(th < -180.0)
            th += 360.0;

        imagesHeadings.push_back(th);
    }

    return imagesHeadings;
}

vector<cv::Point2i> getImagesCoords(vector<cv::Point2i>& gpsPixelCoords, vector<double>& gpsTimes, vector<double>& imagesTimes, double deltaTime)
{
    vector<cv::Point2i> imagesPixelsCoords;

    bool flagExit=false;
    int g=1;
    for(int i=0; i<imagesTimes.size(); ++i){
        double t = imagesTimes[i]+deltaTime;
        while(t > gpsTimes[g]){
            g++;
            if(g>=gpsTimes.size()){
                flagExit=true;
                break;
            }
        }
        if(flagExit){
            break;
        }

        // interpolate value
        double alpha = 1.0;
        if(gpsTimes[g]>gpsTimes[g-1])
            alpha = (imagesTimes[i]+deltaTime-gpsTimes[g-1])/(gpsTimes[g]-gpsTimes[g-1]);

        Point2i p;
        p.x = (1.0-alpha)*gpsPixelCoords[g-1].x + (alpha)*gpsPixelCoords[g].x;
        p.y = (1.0-alpha)*gpsPixelCoords[g-1].y + (alpha)*gpsPixelCoords[g].y;

        imagesPixelsCoords.push_back(p);
    }

    return imagesPixelsCoords;
}

double diff_sec(timespec t1, timespec t2)
{
    return (t1.tv_sec - t2.tv_sec) + (t1.tv_nsec - t2.tv_nsec)/1000000000.0;
}

double Deg2Dec(double deg, double min, double sec, char hem)
{
    double coord = deg + min/60.0 + sec/3600.0;
    if(hem == 'W' || hem == 'S')
        return -coord;
    else
        return coord;
}

vector<cv::Point2i> convertGPSToPixels(const vector<GPS>& gps)
{
    // CALIBRATION POINTS - pixels - Lat/Lng
    //  1 -  878  331 - 30°04'01,12"S 51°07'17,49"W
    //  2 - 1250  703 - 30°04'02,63"S 51°07'15,73"W
    //  3 - 3114 2486 - 30°04'10,28"S 51°07'06,48"W
    //  4 -  950 1883 - 30°04'07,58"S 51°07'17,23"W
    //  5 - 3304   96 - 30°04'00,12"S 51°07'05,87"W
    //  6 - 1487   14 - 30°03'59,83"S 51°07'14,58"W
    //  7 -  165 1250 - 30°04'04,92"S 51°07'20,97"W
    //  8 - 1456 1788 - 30°04'07,18"S 51°07'14,76"W
    //  9 - 2122  738 - 30°04'02,77"S 51°07'11,53"W
    // 10 - 2419 2389 - 30°04'09,75"S 51°07'10,04"W
    vector<Vector2i> pPts;
    vector<Vector2d> wPts;
    pPts.push_back(Vector2i( 878, 331)); wPts.push_back(Vector2d(Deg2Dec(30,04,01.12,'S'),Deg2Dec(51,07,17.49,'W')));
    pPts.push_back(Vector2i(1250, 703)); wPts.push_back(Vector2d(Deg2Dec(30,04,02.63,'S'),Deg2Dec(51,07,15.73,'W')));
    pPts.push_back(Vector2i(3114,2486)); wPts.push_back(Vector2d(Deg2Dec(30,04,10.28,'S'),Deg2Dec(51,07,06.48,'W')));
    pPts.push_back(Vector2i( 950,1883)); wPts.push_back(Vector2d(Deg2Dec(30,04,07.58,'S'),Deg2Dec(51,07,17.23,'W')));
    pPts.push_back(Vector2i(3304,  96)); wPts.push_back(Vector2d(Deg2Dec(30,04,00.12,'S'),Deg2Dec(51,07,05.87,'W')));
    pPts.push_back(Vector2i(1487,  14)); wPts.push_back(Vector2d(Deg2Dec(30,03,59.83,'S'),Deg2Dec(51,07,14.58,'W')));
    pPts.push_back(Vector2i( 165,1250)); wPts.push_back(Vector2d(Deg2Dec(30,04,04.92,'S'),Deg2Dec(51,07,20.97,'W')));
    pPts.push_back(Vector2i(1456,1788)); wPts.push_back(Vector2d(Deg2Dec(30,04,07.18,'S'),Deg2Dec(51,07,14.76,'W')));
    pPts.push_back(Vector2i(2122, 738)); wPts.push_back(Vector2d(Deg2Dec(30,04,02.77,'S'),Deg2Dec(51,07,11.53,'W')));
    pPts.push_back(Vector2i(2419,2389)); wPts.push_back(Vector2d(Deg2Dec(30,04,09.75,'S'),Deg2Dec(51,07,10.04,'W')));

    // Convert wPts to meters
    for(unsigned int i=0; i<wPts.size(); ++i){
        UTMCoordinates utm;
        UTMConverter::latitudeAndLongitudeToUTMCoordinates(wPts[i][0],wPts[i][1],utm);
//        cout << i
//             << " Lat/Lng " <<  wPts[i][0] << ' ' << wPts[i][1]
//             << " meters " << utm.northing << ' ' << utm.easting << endl;
        wPts[i][0] = utm.northing;
        wPts[i][1] = utm.easting;
    }

    // METERS -> PIXELS
    // FINDING PARAMETERS
    // x_p = (x_m - xo_m)/(xf_m - xo_m) * (xf_p - xo_p) = (x_m - xo_m) * alfa = x_m * a + b
    // y_p = (y_m - yo_m)/(yf_m - yo_m) * (yf_p - yo_p) = (y_m - yo_m) * ratioY = y_m * c + d

    // Linear System
    // [x_m1 1 0 0; 0 0 y_m1 1; x_m2 1 0 0; 0 0 y_m2 1; ...] [a; b; c; d]^T = [x_p1; y_p1; x_p2; y_p2; ...; x_pn; y_pn]^T
    // A * state = B
    // nx4 * 4x1 = nx1

    int n=wPts.size();
    MatrixXd A(2*n,4);
    VectorXd state(4);
    VectorXd b(2*n);

    for(int l=0; l<n; ++l){
        int i=2*l;
        int j=2*l+1;
        A(i,0) = wPts[l][1]; A(i,1) = 1; A(i,2) = 0;          A(i,3) = 0;
        A(j,0) = 0;          A(j,1) = 0; A(j,2) = wPts[l][0]; A(j,3) = 1;
        b(i) = pPts[l][0];   b(j) = pPts[l][1];
    }

//    cout << "A " << A << endl;
//    cout << "b " << b << endl;

    // Solving the linear system
    // A^T * A * x = A^T * b
    // 4xn * nx4 * 4x1 = 4xn * nx1
    state = (A.transpose() * A).inverse() * (A.transpose() * b);

    VectorXd result = A*state;

//    cout << "state " << state << endl;
//    cout << "Error " << result-b << endl;

    // Convert all values to meters
    vector<cv::Point2i> posInPixels;
    for(unsigned int i=0; i<gps.size(); ++i){
        UTMCoordinates utm;
        UTMConverter::latitudeAndLongitudeToUTMCoordinates(gps[i].values[GPS::Lat],gps[i].values[GPS::Lng],utm);
        cv::Point2i p;
        p.x = utm.easting*state[0] + state[1];
        p.y = utm.northing*state[2] + state[3];
        posInPixels.push_back(p);
//        cout << i << " time " << gps[i].values[GPS::timeMS]/1000.0
//             << " Lat/Lng " <<  gps[i].values[GPS::Lat] << ' ' << gps[i].values[GPS::Lng]
//             << " meters " << utm.easting << ' ' << utm.northing
//             << " pixels " << p.x << ' ' << p.y << endl;
    }

//    Mat img = imread("../Datasets/ufrgs_dronao_28ago/121012_boa.png",CV_LOAD_IMAGE_COLOR);
    // Draw Calibration points
//    for(int l=0; l<n; ++l){
//        Point2i center(pPts[l][0],pPts[l][1]);
//        circle( img, center, 50, Scalar( 0, 0, 255 ), 5);
//        circle( img, center, 5, Scalar( 0, 0, 255 ), 15);

//        Point2i newCenter(result[2*l],result[2*l+1]);
//        circle( img, newCenter, 50, Scalar( 0, 180, 255 ), 5);
//        circle( img, newCenter, 5, Scalar( 0, 180, 255 ), 15);
//    }

    return posInPixels;
}

vector<double> getGPSRelTime(const vector<GPS>& gps)
{
    vector<double> gpsTimesInSec;
    double initialTime;
    for(unsigned int i=0; i<gps.size(); ++i){
        if(i==0){
            initialTime = gps[i].values[GPS::timeMS]/1000.0;
            gpsTimesInSec.push_back(0.0);
        }else{
            gpsTimesInSec.push_back(gps[i].values[GPS::timeMS]/1000.0-initialTime);
        }
    }

//    cout << "Times ";
//    for(int i=0; i<gpsTimesInSec.size(); ++i){
//        cout << gpsTimesInSec[i] << ' ';
//    }
//    cout << endl;

    return gpsTimesInSec;
}

vector<double> getATTRelTime(const vector<ATT>& att)
{
    vector<double> attTimesInSec;
    double initialTime;
    for(unsigned int i=0; i<att.size(); ++i){
        if(i==0){
            initialTime = att[i].values[ATT::timeMS]/1000.0;
            attTimesInSec.push_back(0.0);
        }else{
            attTimesInSec.push_back(att[i].values[ATT::timeMS]/1000.0-initialTime);
        }
    }

//    cout << "Times ";
//    for(int i=0; i<attTimesInSec.size(); ++i){
//        cout << attTimesInSec[i] << ' ';
//    }
//    cout << endl;

    return attTimesInSec;
}

vector<double> getImagesRelTime(string trajPath)
{
    // Read images names
    fstream input;
    input.open(trajPath,std::fstream::in);
    if(!input.is_open()){
        cout << "File: " << trajPath << " does not exist" << endl;
        exit(0);
    }

    vector<double> imagesTimesInSec;
//    time_t initialTime;
    timespec iT;

    int i=0;
    while(input.peek() != fstream::traits_type::eof()){
        string imagePath;
        getline(input,imagePath);
//        cout << "image " << i << " " << imagePath << endl;

        struct stat buf;
        stat(imagePath.c_str(), &buf);
//        time_t mtim = buf.st_mtim.tv_sec;
//        cout << "st_mtim " << ctime(&mtim);
//        struct tm* creation = localtime(&mtim);
//        cout << setfill('0') << setw(2) << creation->tm_mday << "/"
//             << setfill('0') << setw(2) << creation->tm_mon << "/"
//             << 1900+creation->tm_year << " "
//             << setfill('0') << setw(2) << creation->tm_hour << ":"
//             << setfill('0') << setw(2) << creation->tm_min << ":"
//             << setfill('0') << setw(2) << creation->tm_sec << endl ;

        if(i==0){
            imagesTimesInSec.push_back(0);
//            initialTime = mtim;
            iT = buf.st_mtim;
        }else{

//            double diff = difftime(mtim, initialTime);
            double diff = diff_sec(buf.st_mtim, iT);
            imagesTimesInSec.push_back(diff);
        }

//        break;
//        imagesNames.push_back(tempStr);
        i++;
    }
//    cout << "Num images " << imagesNames.size() << endl;

//    cout << "Times ";
//    for(int i=0; i<imagesTimesInSec.size(); ++i){
//        cout << imagesTimesInSec[i] << ' ';
//    }
//    cout << endl;

    return imagesTimesInSec;
}

void readLogFile(string lf, vector<IMU>& imu1, vector<IMU>& imu2, vector<GPS>& gps, vector<ATT>& att)
{
    ReaderLog logFile(lf);
    logFile.Read();

    imu1 = logFile.get_sensor_IMU();
    imu2 = logFile.get_sensor_IMU2();
    gps = logFile.get_sensor_GPS();
    att = logFile.get_sensor_ATT();
}

void parseArgs(int argc, char* argv[], string& lf, string& tp, int& deltaTime)
{
    if(argc<3)
    {
        cout << "Usage: ReaderLog -l <path/logFile> -t <path/trajectory>" << endl;
        exit(0);
    }

    int p=0;
    while(p<argc)
    {
        // print help and exit
        if(!strncmp(argv[p], "-h", 2) || !strncmp(argv[p], "-H", 2) || !strncmp(argv[p], "--help", 6))
        {
            cout << "Usage: ReaderLog -l <path/logFile>" << endl;
            exit(0);
        }
        else if(!strncmp(argv[p], "-l", 2) || !strncmp(argv[p], "-L", 2))
        {
            // check if there is a file name
            if(argc>p+1)
            {
                lf=argv[p+1];
                p+=2;
            }
            else
            {
                cerr << "Failed to load logfile, missing argument " << p << endl;
                p++;
            }
        }
        else if(!strncmp(argv[p], "-t", 2) || !strncmp(argv[p], "-T", 2))
        {
            // check if there is a file name
            if(argc>p+1)
            {
                tp=argv[p+1];
                p+=2;
            }
            else
            {
                cerr << "Failed to load trajectory, missing argument " << p << endl;
                p++;
            }
        }
        else if(!strncmp(argv[p], "-d", 2) || !strncmp(argv[p], "-D", 2))
        {
            // check if there is a delta value
            if(argc>p+1)
            {
                deltaTime=stod(argv[p+1]);
                p+=2;
            }
            else
            {
                cerr << "Failed to load deltaTime, missing argument " << p << endl;
                p++;
            }
        }
        else
            p++;
    }
}

void exportGroundTruth(string& tp, vector<cv::Point2i>& imagesPixelCoords, vector<double>& imagesHeadings)
{
    // Open odometry file, if available
    string rawname = tp.substr(0, tp.find_last_of("."));
    fstream gTruth;
    gTruth.open(rawname+"_truth.txt",std::fstream::out);

    for(int l=0; l<imagesPixelCoords.size(); ++l){
        gTruth << imagesPixelCoords[l].x << ' ' << imagesPixelCoords[l].y << ' ' << imagesHeadings[l] << endl;
    }

    gTruth.close();
}

int main(int argc, char* argv[])
{
    string lf, tp;
    int deltaTime;

    parseArgs(argc,argv,lf,tp,deltaTime);

    Mat img = imread("../Datasets/ufrgs_dronao_28ago/121012_boa.png",CV_LOAD_IMAGE_COLOR);

    cout << "delta " << deltaTime << endl;

    vector<IMU> imu1, imu2;
    vector<GPS> gps;
    vector<ATT> att;

    readLogFile(lf,imu1,imu2,gps,att);

    cout << "GPS " << gps.size() << " IMU1 " << imu1.size() << " IMU2 " << imu2.size() << endl;
//    for(int i=0; i<min(10,int(imu1.size())); ++i){
//        cout << i << ' ' << imu1[i].values[IMU::AccX] << ' ' << imu1[i].values[IMU::AccY] << endl;
//    }

    vector<cv::Point2i> gpsPixelCoords = convertGPSToPixels(gps);

    // Draw path
    for(int l=0; l<gpsPixelCoords.size()-1; ++l){
        line( img, gpsPixelCoords[l], gpsPixelCoords[l+1], Scalar( 0, 0, 255 ), 5);
    }

    vector<double> gpsTimes = getGPSRelTime(gps);
    vector<double> attTimes = getATTRelTime(att);
    vector<double> imagesTimes = getImagesRelTime(tp);

    vector<cv::Point2i> imagesPixelCoords = getImagesCoords(gpsPixelCoords, gpsTimes, imagesTimes, deltaTime);
    vector<double> imagesHeadings = getImagesHeadings(att, attTimes, imagesTimes, deltaTime);

    // Draw path
    for(int l=0; l<imagesPixelCoords.size()-1; ++l){
        line( img, imagesPixelCoords[l], imagesPixelCoords[l+1], Scalar( 0, 0, 255 ), 5);
    }

    circle( img, imagesPixelCoords[61], 50, Scalar( 0, 180, 255 ), 5);
    circle( img, imagesPixelCoords[61], 5, Scalar( 0, 180, 255 ), 15);

    resize(img,img,Size(0,0),0.35,0.35);
    imshow("Mapa",img);
//    waitKey();

    Mat headingImg(400,800,CV_8UC3,Scalar(255,255,255));
    cout << headingImg.rows << ' ' << headingImg.cols << endl;

    vector<cv::Point2d> ptsHeadings;
    double n = imagesHeadings.size();
    for(int l=0; l<n; ++l){
        cv::Point2d p;
        p.x = (l*headingImg.cols)/n;
//        p.y = (360.0-imagesHeadings[l])/360.0 * headingImg.rows;
        p.y = (180.0-imagesHeadings[l])/360.0 * headingImg.rows;
//        cout << ' ' << imagesHeadings[l] << flush;
        ptsHeadings.push_back(p);
    }

    for(int l=0; l<ptsHeadings.size()-1; ++l){
        line( headingImg, ptsHeadings[l], ptsHeadings[l+1], Scalar( 0, 0, 255 ), 2);
    }

    imshow("plot",headingImg);
    waitKey();

    exportGroundTruth(tp, imagesPixelCoords, imagesHeadings);

    return 0;
}
