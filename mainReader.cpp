#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <string>
using namespace std;

#include "PixelTransform.h"
#include "Utils.h"
#include "readerlog.h"

vector<IMU> getImagesIMU(vector<IMU>& imu, vector<double>& imuTimes, vector<double>& imagesTimes, double deltaTime)
{
    vector<IMU> imagesIMU;
    vector<double> imagesIMUid;

    int N = imagesTimes.size();

    // find nearest imu value for each image
    bool flagExit=false;
    int g=1;
    for(int i=0; i<N; ++i){
        double t = imagesTimes[i]+deltaTime;
        while(t > imuTimes[g]){
            g++;
            if(g>=imuTimes.size()){
                flagExit=true;
                break;
            }
        }
        if(flagExit){
            break;
        }

        // interpolate value
        double alpha = 1.0;
        if(imuTimes[g]>imuTimes[g-1])
            alpha = (imagesTimes[i]+deltaTime-imuTimes[g-1])/(imuTimes[g]-imuTimes[g-1]);

//        IMU temp;
//        for(int s=0;s<7;++s)
//            temp.values[s] = (1.0-alpha)*imu[g-1].values[s] + alpha*imu[g].values[s];
//        imagesIMU.push_back(temp);

        imagesIMUid.push_back((1.0-alpha)*(g-1) + alpha*g);
    }

    // compute limits of nearest imu values for each image
//    vector<pair<int,int> > limits;
//    limits.push_back(pair<int,int>(
//                                   std::max(0,int(imagesIMUid[0]-(imagesIMUid[0]+imagesIMUid[1])/2)),
//                                   (imagesIMUid[0]+imagesIMUid[1])/2
//                                  ));
//    for(int i=1; i<N-1; ++i)
//        limits.push_back(pair<int,int>(
//                                       (imagesIMUid[i-1]+imagesIMUid[i])/2,
//                                       (imagesIMUid[i]+imagesIMUid[i+1])/2
//                                      ));
//    limits.push_back(pair<int,int>(
//                                   (imagesIMUid[N-2]+imagesIMUid[N-1])/2,
//                                   std::min(N-1,int(imagesIMUid[N-1]+(imagesIMUid[N-2]+imagesIMUid[N-1])/2))
//                                  ));

//    // compute average
//    for(int i=0; i<N; ++i){
//        IMU temp;
//        for(int s=0;s<7;++s)
//            temp.values[s]=0;
//        for(int l=limits[i].first; l<limits[i].second; ++l){
//            for(int s=0;s<7;++s)
//                temp.values[s]+=imu[l].values[s];
//        }
//        for(int s=0;s<7;++s)
//            temp.values[s] /= (limits[i].second-limits[i].first);
//        imagesIMU.push_back(temp);
//    }

    for(int i=0; i<N; ++i){
        double max=0;
        IMU temp;
        for(int l=imagesIMUid[i]-12; l<=imagesIMUid[i]; ++l){
            if(fabs(imu[l].values[IMU::GyrY]) > max){
                max = fabs(imu[l].values[IMU::GyrY]);
                temp = imu[l];
            }
        }
        imagesIMU.push_back(temp);
    }

    return imagesIMU;
}

vector<IMU> getImagesCombinedIMU(vector<IMU>& imu1, vector<double>& imu1Times, vector<IMU>& imu2, vector<double>& imu2Times, vector<double>& imagesTimes, double deltaTime)
{
    vector<IMU> imagesIMU1 = getImagesIMU(imu1, imu1Times, imagesTimes, deltaTime);
    vector<IMU> imagesIMU2 = getImagesIMU(imu2, imu2Times, imagesTimes, deltaTime);

    vector<IMU> avgImagesIMU;
    for(int i=0; i<imagesIMU1.size(); ++i){
        IMU temp;
        for(int s=0;s<7;++s)
            temp.values[s] = (imagesIMU1[i].values[s]+imagesIMU2[i].values[s])/2.0;
        avgImagesIMU.push_back(temp);
    }
    return avgImagesIMU;
}

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

vector<Vector3d> convertIMUDataToOdometry(const vector<GPS>& imu, PixelTransform tf)
{

}

vector<cv::Point2i> convertGPSToPixels(const vector<GPS>& gps, PixelTransform tf)
{
    // Convert all values to meters
    vector<cv::Point2i> posInPixels;
    for(unsigned int i=0; i<gps.size(); ++i){
        UTMCoordinates utm;
        UTMConverter::latitudeAndLongitudeToUTMCoordinates(gps[i].values[GPS::Lat],gps[i].values[GPS::Lng],utm);
        posInPixels.push_back(tf.UTMtoPixel(utm));
    }

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

    return gpsTimesInSec;
}

vector<double> getIMURelTime(const vector<IMU>& imu)
{
    vector<double> imuTimesInSec;
    double initialTime;
    for(unsigned int i=0; i<imu.size(); ++i){
        if(i==0){
            initialTime = imu[i].values[IMU::timeMS]/1000.0;
            imuTimesInSec.push_back(0.0);
        }else{
            imuTimesInSec.push_back(imu[i].values[IMU::timeMS]/1000.0-initialTime);
        }
    }

    return imuTimesInSec;
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

    return attTimesInSec;
}

vector<bool> removeFuckedImages(string trajPath, vector<IMU>& imagesIMU)
{
    // Read images names
    fstream input;
    input.open(trajPath,std::fstream::in);
    if(!input.is_open()){
        cout << "File: " << trajPath << " does not exist" << endl;
        exit(0);
    }

    vector<double> gyrX, gyrY, gyrZ;

    for(int i=0; i<imagesIMU.size(); ++i){
        gyrX.push_back(imagesIMU[i].values[IMU::GyrX]);
        gyrY.push_back(imagesIMU[i].values[IMU::GyrY]);
        gyrZ.push_back(imagesIMU[i].values[IMU::GyrZ]);
    }

    double min, max;
    min = *std::min_element(gyrX.begin(),gyrX.end());
    max = *std::max_element(gyrX.begin(),gyrX.end());
    double thresholdX = std::max(fabs(max),fabs(min))*0.1;
    min = *std::min_element(gyrY.begin(),gyrY.end());
    max = *std::max_element(gyrY.begin(),gyrY.end());
    double thresholdY = std::max(fabs(max),fabs(min))*0.1;
    min = *std::min_element(gyrZ.begin(),gyrZ.end());
    max = *std::max_element(gyrZ.begin(),gyrZ.end());
    double thresholdZ = std::max(fabs(max),fabs(min))*0.1;

    vector<bool> goodOnes;

    int i=0;
    while(input.peek() != fstream::traits_type::eof()){
        string imagePath;
        getline(input,imagePath);

        if(fabs(gyrX[i])>thresholdX || fabs(gyrY[i])>thresholdY || fabs(gyrZ[i])>thresholdZ){
//            Mat image = imread(imagePath,CV_LOAD_IMAGE_COLOR);
//            resize(image,image,Size(0,0),0.25,0.25);
            string rawname = imagePath.substr(imagePath.find_last_of("/")+1,imagePath.size());
            cout << rawname << endl;
//            imshow(rawname,image);
            goodOnes.push_back(false);
        }else{
            goodOnes.push_back(true);
        }

        i++;
    }

    return goodOnes;
}

void exportGoodTraj(vector<bool>& goodOnes, string tp, vector<cv::Point2i>& imagesPixelCoords, vector<double>& imagesHeadings)
{
    // Read images names
    fstream input;
    input.open(tp,std::fstream::in);
    if(!input.is_open()){
        cout << "File: " << tp << " does not exist" << endl;
        exit(0);
    }

    string rawname = tp.substr(0, tp.find_last_of("."));
    fstream gTruth;
    gTruth.open(rawname+"_good_truth.txt",std::fstream::out);
    fstream newTraj;
    newTraj.open(rawname+"_good.txt",std::fstream::out);

    int i=0;
    while(input.peek() != fstream::traits_type::eof()){
        string imagePath;
        getline(input,imagePath);

        if(goodOnes[i]){
            newTraj << imagePath << endl;
            gTruth << imagesPixelCoords[i].x << ' ' << imagesPixelCoords[i].y << ' ' << imagesHeadings[i] << endl;
        }

        i++;
    }
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
    timespec iT;

    int i=0;
    while(input.peek() != fstream::traits_type::eof()){
        string imagePath;
        getline(input,imagePath);

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
        i++;
    }

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
    string rawname = tp.substr(0, tp.find_last_of("."));
    fstream gTruth;
    gTruth.open(rawname+"_truth.txt",std::fstream::out);

    for(int l=0; l<imagesPixelCoords.size(); ++l){
        gTruth << imagesPixelCoords[l].x << ' ' << imagesPixelCoords[l].y << ' ' << imagesHeadings[l] << endl;
    }

    gTruth.close();
}

void plotData(string name, vector<double> data, Mat& canvas, Scalar color, double min=-1, double max=-1)
{
//    cout << canvas.rows << ' ' << canvas.cols << endl;

    if(min==-1 && max==-1){
        min = *std::min_element(data.begin(),data.end());
        max = *std::max_element(data.begin(),data.end());
    }

    vector<cv::Point2d> pts;
    double n = data.size();
    for(int i=0; i<n; ++i){
        cv::Point2d p;
        p.x = (i*canvas.cols)/n;
        p.y = (max-data[i])/(max-min) * canvas.rows;
        pts.push_back(p);
    }

    for(int i=0; i<pts.size()-1; ++i){
        line( canvas, pts[i], pts[i+1], color, 2);
    }
}

void plotData(string name, vector<double> data, double min=-1, double max=-1, int w=800, int h=400)
{
    Mat canvas(h,w,CV_8UC3,Scalar(255,255,255));
    plotData(name,data,canvas,Scalar(0,0,255),min,max);
    imshow(name,canvas);
}

void plotIMU(vector<IMU>& imagesIMU, Mat& canvas)
{
    vector<double> accX;
    vector<double> accY;
    vector<double> accZ;
    vector<double> gyrX;
    vector<double> gyrY;
    vector<double> gyrZ;

    for(int i=0; i<imagesIMU.size(); ++i)
    {
        accX.push_back(imagesIMU[i].values[IMU::AccX]);
        accY.push_back(imagesIMU[i].values[IMU::AccY]);
        accZ.push_back(imagesIMU[i].values[IMU::AccZ]);
        gyrX.push_back(imagesIMU[i].values[IMU::GyrX]);
        gyrY.push_back(imagesIMU[i].values[IMU::GyrY]);
        gyrZ.push_back(imagesIMU[i].values[IMU::GyrZ]);
    }

//    plotData("AccX",accX);
//    plotData("AccY",accY);
//    plotData("AccZ",accZ);
    plotData("GyrX",gyrX,canvas,Scalar(160,160,160));
    plotData("GyrY",gyrY,canvas,Scalar(0,0,0));
//    plotData("GyrZ",gyrZ);
}

vector<Pose3d> estimateOdometry(vector<IMU>& imu)
{
//    vector<Pose3d> odom;
//    Pose3d pos;
//    odom.push_back(pos);

    enum { X, Y, Z};
    enum { Rol, Pit, Yaw}; // roll pitch yaw

    Vector3d pos;
    Vector3d vel;
    Vector3d att;

    vector<double> px, py, pz, vx, vy, vz, ar, ap, ay;

    for(int i=1; i<imu.size();++i){
        // Get dT in seconds
        double dT = (imu[i].values[IMU::timeMS]-imu[i-1].values[IMU::timeMS])/1000.0;

        // Rotate the acceleration vector from the ROBOT frame to the WORLD frame
        Eigen::Vector3d accelR(imu[i].values[IMU::AccX], imu[i].values[IMU::AccY], imu[i].values[IMU::AccZ]);

        Eigen::AngleAxisd rollAngle(att[Rol], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(att[Pit], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(att[Yaw], Eigen::Vector3d::UnitZ());
        Eigen::Vector3d accel = yawAngle * pitchAngle * rollAngle * accelR;

        // Cancelgravity gravity
        accel[Z] += 9.8666269;

        // Update the velocity
        vel += accel * dT;

        // Update the position
        pos += vel * dT;
//        cout <<  "a " << accel[X] << ',' << accel[Y] << ',' << accel[Z]
//             << " v " << vel[X] << ',' << vel[Y] << ',' << vel[Z]
//             << " p " << pos[X] << ',' << pos[Y] << ',' << pos[Z] << endl;

        // Update the attitude
        Eigen::Vector3d gyro(imu[i].values[IMU::GyrX], imu[i].values[IMU::GyrY], imu[i].values[IMU::GyrZ]);
        att += gyro * dT;

        px.push_back(pos[X]); py.push_back(pos[Y]); pz.push_back(pos[Z]);
        vx.push_back(vel[X]); vy.push_back(vel[Y]); vz.push_back(vel[Z]);
        ar.push_back(att[Rol]); ap.push_back(att[Pit]); ay.push_back(att[Yaw]);
    }

//    Mat canvas(400,800,CV_8UC3,Scalar(255,255,255));
//    plotData("PosX",px,canvas,Scalar(160,160,160));
//    plotData("PosY",py,canvas,Scalar(0,0,0));
////    plotData("PosZ",pz,canvas,Scalar(0,0,0));
//    imshow("Odom",canvas);
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

    // Initialize transform
    PixelTransform tf;
    vector<cv::Point2i> gpsPixelCoords = convertGPSToPixels(gps,tf);

    vector<double> gpsTimes = getGPSRelTime(gps);
    vector<double> attTimes = getATTRelTime(att);
    vector<double> imu1Times = getIMURelTime(imu1);
    vector<double> imu2Times = getIMURelTime(imu2);
    vector<double> imagesTimes = getImagesRelTime(tp);

    vector<cv::Point2i> imagesPixelCoords = getImagesCoords(gpsPixelCoords, gpsTimes, imagesTimes, deltaTime);
    vector<double> imagesHeadings = getImagesHeadings(att, attTimes, imagesTimes, deltaTime);
    vector<IMU> imagesIMU = getImagesCombinedIMU(imu1, imu1Times, imu2, imu2Times, imagesTimes, deltaTime);

    Mat canvas(400,800,CV_8UC3,Scalar(255,255,255));

    vector<bool> goodOnes = removeFuckedImages(tp,imagesIMU);
    exportGoodTraj(goodOnes, tp, imagesPixelCoords, imagesHeadings);

    // Draw path
//    for(int l=0; l<gpsPixelCoords.size()-1; ++l){
//        line( img, gpsPixelCoords[l], gpsPixelCoords[l+1], Scalar( 0, 0, 255 ), 5);
//    }
    for(int l=0; l<imagesPixelCoords.size()-1; ++l){
        line( img, imagesPixelCoords[l], imagesPixelCoords[l+1], Scalar( 255, 80, 0), 5);
    }

    vector<double> xImg, yImg;
    for(int l=0; l<imagesPixelCoords.size(); ++l){
        xImg.push_back(imagesPixelCoords[l].x);
        yImg.push_back(imagesPixelCoords[l].y);
    }

    plotData("x",xImg,canvas,Scalar(255,0,0));
    plotData("y",yImg,canvas,Scalar(0,180,0));
    plotData("heading",imagesHeadings,canvas,Scalar(0,0,255));

//    plotIMU(imagesIMU,canvas);
    imshow("plotao",canvas);

    circle( img, imagesPixelCoords[16], 50, Scalar( 0, 180, 255 ), 5);
    circle( img, imagesPixelCoords[16], 5, Scalar( 0, 180, 255 ), 15);

    resize(img,img,Size(0,0),0.35,0.35);
    imshow("Mapa",img);

    waitKey();

    exportGroundTruth(tp, imagesPixelCoords, imagesHeadings);

    estimateOdometry(imu1);

    return 0;
}
