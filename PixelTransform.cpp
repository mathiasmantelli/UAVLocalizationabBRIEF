#include "PixelTransform.h"

PixelTransform::PixelTransform()
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

    scaleX = state[0];
    deltaX = state[1];
    scaleY = state[2];
    deltaY = state[3];

    scale = sqrt(scaleX*scaleX + scaleY*scaleY);

    //  VectorXd result = A*state;
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
}

cv::Point2i PixelTransform::UTMtoPixel(UTMCoordinates& utm)
{
    cv::Point2i p;
    p.x = utm.easting*scaleX + deltaX;
    p.y = utm.northing*scaleY + deltaY;
    return p;
}

double PixelTransform::Pix2Met (double val)
{
    return val/scale;
}

double PixelTransform::Pix2MetX (double val)
{
    return val/scaleX;
}

double PixelTransform::Pix2MetY (double val)
{
    return val/scaleY;
}

double PixelTransform::Met2Pix (double val)
{
    return val*scale;
}

double PixelTransform::Met2PixX (double val)
{
    return val*scaleX;
}

double PixelTransform::Met2PixY (double val)
{
    return val*scaleY;
}


double PixelTransform::Deg2Dec(double deg, double min, double sec, char hem)
{
    double coord = deg + min/60.0 + sec/3600.0;
    if(hem == 'W' || hem == 'S')
        return -coord;
    else
        return coord;
}
