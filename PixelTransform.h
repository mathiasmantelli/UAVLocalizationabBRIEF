#ifndef PIXELTRANSFORM_H
#define PIXELTRANSFORM_H

#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
using namespace cv;

#include <Eigen/Dense>
using namespace Eigen;

#include "UTMConverter.h"
#include "Utils.h"

class PixelTransform
{
public:
    PixelTransform();
    cv::Point2i UTMtoPixel(UTMCoordinates& utm);

    double Pix2Met (double val);
    double Pix2MetX(double val);
    double Pix2MetY(double val);
    double Met2Pix (double val);
    double Met2PixX(double val);
    double Met2PixY(double val);

private:
    double scale;
    double scaleX;
    double scaleY;
    double deltaX;
    double deltaY;

    double Deg2Dec(double deg, double min, double sec, char hem);
};

#endif // PIXELTRANSFORM_H
