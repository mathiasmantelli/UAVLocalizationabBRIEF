#ifndef SIFTHEURISTIC_H
#define SIFTHEURISTIC_H

#include "Heuristic.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>

#include <boost/geometry/index/rtree.hpp>

#include <cmath>
#include <vector>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<float, 2, bg::cs::cartesian> pointRT;
typedef bg::model::box<pointRT> boxRT;

typedef std::pair<cv::Point2f, unsigned> valueCV;
typedef std::pair<pointRT, unsigned> valueRT;

class Rtree
{
public:
    Rtree(std::vector<valueCV> inputPoints);

    std::vector<unsigned int> findPointsInsideCircle(cv::Point2f center, float radius);

private:
    // create the rtree using default constructor
    bgi::rtree< valueRT, bgi::rstar<16> > rtree;
};

class SIFTHeuristic : public KernelHeuristic
{
public:
    SIFTHeuristic(STRATEGY s, int id, cv::Mat& grayMap, double *kernel, int kW, int kH, int rad, double l=5.0, unsigned int cd=INTENSITYC);
    double calculateValue(int x, int y, cv::Mat *image=NULL, cv::Mat* map=NULL);

    void updateMatcher(cv::Mat &localImage);

private:

    bool checkTransformation(std::vector< cv::DMatch >& good_matches, std::vector<unsigned int>& nearbyPoints);

    cv::Ptr<cv::Feature2D> feature_detector;
    cv::Ptr<cv::Feature2D> feature_extractor;
    cv::FlannBasedMatcher feature_matcher;
    std::vector<cv::KeyPoint> keypoints_globalMap;
    std::vector<cv::KeyPoint> keypoints_localMap;
    cv::Mat descriptors_globalMap;
    int descriptorsType;

    cv::FlannBasedMatcher localMatcher;

    Rtree* rtree;
};

#endif // SIFTHEURISTIC_H
