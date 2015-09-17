#include "SiftHeuristic.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"

#include "Utils.h"

Rtree::Rtree(std::vector<valueCV> inputPoints)
{
//    std::cout << "filling index with boxes shared pointers:" << std::endl;
    for( unsigned i = 0 ; i < inputPoints.size(); ++i )
    {
        valueRT v(pointRT(inputPoints[i].first.x,inputPoints[i].first.y),inputPoints[i].second);
        rtree.insert(v);
    }
}

std::vector<unsigned int> Rtree::findPointsInsideCircle(cv::Point2f center, float radius)
{
    std::vector<valueRT> result;
    std::vector<unsigned int> resultIDs;

    // find values intersecting some area defined by a box
    boxRT query_box(pointRT(center.x-radius, center.y-radius), pointRT(center.x+radius, center.y+radius));
    rtree.query(bgi::intersects(query_box), std::back_inserter(result));

    for( unsigned int i = 0 ; i < result.size(); ++i )
    {
        resultIDs.push_back(result[i].second);
//        std::cout << result[i].first.get<0>() << ',' << result[i].first.get<1>() << " id " << result[i].second << std::endl;
    }

    return resultIDs;
}

SIFTHeuristic::SIFTHeuristic(STRATEGY s, int id, cv::Mat& grayMap, double *kernel, int kW, int kH, int rad, double l, unsigned int cd):
KernelHeuristic(s,id,l,cd,rad,kernel,kW,kH)
{
    feature_detector=cv::xfeatures2d::SIFT::create(100000,3);
    feature_extractor=feature_detector;
    feature_detector->detect(grayMap,keypoints_globalMap);
    feature_extractor->compute(grayMap,keypoints_globalMap,descriptors_globalMap);
    descriptorsType = descriptors_globalMap.type();

    // Add points to Rtree
    vector<valueCV> points;
    for ( unsigned i = 0 ; i < keypoints_globalMap.size() ; ++i )
    {
        points.push_back(valueCV(keypoints_globalMap[i].pt,i));
    }
    rtree = new Rtree(points);
}

void SIFTHeuristic::updateMatcher(cv::Mat &localImage)
{
    cv::Mat descriptors_localImage;

    feature_detector->detect(localImage,keypoints_localMap);
    feature_extractor->compute(localImage,keypoints_localMap,descriptors_localImage);

    localMatcher.clear();
    localMatcher.add(descriptors_localImage);
}

double SIFTHeuristic::calculateValue(int x, int y, cv::Mat *image, cv::Mat *map)
{
    std::vector<unsigned int> nearbyPoints = rtree->findPointsInsideCircle(cv::Point2f(x,y),radius);

    if(nearbyPoints.empty())
        return 0.0;
    else{

        // Copy descriptors associated to nearbyPoints
        cv::Mat descriptors(nearbyPoints.size(),128,descriptorsType);
        for(int k=0; k<nearbyPoints.size(); ++k){
            descriptors_globalMap.row(nearbyPoints[k]).copyTo(descriptors.row(k));
        }

        //-- Match descriptor vectors using FLANN matcher
        std::vector< cv::DMatch > matches;
        localMatcher.match( descriptors, matches );

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
//        float countGoodMatches=0.0;
        for( int i = 0; i < matches.size(); i++ )
        {
            if( matches[i].distance <= max(2*min_dist, 0.02) ){
//                countGoodMatches += 1.0;
                good_matches.push_back( matches[i]);
            }
        }

//        if(checkTransformation(good_matches,nearbyPoints) == false)
//            return 0.0;

        return good_matches.size();
    }
}

bool SIFTHeuristic::checkTransformation(std::vector< cv::DMatch >& good_matches, std::vector<unsigned int>& nearbyPoints)
{
    int minNumPoints = 6;

    // Find transformation
    std::vector<cv::Point2f> points1, points2;
    for( int i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        points1.push_back( keypoints_globalMap[ nearbyPoints[good_matches[i].queryIdx] ].pt );
        points2.push_back( keypoints_localMap[ good_matches[i].trainIdx ].pt );
    }

    cv::Mat H;
    if(good_matches.size()>=minNumPoints)
        H = findHomography( points1, points2, CV_RANSAC );

    if(H.empty())
        return false;

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<cv::Point2f> obj_corners(7);
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( radius, 0 );
    obj_corners[2] = cvPoint( radius, radius ); obj_corners[3] = cvPoint( 0, radius );

    obj_corners[4] = cvPoint( radius/2, radius/2 ); // center
    obj_corners[5] = cvPoint( radius, radius/2 ); obj_corners[6] = cvPoint( radius/2, 0 );

    std::vector<cv::Point2f> scene_corners(7);

    perspectiveTransform( obj_corners, scene_corners, H);

    // Compute approximate translation and rotation
//    Pose p;
//    p.y = scene_corners[4].x - obj_corners[4].x;
//    p.x = -(scene_corners[4].y - obj_corners[4].y);
//    p.theta = atan2(scene_corners[5].y-scene_corners[4].y,scene_corners[5].x-scene_corners[4].x);

    // Check if it is a good transformation
    // 1st - the displacement must be smaller than half diagonal of the image (times 0.8)
    double origDiag = Utils::getNorm(obj_corners[1]-obj_corners[3]);
    double ratioDisp = Utils::getNorm(scene_corners[4]-obj_corners[4])/(origDiag*0.5);
    if(ratioDisp > 0.8){
        return false;
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
        return false;
    }

    // 3rd - the angle between the axes must be around 90 degrees
    double angle = Utils::getDiffAngle(scene_corners[5]-scene_corners[4], scene_corners[6]-scene_corners[4]);
    if(fabs(angle - 90.0) > 25.0){
        return false;
    }

    // 4th - the ratio between oposed sides of the projected image must be around 1.0
    double a = Utils::getNorm(scene_corners[1]-scene_corners[0]);
    double b = Utils::getNorm(scene_corners[2]-scene_corners[1]);
    double c = Utils::getNorm(scene_corners[3]-scene_corners[2]);
    double d = Utils::getNorm(scene_corners[0]-scene_corners[3]);
    double ratioX = std::max(a/c,c/a);
    double ratioY = std::max(b/d,d/b);
    if(ratioX > 1.8 || ratioY > 1.8){
        return false;
    }

    return true;
}

