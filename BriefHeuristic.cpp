#include "BriefHeuristic.h"

BriefHeuristic::BriefHeuristic(STRATEGY s, int id, int cd, double l):Heuristic(s,id,cd,l)
{
    totalPairs = 1000;
    lowThreshold = 0.55;
    multiplierThreshold = 4;
    margin = 10;
}

void BriefHeuristic::printInfo(){
    cout << endl << "Brief parameters:\nPairs:\t\t" << totalPairs << "\nLow Threshold:\t" << lowThreshold << "\nMultiplier:\t" << multiplierThreshold << "\nMargin:\t\t" << margin << endl << endl;
}

void BriefHeuristic::updateDroneDescriptor(cv::Mat& drone){
    width = drone.cols;
    height = drone.rows;

    if(pairs.size() < 1){
        int halfMargin = margin/2;
        cv::RNG rng;
        for(int x=0;x<totalPairs;x++){
            vector<cv::Point> pair;
            pair.push_back(cv::Point((int)rng.uniform(halfMargin, drone.cols-halfMargin), (int)rng.uniform(halfMargin, drone.rows-halfMargin)));
            pair.push_back(cv::Point((int)rng.uniform(halfMargin, drone.cols-halfMargin), (int)rng.uniform(halfMargin, drone.rows-halfMargin)));
            pairs.push_back(pair);
        }
    }

    vector<int> bin;

    for(int c=0;c<pairs.size();c++){
        vec3 color1(getValuefromPixel(pairs[c][0].x,pairs[c][0].y,&drone));
        vec3 color2(getValuefromPixel(pairs[c][1].x,pairs[c][1].y,&drone));
        if(color1.x>color2.x) bin.push_back(1);
        else bin.push_back(0);
        if(color1.y>color2.y) bin.push_back(1);
        else bin.push_back(0);
        if(color1.z>color2.z) bin.push_back(1);
        else bin.push_back(0);

    }

    droneDescriptor = bin;
}

double BriefHeuristic::calculateValue2(Pose p, cv::Mat *map){
    vector<int> bin;

    float theta = p.theta*-1-1.5708;

    cv::Mat rot = (cv::Mat_<double>(2,2) << cos(theta), -sin(theta), sin(theta), cos(theta));
    cv::Point trans = cv::Point(p.x, p.y);

    int xPad = width/2;
    int yPad = height/2;

    for(int c=0;c<pairs.size();c++){
        cv::Point point = transform (cv::Point(pairs[c][0].x-xPad, pairs[c][0].y-yPad), rot, trans, map->cols, map->rows);
        cv::Vec3b color = map->at<cv::Vec3b>(point.y, point.x);
        double r = color[0];
        double g = color[1];
        double b = color[2];
        vec3 color1(r, g, b);
        point = transform (cv::Point(pairs[c][1].x-xPad, pairs[c][1].y-yPad), rot, trans, map->cols, map->rows);
        color = map->at<cv::Vec3b>(point.y, point.x);
        r = color[0];
        g = color[1];
        b = color[2];
        vec3 color2(r, g, b);
        if(color1.x>color2.x) bin.push_back(1);
        else bin.push_back(0);
        if(color1.y>color2.y) bin.push_back(1);
        else bin.push_back(0);
        if(color1.z>color2.z) bin.push_back(1);
        else bin.push_back(0);
    }

    int diff = 0;
    for(int i= 0; i < droneDescriptor.size(); i++){
        if(bin[i] != droneDescriptor[i]) diff++;
    }
    return (float)((totalPairs*3)-diff)/(float)(totalPairs*3);
}

cv::Point BriefHeuristic::transform(cv::Point pt, cv::Mat rot, cv::Point trans, int max_x, int max_y){
    cv::Mat res(1,2,CV_64F);

    res.at<double>(0,0)=pt.x;
    res.at<double>(0,1)=pt.y;

    cv::Mat dst = res*rot;

    cv::Point point = cv::Point(dst.at<double>(0,0)+trans.x,dst.at<double>(0,1)+trans.y);

    if(point.x < 0) point.x = 0;
    else if(point.x > max_x) point.x = max_x;
    if(point.y < 0) point.y = 0;
    else if(point.y > max_y) point.y = max_y;

    return point;
}

double BriefHeuristic::calculateValue(int x, int y, cv::Mat *image, cv::Mat* map){
    return 0;
}
