#include "BriefHeuristic.h"

BriefHeuristic::BriefHeuristic(STRATEGY s, int id, int cd, double l):Heuristic(s,id,cd,l)
{
    totalPairs = 1000;
    lowThreshold = 0.5;
    multiplierThreshold = 4;
    margin = 10;

}

void BriefHeuristic::printInfo(){
    cout << endl << "Brief parameters:\nPairs:\t\t" << totalPairs << "\nLow Threshold:\t" << lowThreshold << "\nMultiplier:\t" << multiplierThreshold << "\nMargin:\t\t" << margin << endl << endl;
}

void BriefHeuristic::updateDroneDescriptor(cv::Mat& drone){
    width = drone.cols;
    height = drone.rows;

    //imshow("d", drone);

    if(pairs.size() < 1){
        for(int p=0; p<totalPairs; p++){
            vector<cv::Point> pair(2);
            pairs.push_back(pair);
        }
    }

    int halfMargin = margin/2;
    cv::RNG rng;
    for(int x=0;x<totalPairs;x++){
        pairs[x][0] = cv::Point((int)rng.uniform(halfMargin, drone.cols-halfMargin), (int)rng.uniform(halfMargin, drone.rows-halfMargin));
        pairs[x][1] = cv::Point((int)rng.uniform(halfMargin, drone.cols-halfMargin), (int)rng.uniform(halfMargin, drone.rows-halfMargin));
    }

    vector<int> bin;

    for(int c=0;c<pairs.size();c++){
        vec3 color1(getValuefromPixel(pairs[c][0].x,pairs[c][0].y,&drone));
        vec3 color2(getValuefromPixel(pairs[c][1].x,pairs[c][1].y,&drone));
        if(color_difference!=INTENSITYC) {
            if(color1.r>color2.r) bin.push_back(1);
            else bin.push_back(0);
            if(color1.g>color2.g) bin.push_back(1);
            else bin.push_back(0);
            if(color1.b>color2.b) bin.push_back(1);
            else bin.push_back(0);
        }
        else {
            if(color1.g>color2.g) bin.push_back(1);
            else bin.push_back(0);
        }
    }

    droneDescriptor = bin;
}

double BriefHeuristic::calculateValue2(Pose p, cv::Mat *map){
    vector<int> bin;

    float theta = p.theta*(-1)-1.5708;

    int xPad = width/2;
    int yPad = height/2;
    if(color_difference != INTENSITYC){
        for(int c=0;c<pairs.size();c++){
            cv::Point point1 = cv::Point(((pairs[c][0].x-xPad)*cos(theta)+(pairs[c][0].y-yPad)*sin(theta))+p.x,
                                        ((pairs[c][0].x-xPad)*-sin(theta)+(pairs[c][0].y-yPad)*cos(theta))+p.y);
            if(point1.x < 0 || point1.x >= map->cols || point1.y < 0 || point1.y >= map->rows){
                bin.push_back(HEURISTIC_UNDEFINED_INT);
                bin.push_back(HEURISTIC_UNDEFINED_INT);
                bin.push_back(HEURISTIC_UNDEFINED_INT);
            }
            else{
                cv::Point point2 = cv::Point(((pairs[c][1].x-xPad)*cos(theta)+(pairs[c][1].y-yPad)*sin(theta))+p.x, ((pairs[c][1].x-xPad)*-sin(theta)+(pairs[c][1].y-yPad)*cos(theta))+p.y);
                if(point2.x < 0 || point2.x >= map->cols || point2.y < 0 || point2.y >= map->rows){
                    bin.push_back(HEURISTIC_UNDEFINED_INT);
                    bin.push_back(HEURISTIC_UNDEFINED_INT);
                    bin.push_back(HEURISTIC_UNDEFINED_INT);
                }
                else{
                    cv::Vec3b color1 = map->at<cv::Vec3b>(point1.y, point1.x);
//                    double r = color[0];
//                    double g = color[1];
//                    double b = color[2];
//                    vec3 color1(r, g, b);
                    cv::Vec3b color2 = map->at<cv::Vec3b>(point2.y, point2.x);
//                    r = color[0];
//                    g = color[1];
//                    b = color[2];
//                    vec3 color2(r, g, b);
                    if(color1[0]>color2[0]) bin.push_back(1);
                    else bin.push_back(0);
                    if(color1[1]>color2[1]) bin.push_back(1);
                    else bin.push_back(0);
                    if(color1[2]>color2[2]) bin.push_back(1);
                    else bin.push_back(0);
                }
            }
        }
    }else{
        for(int c=0;c<pairs.size();c++){
            cv::Point point1 = cv::Point(((pairs[c][0].x-xPad)*cos(theta)+(pairs[c][0].y-yPad)*sin(theta))+p.x, ((pairs[c][0].x-xPad)*-sin(theta)+(pairs[c][0].y-yPad)*cos(theta))+p.y);
            if(point1.x < 0 || point1.x >= map->cols || point1.y < 0 || point1.y >= map->rows){
                bin.push_back(HEURISTIC_UNDEFINED_INT);
            }
            else{
                cv::Point point2 = cv::Point(((pairs[c][1].x-xPad)*cos(theta)+(pairs[c][1].y-yPad)*sin(theta))+p.x, ((pairs[c][1].x-xPad)*-sin(theta)+(pairs[c][1].y-yPad)*cos(theta))+p.y);
                if(point2.x < 0 || point2.x >= map->cols || point2.y < 0 || point2.y >= map->rows){
                    bin.push_back(HEURISTIC_UNDEFINED_INT);
                }
                else{
                    cv::Vec3b color1 = map->at<cv::Vec3b>(point1.y, point1.x);
//                    double g1 = color[1];

                    cv::Vec3b color2 = map->at<cv::Vec3b>(point2.y, point2.x);
//                    double g2 = color[1];

                    if(color1[0]>color2[0]) bin.push_back(1);
                    else bin.push_back(0);
                }
            }
        }
    }

    int diff = 0;
    for(int i= 0; i < droneDescriptor.size(); i++){
        if(bin[i] == HEURISTIC_UNDEFINED_INT || bin[i] != droneDescriptor[i]) diff++;
    }
    if(color_difference != INTENSITYC)
        return (float)((totalPairs*3)-diff)/(float)(totalPairs*3);
    else
        return (float)((totalPairs)-diff)/(float)(totalPairs);

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
