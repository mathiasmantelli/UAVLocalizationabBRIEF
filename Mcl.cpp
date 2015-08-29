#include <random>
#include <chrono>
#include <limits>

#include "Mcl.h"
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>

#include <GL/glut.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include "DroneRobot.h"
#include "ColorCPU.h"
#include "RadiusVolumeTransferFunctions.h"

//////////////////////
// Métodos Públicos //
//////////////////////

MCL::MCL(vector<Heuristic*> &hVector, vector<MapGrid *> &cMaps, vector<Mat> &gMaps, Pose &initial):
    heuristics(hVector),
    heuristicValues(heuristics.size(),0.0),
    heuristicGradients(heuristics.size(),0.0),
    frameColorConverted(3),
    binaryFrameMask(),
    cachedMaps(cMaps),
    globalMaps(gMaps)
{
    numParticles = 500;
    resamplingThreshold = numParticles/8;
    lastOdometry.x=0.0;
    lastOdometry.y=0.0;
    lastOdometry.theta=0.0;

    particles.resize(numParticles);

    realPose = initial;
    odomPose = initial;

    std::default_random_engine generator;
//    std::uniform_real_distribution<double> randomX(0.25*globalMaps[0].cols,0.75*globalMaps[0].cols);
//    std::uniform_real_distribution<double> randomY(0.25*globalMaps[0].cols,0.75*globalMaps[0].cols);
    std::uniform_real_distribution<double> randomX(0.0,globalMaps[0].cols);
    std::uniform_real_distribution<double> randomY(0.0,globalMaps[0].rows);
    std::uniform_real_distribution<double> randomTh(-M_PI,M_PI);

    // generate initial set
    for(int i=0; i<particles.size(); i++){

        bool valid = false;
        do{

            // sample particle pose
            particles[i].p.x = randomX(generator);
            particles[i].p.y = randomY(generator);
            particles[i].p.theta = randomTh(generator);

            particles[i].p=initial;

            // check if particle is valid (known and not obstacle)
//            if(realMap->isKnown((int)particles[i].p.x,(int)particles[i].p.y) &&
//               !realMap->isObstacle((int)particles[i].p.x,(int)particles[i].p.y))
                valid = true;

        }while(!valid);

        cout << "Particle (" << i << ") " << RAD2DEG(particles[i].p.theta) << endl;
    }

    /******************** Prepare log file *****************************/
    time_t t = time(0);
    struct tm *now = localtime(&t);
    stringstream logName;

    logName << "Logs/mcl-" << -100+now->tm_year
                    << setfill('0') << setw(2) << 1+now->tm_mon
                    << setfill('0') << setw(2) << now->tm_mday << '-'
                    << setfill('0') << setw(2) << now->tm_hour
                    << setfill('0') << setw(2) << now->tm_min
                    << setfill('0') << setw(2) << now->tm_sec << ".txt";
    cout << logName.str() << endl; cout.flush();
    particleLog.open(logName.str().c_str(), std::fstream::out);
    particleLog << "trueX trueY meanPX meanPY closestx cloesty closestTh closestw meanParticleError meanParticleStdev meanError stdevError trueTh meanAngle angleStdev angleError stdevAngleError\n";
}

MCL::~MCL()
{
    particleLog.close();
}

void MCL::draw(int x_aux, int y_aux, int halfWindowSize)
{
    int h = globalMaps[0].rows;
    int w = globalMaps[0].cols;

    // Atualiza a região da janela
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    if(w > h){
        glOrtho (0 + x_aux + halfWindowSize,
                 w + x_aux - halfWindowSize,
                 h+(w-h)/2 + y_aux - halfWindowSize,
                 -(w-h)/2 + y_aux + halfWindowSize,
                 -1, 50);
    }else{
        glOrtho (-(h-w)/2 + x_aux + halfWindowSize,
                 w+(h-w)/2 + x_aux - halfWindowSize,
                 h + y_aux - halfWindowSize,
                 0 + y_aux + halfWindowSize,
                 -1, 50);
    }
    glMatrixMode (GL_MODELVIEW);

    glClearColor(1.0, 1.0, 1.0, 1.0);
    glClear (GL_COLOR_BUFFER_BIT);

    glColor3f(1.0f, 1.0f, 1.0f);

    glEnable(GL_TEXTURE_2D);
    // Draw map
    // Note: Window co-ordinates origin is top left, texture co-ordinate origin is bottom left.
    glBindTexture(GL_TEXTURE_2D, imageTex);
    glBegin(GL_QUADS);
        glTexCoord2f(1, 1);
        glVertex2f(0,  h);
        glTexCoord2f(0, 1);
        glVertex2f( w,  h);
        glTexCoord2f(0, 0);
        glVertex2f(w, 0);
        glTexCoord2f(1, 0);
        glVertex2f(0, 0);
    glEnd();
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);

    // Draw real path
    if(realPath.size() > 1){
        glLineWidth(3);
        glBegin( GL_LINE_STRIP );
        {
            glColor3f(0.0,0.0,1.0);
            for(unsigned int i=0;i<realPath.size()-1; i++){
                glVertex2f(realPath[i].x, realPath[i].y);
                glVertex2f(realPath[i+1].x, realPath[i+1].y);
            }
        }
        glEnd();
        glLineWidth(1);
    }

    // Draw odom path
    if(odomPath.size() > 1){
        glLineWidth(3);
        glBegin( GL_LINE_STRIP );
        {
            glColor3f(1.0,0.0,0.5);
            for(unsigned int i=0;i<odomPath.size()-1; i++){
                glVertex2f(odomPath[i].x, odomPath[i].y);
                glVertex2f(odomPath[i+1].x, odomPath[i+1].y);
            }
        }
        glEnd();
        glLineWidth(1);
    }

    // Draw real pose
    double x=realPose.x;
    double y=realPose.y;
    double th=realPose.theta;
    glColor3f(0.0,0.0,1.0);
    glPointSize(12);
    glBegin( GL_POINTS ); // point
    {
        glVertex2f(x, y);
    }
    glEnd();
    glColor3f(0.0, 0.0, 0.0);
    glLineWidth(2);
    glBegin( GL_LINES ); // direction
    {
        glVertex2f(x, y);
        glVertex2f(x+cos(th)*150, y+sin(th)*150);
    }
    glEnd();
    glLineWidth(1);

    // Draw odom pose
    x=odomPose.x;
    y=odomPose.y;
    th=odomPose.theta;
    glColor3f(1.0,0.0,0.5);
    glPointSize(12);
    glBegin( GL_POINTS ); // point
    {
        glVertex2f(x, y);
    }
    glEnd();
    glColor3f(0.0, 0.0, 0.0);
    glLineWidth(2);
    glBegin( GL_LINES ); // direction
    {
        glVertex2f(x, y);
        glVertex2f(x+cos(th)*150, y+sin(th)*150);
    }
    glEnd();
    glLineWidth(1);

    // Draw particles
    for(int p=0;p<particles.size();p++){

        double x=particles[p].p.x;
        double y=particles[p].p.y;
        double th=particles[p].p.theta;


        // Draw point
//        if(particles[p].w == 0.0)
            glColor3f(1.0,0.0,0.0);
//        else
//            glColor3f(1.0,1.0,0.0);

        glPointSize(6);
        glBegin( GL_POINTS );
        {
            glVertex2f(x, y);
        }
        glEnd();

        // Draw direction
        glColor3f(0.0, 0.0, 1.0);
        glLineWidth(2);
        glBegin( GL_LINES );
        {
            glVertex2f(x, y);
            glVertex2f(x+cos(th)*50, y+sin(th)*50);
        }
        glEnd();
        glLineWidth(1);

    }

    // Desenha robô
//    double xRobot = robot_->truePose_.x/robot_->scale;
//    double yRobot = robot_->truePose_.y/robot_->scale;
//    double angRobot = -robot_->truePose_.theta;

//    double xRobot = robot_->trueLocRX;//w/2;
//    double yRobot = h-robot_->trueLocRY; //h/2
//    double angRobot = -robot_->trueLocTh*180.0/M_PI;

//    glTranslatef(xRobot,yRobot,0.0);
//    drawRobot(angRobot, robot_->scale);
//    glTranslatef(-xRobot,-yRobot,0.0);
//    //printRealPos(xRobot, yRobot, 0.7, 0.0, 0.0);

//    if(robot_->slam->saveImage)
//        screenshot();

    glutSwapBuffers();
    glutPostRedisplay();
}

bool MCL::run(Pose &u, Mat &z, double time, Pose& real)
{
//    double delta = sqrt(pow(u.x-lastOdometry.x,2)+pow(u.y-lastOdometry.y,2));
//    if(delta<1.0)
//        return false;

    // Create different versions of the input image
    createColorVersions(z);

    realPose = real;
    realPath.push_back(realPose);

//    cout << "Starting MCL" << endl;

    double elapsedTime, totalElapsedTime=0.0;
    struct timeval tstart, tend;

    // Start counting the elapsed time of this iteration
    gettimeofday(&tstart, NULL);

    sampling(u);
    prepareWeighting();
    weighting(z,u);

//    if(locTechnique == SSD)
//       weightingSSD(z);
//    if(locTechnique == DENSITY)
//       weightingDensity(densities,u,gradients);
//    if(locTechnique == COLOR_ONLY)
//       weightingColor(); // receive color maps

    double sumWeights = 0.0;
    for(int i=0; i<particles.size(); i++)
        sumWeights += particles[i].w;


//    if(computeNeff() < numParticles/2.0)
    if(sumWeights!=0)
        resampling();

    lastOdometry = u;

    // Stop counting the elapsed time of this SLAM iteration
    gettimeofday(&tend, NULL);

    // Compute and print the elapsed time
    if (tstart.tv_usec > tend.tv_usec) {
        tend.tv_usec += 1000000;
        tend.tv_sec--;
    }
    // increment time for density due to himm cost
    if(locTechnique == DENSITY)
        elapsedTime+=time;

    elapsedTime = ((double)tend.tv_sec - (double)tstart.tv_sec) + ((double)tend.tv_usec - (double)tstart.tv_usec)/1000000.0;
    //cout << "elapsed time MCL: " << elapsedTime << endl;
    particleLog  <<  "elapsed time MCL: " << elapsedTime << endl;
    return true;
}

void MCL::writeErrorLogFile(double trueX, double trueY, double trueTh)
{
    /**************************************************************
     ******************  Position information  ********************
     **************************************************************/
    // For each particle compute the weighed mean position error
    double normalizer = 0.0;
    double meanParticleError = 0.0;
    double meanPX = 0.0;
    double meanPY = 0.0;

    double NEFF = 0.0;
    // Evaluate weighed mean particle
    for(int i=0; i<particles.size(); i++) {
        normalizer+=particles[i].w;
        NEFF+=particles[i].w*particles[i].w;
    }

    NEFF = 1/NEFF;

    if(normalizer==0.0)
        for(int i=0; i<particles.size(); i++)
            particles[i].w=1.0/particles.size();

    // Evaluate weighed mean particle
    for(int i=0; i<particles.size(); i++)
    {
        meanPX += particles[i].p.x*particles[i].w;
        meanPY += particles[i].p.y*particles[i].w;
        normalizer+=particles[i].w;
    }

    meanPX  /= normalizer;
    meanPY  /= normalizer;
    meanParticleError = computeError(trueX, trueY, meanPX, meanPY);

    // Evaluate weighed mean particle variance
    double meanParticleVar = 0.0;
    for(int i=0; i<particles.size(); i++)
    {
        meanParticleVar +=
        pow(computeError(meanPX, meanPY, particles[i].p.x, particles[i].p.y)-meanParticleError, 2.0)*particles[i].w;
    }
    meanParticleVar/=normalizer;
    double meanParticleStdev = sqrt(meanParticleVar);

    // Evaluate mean error
    double meanError = 0.0;
    MCLparticle closest=particles[0];
    double bestError = DBL_MAX;
    for(int i=0; i<particles.size(); i++){
        double x = particles[i].p.x;
        double y = particles[i].p.y;

        // get closest particle
        double error = computeError(trueX, trueY, x, y);
        if(error<bestError)
        {
            closest = particles[i];
            bestError = error;
        }
        // weighing the error
        error *= particles[i].w;
        meanError += error;
    }
    meanError/=normalizer;

    // Evaluate mean error variance
    double varError = 0.0;
    for(int i=0; i<particles.size(); i++){
        double x = particles[i].p.x;
        double y = particles[i].p.y;
        varError += pow(computeError(trueX, trueY, x, y)-meanError, 2.0)*particles[i].w;
    }
    varError = varError/normalizer;
    double stdevError = sqrt(varError);

    /**************************************************************
     ********************  Angle information  *********************
     **************************************************************/
    // For each particle compute the weighed mean angle
    double meanAngle  = 0.0;
    double meanAngle2 = 0.0;
    for(int i=0; i<particles.size(); i++){
        meanAngle   += particles[i].p.theta*particles[i].w;
        meanAngle2  += particles[i].p.theta*particles[i].p.theta*particles[i].w;
    }
    meanAngle  /= normalizer;
    while (meanAngle > M_PI)
        meanAngle -= 2*M_PI;
    while (meanAngle < -M_PI)
        meanAngle += 2*M_PI;

    // For each particle compute the mean particle angle var and stdev
    double angleVar = 0.0;
    for(int i=0; i<particles.size(); i++){
        angleVar   += pow((particles[i].p.theta-meanAngle), 2.0)*particles[i].w;
    }
    angleVar/=normalizer;
    double angleStdev = sqrt(angleVar);
    while (angleStdev > M_PI)
        angleStdev -= 2*M_PI;
    while (angleStdev < -M_PI)
        angleStdev += 2*M_PI;
    angleVar = angleStdev*angleStdev;


    // Compute the mean angle error
    double angleError = computeAngleError(trueTh, meanAngle);
//    angleError = 0.0;
//    for(int i=0; i<particles.size(); i++){
//        angleError   += (particles[i].p.theta-trueTh)*particles[i].w;
//    }
    angleError/=normalizer;
    while (angleError > M_PI)
        angleError -= 2*M_PI;
    while (angleError < -M_PI)
        angleError += 2*M_PI;

    // Compute the angle var error
    double varAngleError = 0.0;
    //cout << angleError << endl;
    for(int i=0; i<particles.size(); i++){
        varAngleError   += pow((particles[i].p.theta-trueTh)-angleError, 2.0)*particles[i].w;
    }
    varAngleError/=normalizer;
    double stdevAngleError = sqrt(varAngleError);
    while (stdevAngleError > M_PI)
        stdevAngleError -= 2*M_PI;
    while (stdevAngleError < -M_PI)
        stdevAngleError += 2*M_PI;
    varAngleError = stdevAngleError*stdevAngleError;



    //particleLog << "trueX trueY meanPX meanPY closestx cloesty closestw meanParticleError meanParticleStdev meanError stdevError trueTh meanAngle angleStdev angleError stdevAngleError\n";
    particleLog  << "Standard Measurements: " << trueX <<  " " << trueY << " " << " " << meanPX << " " << meanPY << " "
                 <<  closest.p.x << " " << closest.p.y << " " << closest.p.theta << " " << closest.w
                 << meanParticleError <<  " " << meanParticleStdev  << " " << meanError << " " << stdevError << " "
                 << trueTh << " " << meanAngle << " " << angleStdev << " " << angleError << " " << stdevAngleError << " "
                 << NEFF << endl;
    particleLog.flush();

}

//////////////////////
// Métodos Privados //
//////////////////////

void MCL::sampling(Pose &u)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_real_distribution<double> randomValue(-1.0,1.0);

    odomPose.x += cos(odomPose.theta)*u.x - sin(odomPose.theta)*u.y;
    odomPose.y += sin(odomPose.theta)*u.x + cos(odomPose.theta)*u.y;
    odomPose.theta += u.theta;
    while(odomPose.theta > M_PI)
        odomPose.theta -= 2*M_PI;
    while(odomPose.theta < -M_PI)
        odomPose.theta += 2*M_PI;

    odomPath.push_back(odomPose);

//    cout << "Real Pose " << realPose << endl;
//    cout << "Odom Pose " << odomPose << endl;

    for(int i=0; i<particles.size(); i++){
        particles[i].p.x += cos(particles[i].p.theta)*u.x - sin(particles[i].p.theta)*u.y + randomValue(generator)*3.0;
        particles[i].p.y += sin(particles[i].p.theta)*u.x + cos(particles[i].p.theta)*u.y + randomValue(generator)*3.0;
        particles[i].p.theta += u.theta + randomValue(generator)*3*M_PI/180.0;
        while(particles[i].p.theta > M_PI)
            particles[i].p.theta -= 2*M_PI;
        while(particles[i].p.theta < -M_PI)
            particles[i].p.theta += 2*M_PI;
    }
}

void MCL::weighting(Mat& z_robot, Pose &u)
{
    int count = 0;

    for(int i=0;i<particles.size();++i)
    {
        int x=round(particles[i].p.x);
        int y=round(particles[i].p.y);

        // check if particle is not valid (unknown or obstacle)
        if(!cachedMaps[0]->isKnown(x,y) || cachedMaps[0]->isObstacle(x,y)){
            particles[i].w = 0.0;
            count++;
            continue;
        }

        double varColor = pow(3.0, 2.0); // normalized gaussian
        double varDensity = pow(0.1,2.0); //10%
        double varEntropy = pow(0.1,2.0); //10%
        double prob=1.0;

        for(int l=0;l<heuristics.size();++l)
        {
            Heuristic* h = heuristics[l];
            int mapID = selectMapID(h->getColorDifference());

            switch(h->getType())
            {
                case SSD:
                {
                    Mat z_particle = Utils::getRotatedROIFromImage(particles[i].p, z_robot.size(), globalMaps[0]);
                    prob = Utils::matchImages(z_robot,z_particle,CV_TM_CCORR_NORMED);
                    break;
                }
                case COLOR_ONLY:
                {
                    /// compute color difference
                    double diff  = h->calculateValue(x,y,&globalMaps[mapID]);

                    /// Gaussian weighing
                    if(diff!=HEURISTIC_UNDEFINED)
                        prob *= 1.0/(sqrt(2*M_PI*varColor))*exp(-0.5*(pow(diff,2)/varColor));
                    else
                        prob *= 1.0/(numParticles);
                    break;
                }
                case DENSITY:
                {
                    /// Gaussian weighing
                    if(heuristicValues[l]!=HEURISTIC_UNDEFINED)
                        prob *= 1.0/(sqrt(2*M_PI*varDensity))*exp(-0.5*(pow((cachedMaps[l]->getPureHeuristicValue(x,y)-heuristicValues[l]),2)/varDensity));

                    /// Hyperbolic secant^5 weighing
                    //if(densities[l]!=HEURISTIC_UNDEFINED_INT) {
                    //    double val = abs(densityMaps[l]->getHeuristicValue(x,y)-densities[l])/150.0;
                    //    prob *= (1/cosh(val*val*val*val*val));
                    //}

                    if(heuristicValues[l]==HEURISTIC_UNDEFINED && cachedMaps[l]->getHeuristicValue(x,y)==HEURISTIC_UNDEFINED_INT)
                        prob *= 1.0/numParticles;
                    break;
                }
                case ENTROPY:
                {
                    /// Gaussian weighing
                    if(heuristicValues[l]!=HEURISTIC_UNDEFINED)
                        prob *= 1.0/(sqrt(2*M_PI*varEntropy))*exp(-0.5*(pow((cachedMaps[l]->getPureHeuristicValue(x,y)-heuristicValues[l]),2)/varEntropy));

                    /// Hyperbolic secant^5 weighing
                    //if(densities[l]!=HEURISTIC_UNDEFINED_INT) {
                    //    double val = abs(densityMaps[l]->getHeuristicValue(x,y)-densities[l])/150.0;
                    //    prob *= (1/cosh(val*val*val*val*val));
                    //}

                    if(heuristicValues[l]==HEURISTIC_UNDEFINED && cachedMaps[l]->getHeuristicValue(x,y)==HEURISTIC_UNDEFINED_INT)
                        prob *= 1.0/numParticles;
                    break;
                }
            }
        }

        particles[i].w = prob;
    }

    cout << "Matei: " << count << " partículas." << endl;

    /// FALTA ARRUMAR ESSA FUNCAO
    //discardInvalidDeltaAngles(u,gradients);

    double sumWeights = 0.0;
    for(int i=0; i<particles.size(); i++)
        sumWeights += particles[i].w;
    cout << "SumWeights B " << sumWeights << endl;

    // Correct zero error
    if(sumWeights==0.0)
    {
        for(int i=0; i<particles.size(); i++) {
            // check if particle is valid (known and not obstacle)
            if(!cachedMaps[0]->isKnown((int)particles[i].p.x,(int)particles[i].p.y) ||
               cachedMaps[0]->isObstacle((int)particles[i].p.x,(int)particles[i].p.y)) {
                particles[i].w = 0.0;
            }
            else {
                particles[i].w = 1.0;
                sumWeights+=1.0;
            }
        }
    }

    count = 0;
    neff = 0;

    //normalize particles
    if(sumWeights!=0.0)
        for(int i=0; i<particles.size(); i++){
            particles[i].w /= sumWeights;

            if(particles[i].w == 0.0)
                count++;

            neff+=pow(particles[i].w,2.0);
        }
    else {
        for(int i=0; i<particles.size(); i++)
            particles[i].w = 1.0/numParticles;
        neff=numParticles;
    }
    cout << "Confirmando, matei: " << count << " partículas." << endl;
    neff=1.0/neff;
    cout << "NEFF: " << neff << endl;
}


//void MCL::weightingColor()
//{
//    double sumWeights = 0.0;
//    double var = pow(3.0, 2.0); // normalized gaussian


//    for(int i=0;i<particles.size();++i)
//    {
//        double x = particles[i].p.x;
//        double y = particles[i].p.y;
//        double prob=1.0;

//        for(int s=0;s<colorHeuristics.size();++s)
//        {

//            ColorHeuristic* h = colorHeuristics[s];
//            /// compute color difference
//            int mapID = selectMapID(h->getColorDifference());
//            double diff  = h->calculateValue(int(round(x)),int(round(y)),&globalMaps[mapID]);

//            /// Gaussian weighing
//            if(diff!=HEURISTIC_UNDEFINED)
//                prob *= 1.0/(sqrt(2*M_PI*var))*exp(-0.5*(pow(diff,2)/var));
//            else
//                prob *= 1.0/(numParticles);
//        }
//        // sum weights
//        particles[i].w *= prob;
//        sumWeights += particles[i].w;
//    }

//    //normalize particles
//    if(sumWeights!=0.0)
//        for(int i=0; i<particles.size(); i++)
//            particles[i].w /= sumWeights;
//    else
//        for(int i=0; i<particles.size(); i++)
//            particles[i].w = 1.0/numParticles;
//}

//void MCL::weightingDensity(vector<int> &densities, Pose &u, vector<double> &gradients)
//{
//    double sumWeights = 0.0;
//    double var = pow(51.5,2.0); //10% of 255

//    // Compute min and max density value (for some reason)
//    int minVal = INT_MAX;
//    int maxVal = INT_MIN;
//    for(int i=0; i<particles.size(); i++){
//        int x=round(particles[i].p.x);
//        int y=round(particles[i].p.y);
//        for(int l=0; l<cachedMaps.size(); ++l) {
//            if(cachedMaps[l]->getHeuristicValue(x,y)<minVal && cachedMaps[l]->getHeuristicValue(x,y)!= HEURISTIC_UNDEFINED_INT )
//                minVal = cachedMaps[l]->getHeuristicValue(x,y);
//            if(cachedMaps[l]->getHeuristicValue(x,y)>maxVal && cachedMaps[l]->getHeuristicValue(x,y)!= HEURISTIC_UNDEFINED_INT )
//                maxVal = cachedMaps[l]->getHeuristicValue(x,y);
//        }
//    }
//    cout << "MIN DENSITY: " << minVal << "  MAX DENSITY: " << maxVal << endl;
////    for(int j=0;j<densities.size();++j)
////        cout << "HEURISTIC: " << densities[j] << " ";
////    cout << endl;

//    int count = 0;
////    vector<int> acounter(densities.size(),0);
//    for(int i=0; i<particles.size(); i++){
//        // check if particle is valid (known and not obstacle)
//        if(!cachedMaps[0]->isKnown((int)particles[i].p.x,(int)particles[i].p.y) ||
//           cachedMaps[0]->isObstacle((int)particles[i].p.x,(int)particles[i].p.y)){
//            particles[i].w = 0.0;
//            count++;
//            continue;
//        }

//        int x=round(particles[i].p.x);
//        int y=round(particles[i].p.y);

//        // Compute likelihood of each density
//        double prob = 1.0;
//        bool ag = false;
//        for(int l=0; l<densities.size(); l=l+1) {
/////*** Filtering using simple 1o 0.5 probability accoding to a threshold ***/
///// Filter out some density values
////           if(abs((densities[l]>100?densities[l]:densities[l]) -
////                   (densityMaps[l]->getHeuristicValue(x,y)>100 ?
////                    densityMaps[l]->getHeuristicValue(x,y) : 0)) <= 40 || densities[l]==HEURISTIC_UNDEFINED_INT)
////           {
///// Unfiltered densities
////            if(abs(densities[l]-densityMaps[l]->getHeuristicValue(x,y)) <= 10 || densities[l]==HEURISTIC_UNDEFINED_INT){
////                prob *= 1.0;
////                count++;
////            }else{
////                acounter[l]++;
////                prob *= 0.5;
////            }

///// Gaussian weighing
//            if(densities[l]!=HEURISTIC_UNDEFINED_INT)
//                prob *= 1.0/(sqrt(2*M_PI*var))*exp(-0.5*(pow((cachedMaps[l]->getHeuristicValue(x,y)-densities[l]),2)/var));

///// Hyperbolic secant^5 weighing
////            if(densities[l]!=HEURISTIC_UNDEFINED_INT) {
////                double val = abs(densityMaps[l]->getHeuristicValue(x,y)-densities[l])/150.0;
////                prob *= (1/cosh(val*val*val*val*val));
////            }

//            if(densities[l] ==HEURISTIC_UNDEFINED_INT && cachedMaps[l]->getHeuristicValue(x,y)==HEURISTIC_UNDEFINED_INT)
//                    prob *= 1.0/numParticles;
//        }
//        particles[i].w = prob;///(double)densities.size();
//        sumWeights += particles[i].w;
//    }
////    cout << "Agressividade: ";
////    for (int i=0;i<acounter.size();++i)
////        cout << acounter[i] << " ";
////    cout << endl;

//    //cout <<  "Entradas: " << count << endl;
//    //cout << "SumWeights A " << sumWeights << endl;

//    discardInvalidDeltaAngles(u,gradients);

//    sumWeights = 0.0;
//    for(int i=0; i<particles.size(); i++)
//        sumWeights += particles[i].w;

//    cout << "SumWeights B " << sumWeights << endl;

//    // Correct zero error
//    if(sumWeights==0.0)
//    {
//        for(int i=0; i<particles.size(); i++) {
//            // check if particle is valid (known and not obstacle)
//            if(!cachedMaps[0]->isKnown((int)particles[i].p.x,(int)particles[i].p.y) ||
//               cachedMaps[0]->isObstacle((int)particles[i].p.x,(int)particles[i].p.y)) {
//                particles[i].w = 0.0;
//            }
//            else {
//                particles[i].w = 1.0;
//                sumWeights+=1.0;
//            }
//        }
//    }
//    cout << "Matei: " << count << " partículas." << endl;

//    count = 0;

//    neff = 0;

//    //normalize particles
//    if(sumWeights!=0.0)
//        for(int i=0; i<particles.size(); i++){
//            particles[i].w /= sumWeights;

//            if(particles[i].w == 0.0)
//                count++;

//            neff+=pow(particles[i].w,2.0);
//        }
//    else {
//        for(int i=0; i<particles.size(); i++)
//            particles[i].w = 1.0/numParticles;
//        neff=numParticles;
//    }
//    cout << "Confirmando Matei: " << count << " partículas." << endl;
//    neff=1.0/neff;
//    cout << "NEFF: " << neff << endl;

//}

//void MCL::weightingSSD(Mat &z_robot)
//{
//    double sumWeights = 0.0;

//    // Evaluate all particles
//    int count = 0;
//    int globalMapID = 0;

//    for(int i=0; i<particles.size(); i++){
//        Mat z_particle = Utils::getRotatedROIFromImage(particles[i].p, z_robot.size(), globalMaps[globalMapID]);
//        particles[i].w = Utils::matchImages(z_robot,z_particle,CV_TM_CCORR_NORMED);
//        sumWeights+= particles[i].w;
////        Mat window;
////        resize(z_particle,window,Size(0,0),0.2,0.2);
////        imshow("particle"+to_string(i), window );
//    }

//    //normalize particles
//    if(sumWeights!=0.0)
//        for(int i=0; i<particles.size(); i++)
//            particles[i].w /= sumWeights;
//    else
//        for(int i=0; i<particles.size(); i++)
//            particles[i].w = 1.0/numParticles;
//}

void MCL::resampling()
{
    vector<int> children;
    children.resize(numParticles,0);

    // low variance sampler (table 4.4 in Thrun, 2005)

    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
//    std::default_random_engine generator;

    std::uniform_real_distribution<double> randomValue(0.0,1.0/((double)numParticles));

    int i=0;
    double r=randomValue(generator);
    double c=particles[i].w;

    for(int m=0; m<particles.size(); m++){
        double U = r + m*1.0/(double)numParticles;
        while(U>c){
            i++;
            c+=particles[i].w;
        }
        children[i]++;
    }


    // generate children from current particles
    vector<MCLparticle> nextGeneration;
//    cout << "Children ";
    for(int m=0; m<particles.size(); m++){
//        //cout << children[m] << ' ';
        for(int c=0; c<children[m]; c++)
            nextGeneration.push_back(particles[m]);
    }
//    cout << " size nextGeneration " << nextGeneration.size();


    particles = nextGeneration;

}

////////////////////////
// Métodos Auxiliares //
////////////////////////

void MCL::discardInvalidDeltaAngles(Pose &u, vector<double> &gradients)
{
    if(gradients.size()<=1)
        return;

    vector<bool> keep(particles.size(),false);

    // for each density map
    for(int h=0; h<gradients.size(); h++){
        if(IS_UNDEF(gradients[h]))
            continue;

        double deltaRobot = u.theta - gradients[h];
//        cout << " robot " << RAD2DEG(u.theta) << " - " << RAD2DEG(gradients[h]) << " = " << RAD2DEG(deltaRobot) << endl;

        // Evaluate all particles
        for(int i=0; i<particles.size(); i++){
            if(particles[i].w == 0.0)
                continue;

            double grad = cachedMaps[h]->getOrientation(particles[i].p.x,particles[i].p.y);
            if(IS_UNDEF(grad)){
                continue;
            }
//            cout << " particle (" << i << ") " << RAD2DEG(particles[i].p.theta) << " - " << RAD2DEG(grad);
            grad = particles[i].p.theta - grad;
//            cout << " = " << RAD2DEG(grad);

            double delta = acos(cos(grad)*cos(deltaRobot) + sin(grad)*sin(deltaRobot));
            if(RAD2DEG(delta)<5.0)
                keep[i]=true;
//            cout << " DELTA: " << RAD2DEG(delta) << " w " << 0.5 + 0.5*cos(delta) << endl;

//            particles[i].w *= 0.5 + 0.5*cos(delta);
        }
    }

    int count=0;
    for(int i=0; i<particles.size(); i++){
        if(keep[i])
            count++;
    }

    if(count<particles.size()/4) // Avoid discard if robot cagated
        return;

    for(int i=0; i<particles.size(); i++){
        if(!keep[i])
            particles[i].w = 0.0;
    }


}

double MCL::computeNeff()
{
    double Neff=0.0;
    for(int i=0; i<particles.size(); i++)
        Neff += particles[i].w * particles[i].w;
    return 1.0/Neff;
}

double MCL::computeError(double trueX, double trueY,double particleX, double particleY)
{
    return sqrt(pow(trueX-particleX, 2.0) + pow(trueY-particleY, 2.0));
}

double MCL::computeAngleError(double trueTh, double particleTh)
{
    //cout <<  "trueTh: " << trueTh << " " "particleTh: " << particleTh << endl;
    //return acos(cos(trueTh)*cos(particleTh)+sin(trueTh)*sin(particleTh));
    double angleError=particleTh-trueTh;

    while (angleError > M_PI)
        angleError -= 2*M_PI;
    while (angleError < -M_PI)
        angleError += 2*M_PI;

    // error in radians
    return angleError;

}
double MCL::sumAngles(double a, double b)
{
    //cout <<  "trueTh: " << trueTh << " " "particleTh: " << particleTh << endl;
    //return acos(cos(trueTh)*cos(particleTh)+sin(trueTh)*sin(particleTh));
    double c = a + b;

    while (c > M_PI)
        c -= 2*M_PI;
    while (c < -M_PI)
        c += 2*M_PI;

    // error in radians
    return c;

}

void MCL::prepareWeighting()
{
    // precomputing halfRows e halfCows
    int halfRows = frameColorConverted[0].rows/2;
    int halfCols = frameColorConverted[0].cols/2;

    // Set mask if not set yet
    if(!binaryFrameMask.data)
        binaryFrameMask = Mat(frameColorConverted[0].cols,
                              frameColorConverted[0].rows,
                              CV_8SC3,Scalar(0,0,0));

    // Compute value at the center of the frame using
    // appropriate color space
    for(int c = 0; c<heuristics.size();++c)
    {
        Heuristic* h = heuristics[c];

        // get proper color space
        int mapID = selectMapID(h->getColorDifference());

        double val = 0.0;
        double grad = val;

        switch(h->getType())
        {
        case SSD:
            break;
        case COLOR_ONLY:
            {
            ColorHeuristic* ch = (ColorHeuristic*) heuristics[c];
            ch->setBaselineColor(halfCols,
                                 halfRows,
                                 &frameColorConverted[mapID]);
            break;
            }
        case DENSITY:
        case ENTROPY:
        case MUTUAL_INFORMATION:
            {
            // create discrete density value according to the corresonding mapgrid
            heuristicValues[c] = heuristics[c]->calculateValue(
                        halfCols,
                        halfRows,
                        &frameColorConverted[mapID], &binaryFrameMask);
            // and do the same for the angles
            heuristicGradients[c] = heuristics[c]->calculateGradientSobelOrientation(
                        halfCols,
                        halfRows,
                        &frameColorConverted[mapID], &binaryFrameMask);
            break;
            }
        }
    }
}
void MCL::createColorVersions(Mat& imageRGB)
{
    // RGB
    frameColorConverted[0]=imageRGB.clone();

    // INTENSITYC:
    cvtColor(imageRGB, frameColorConverted[1],CV_BGR2GRAY);
    cvtColor(frameColorConverted[1], frameColorConverted[1],CV_GRAY2BGR);

    // CIELAB1976 || CIELAB1994 || CMCLAB1984 || CIELAB2000 || CIELAB1994MIX || CIELAB2000MIX
    imageRGB.convertTo(frameColorConverted[2], CV_32F);
    frameColorConverted[2]*=1/255.0;
    cvtColor(imageRGB, frameColorConverted[2], CV_BGR2Lab);
}
