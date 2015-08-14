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

//////////////////////
// Métodos Públicos //
//////////////////////

MCL::MCL(vector<MapGrid*> completeDensityMaps, Mat& gMap, string technique) : locTechnique(technique), globalMap(gMap)
{
    numParticles = 50;
    resamplingThreshold = 100;
    lastOdometry.x=0.0;
    lastOdometry.y=0.0;
    lastOdometry.theta=0.0;

    globalMap = gMap;

    particles.resize(numParticles);

//    realMap = completeDensityMaps[0];
    densityMaps = completeDensityMaps;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> randomX(0.25*globalMap.cols,0.75*globalMap.cols);
    std::uniform_real_distribution<double> randomY(0.25*globalMap.cols,0.75*globalMap.cols);
//    std::uniform_real_distribution<double> randomX(0.0,globalMap.cols);
//    std::uniform_real_distribution<double> randomY(0.0,globalMap.rows);
    std::uniform_real_distribution<double> randomTh(-M_PI,M_PI);

    // generate initial set
    for(int i=0; i<particles.size(); i++){

        bool valid = false;
        do{
            // sample particle pose
            particles[i].p.x = randomX(generator);
            particles[i].p.y = randomY(generator);
            particles[i].p.theta = randomTh(generator);

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

    if(locTechnique.compare("ssd")==0)
        strategyCode = 0;
    else if(locTechnique.compare("density")==0)
        strategyCode = 1;
}

MCL::~MCL()
{
    particleLog.close();
}

void MCL::draw(int x_aux, int y_aux, int halfWindowSize)
{
    int h = globalMap.rows;
    int w = globalMap.cols;

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

    // Draw particles
    for(int p=0;p<particles.size();p++){

        double x=particles[p].p.x;
        double y=particles[p].p.y;
        double th=particles[p].p.theta;


        // Draw point
        glColor3f(1.0,0.0,0.0);
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

bool MCL::run(Pose &u, Mat &z, vector<int> &densities, vector<double> &gradients, double time)
{
    double delta = sqrt(pow(u.x-lastOdometry.x,2)+pow(u.y-lastOdometry.y,2));
    if(delta<1.0)
        return false;

    cout << "Starting MCL" << endl;

    double elapsedTime, totalElapsedTime=0.0;
    struct timeval tstart, tend;

    // Start counting the elapsed time of this iteration
    gettimeofday(&tstart, NULL);

    sampling(u);
    if(strategyCode == 0)
       weightingSSD(z);
    if(strategyCode == 1)
       weightingDensity(densities,u,gradients);

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
    if(strategyCode == 2)
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

    double delta = sqrt(pow(u.x-lastOdometry.x,2)+pow(u.y-lastOdometry.y,2));
//    cout << "delta " << delta << endl;

    double angle = u.theta - lastOdometry.theta;

    for(int i=0; i<particles.size(); i++){
        particles[i].p.theta += angle + randomValue(generator)*3*M_PI/180.0;
        while(particles[i].p.theta > M_PI)
            particles[i].p.theta -= 2*M_PI;
        while(particles[i].p.theta < -M_PI)
            particles[i].p.theta += 2*M_PI;
        particles[i].p.x += delta*cos(particles[i].p.theta) + randomValue(generator)*0.25;
        particles[i].p.y += delta*sin(particles[i].p.theta) + randomValue(generator)*0.25;
    }
}

void MCL::weightingDensity(vector<int> &densities, Pose &u, vector<double> &gradients)
{
    double sumWeights = 0.0;
    double var = pow(60,2.0);

    // Evaluate all particles
    int count = 0;
    int minVal = INT_MAX;
    int maxVal = INT_MIN;
    for(int i=0; i<particles.size(); i++){
        int x=round(particles[i].p.x);
        int y=round(particles[i].p.y);
        for(int l=0; l<densities.size(); l=l+1) {
            if(densityMaps[l]->getHeuristicValue(x,y)<minVal && densityMaps[l]->getHeuristicValue(x,y)!= HEURISTIC_UNDEFINED_INT )
                minVal = densityMaps[l]->getHeuristicValue(x,y);
            if(densityMaps[l]->getHeuristicValue(x,y)>maxVal && densityMaps[l]->getHeuristicValue(x,y)!= HEURISTIC_UNDEFINED_INT )
                maxVal = densityMaps[l]->getHeuristicValue(x,y);

        }
    }
    cout << "MIN DENSITY: " << minVal << "  MAX DENSITY: " << maxVal << endl;
    cout << "HEURISTIC: " << densities[0] << endl;

    for(int i=0; i<particles.size(); i++){

        // check if particle is valid (known and not obstacle)
        if(!realMap->isKnown((int)particles[i].p.x,(int)particles[i].p.y) ||
           realMap->isObstacle((int)particles[i].p.x,(int)particles[i].p.y)){
            particles[i].w = 0.0;
            continue;
        }

        int x=round(particles[i].p.x);
        int y=round(particles[i].p.y);

        // Compute likelihood of each density
        double prob = 1.0;
        for(int l=0; l<densities.size(); l=l+1) {
            if(abs(densities[l]-densityMaps[l]->getHeuristicValue(x,y)) <= 40 || densities[l]==HEURISTIC_UNDEFINED_INT){
                prob *= 1.0;
                count++;
            }else{
                prob *= 0.5;
            }


//            if(densities[l]!=HEURISTIC_UNDEFINED_INT)
//                prob *= 1.0/(sqrt(2*M_PI*var))*exp(-0.5*(pow(densityMaps[l]->getHeuristicValue(x,y)-densities[l],2)/var));

//                if(densities[l] ==HEURISTIC_UNDEFINED_INT && densityMaps[l]->getHeuristicValue(x,y)==HEURISTIC_UNDEFINED_INT)
//                    prob = 1.0;
        }
        particles[i].w = prob;///(double)densities.size();
        sumWeights += particles[i].w;
    }
    //cout <<  "Entradas: " << count << endl;
    //cout << "SumWeights A " << sumWeights << endl;

    discardInvalidDeltaAngles(u,gradients);

    sumWeights = 0.0;
    for(int i=0; i<particles.size(); i++)
        sumWeights += particles[i].w;

    //cout << "SumWeights B " << sumWeights << endl;

    // Correct zero error
    if(sumWeights==0.0)
    {
        for(int i=0; i<particles.size(); i++) {
            // check if particle is valid (known and not obstacle)
            if(!realMap->isKnown((int)particles[i].p.x,(int)particles[i].p.y) ||
               realMap->isObstacle((int)particles[i].p.x,(int)particles[i].p.y)) {
                particles[i].w = 0.0;
            }
            else {
                particles[i].w = 1.0;
                sumWeights+=1.0;
            }
        }
    }
    cout << "Matei: " << count << " partículas." << endl;

    //normalize particles
    if(sumWeights!=0.0)
        for(int i=0; i<particles.size(); i++)
            particles[i].w /= sumWeights;
    else
        for(int i=0; i<particles.size(); i++)
            particles[i].w = 1.0/numParticles;


}

void MCL::weightingSSD(Mat &z_robot)
{
    double sumWeights = 0.0;

    // Evaluate all particles
    int count = 0;

    for(int i=0; i<particles.size(); i++){
        Mat z_particle = getParticleObservation(particles[i].p, z_robot.size());
        particles[i].w = evaluateParticleUsingSSD(z_robot, z_particle);

//        Mat window;
//        resize(z_particle,window,Size(0,0),0.2,0.2);
//        imshow("particle"+to_string(i), window );
    }

    //normalize particles
    if(sumWeights!=0.0)
        for(int i=0; i<particles.size(); i++)
            particles[i].w /= sumWeights;
    else
        for(int i=0; i<particles.size(); i++)
            particles[i].w = 1.0/numParticles;
}



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

    // update particles set
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

            double grad = densityMaps[h]->getOrientation(particles[i].p.x,particles[i].p.y);
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

Mat MCL::getParticleObservation(Pose p, Size2f s)
{
    RotatedRect rRect = RotatedRect(Point2f(p.x,p.y), s, RAD2DEG(p.theta)+90.0);
    Rect bRect = rRect.boundingRect();

    Mat source(bRect.height, bRect.width, CV_8UC3, Scalar(0));
    Point2f center(source.cols/2, source.rows/2);

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

    Mat image_roi = globalMap(bRect);

    Mat aux = source.colRange(xi,xf).rowRange(yi,yf);
    image_roi.copyTo(aux);

    // get angle and size from the bounding box
    float angle = rRect.angle;
    Size rect_size = rRect.size;
    if (rRect.angle < -45.) {
        angle += 90.0;
        swap(rect_size.width, rect_size.height);
    }

    // get the rotation matrix
    Mat M = getRotationMatrix2D(center, angle, 1.0);

    // perform the affine transformation
    Mat rotated;
    warpAffine(source, rotated, M, source.size(), INTER_CUBIC);

    // crop the resulting image
    Mat cropped;
    getRectSubPix(rotated, rect_size, center, cropped);

    if (rRect.angle < -45.) {
        transpose(cropped,cropped);
        flip(cropped,cropped,0);
    }


//    imshow("image_roi", image_roi);
//    imshow("rotated", rotated);
//    imshow("cropped", cropped);

//    Point2f vertices[4];
//    rRect.points(vertices);
//    for (int i = 0; i < 4; i++)
//        line(window, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));
//    rectangle(window, brect, Scalar(255,0,0));
//    imshow("rectangles", window);
//    waitKey(0);

    return cropped;
}

//////////////////////
/// Image Matching ///
//////////////////////

Mat MCL::img;
Mat MCL::templ;
Mat MCL::result;
int MCL::match_method;

double MCL::evaluateParticleUsingSSD(Mat& z_robot, Mat& z_particle)
{
    double w = -1;

    int max_Trackbar = 1;

    img = z_robot;
    templ = z_robot;
    match_method = 4;

    /// Create Trackbar
    char* trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
    createTrackbar(trackbar_label, "window", &match_method, max_Trackbar, MatchingMethod );

    MatchingMethod( 0, 0 );

    w = result.at<float>(0,0);

    return w;
}

/**
 * @function MatchingMethod
 * @brief Trackbar callback
 */
void MCL::MatchingMethod( int, void* )
{
  /// Source image to display
  Mat img_display;
  img.copyTo( img_display );

  /// Create the result matrix
  int result_cols =  img.cols - templ.cols + 1;
  int result_rows = img.rows - templ.rows + 1;

  result.create( result_rows, result_cols, CV_32FC1 );

//  double a, b;
//  minMaxLoc(img,&a,&b);
//  cout << "IMG min " << a << " max " << b << endl;
//  minMaxLoc(templ,&a,&b);
//  cout << "TEMPL min " << a << " max " << b << endl;

  /// Do the Matching and Normalize
  matchTemplate( img, templ, result, match_method );
  //normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

  /// Localizing the best match with minMaxLoc
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  Point matchLoc;

  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
  else
    { matchLoc = maxLoc; }

//  /// Show me what you got
//  rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
//  rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );

//  imshow( "image_window", img );
//  imshow( "result_window", templ );

//  waitKey(0);

  return;
}

