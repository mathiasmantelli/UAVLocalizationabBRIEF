#include <pthread.h>
#include <iostream>
#include <iostream>
using std::cout;
using std::endl;
using std::cerr;
#include <cstring>
#include <fstream>
#include <stdexcept>      // std::invalid_argument

#include "SomeKernels.h"
#include "densityheuristic.h"
using namespace std;
using namespace cv;

#include "Robot.h"
//#include "PioneerRobot.h"
#include "DroneRobot.h"
#include "GlutClass.h"

ConnectionMode connectionMode;
LogMode logMode;

string filename;
pthread_mutex_t* mutex;

bool quiet;

void* startRobotThread (void* ref)
{
    Robot* robot=(Robot*) ref;

    robot->initialize(connectionMode, logMode, filename);

    while(robot->isRunning()){
        robot->run();
    }

	return NULL;
}

void* startGlutThread (void* ref)
{
    GlutClass* glut=GlutClass::getInstance();
    glut->setRobot((Robot*) ref);

    glut->initialize();

    glut->process();

	return NULL;
}

// Standard Input Error Message
bool errorMessage(int position=-1, std::string message="")
{
    // Detailed message
    if(message.size()!=0)
        cerr << message << endl;

    // Print character position if available
    if(position>0)
        cerr << "Failure at input element "<< position << endl;

    // Standard error message
    cerr << "Usage: PhiR2Framework -s <diff-intensity|diff-rgb|diff-cie1976|diff-cmc1984|diff-cie1994|diff-cie2000|diff-cie1994mix|diff-cie2000mix> <threshold> <gaussian|circular|inverted> <radius>" << endl;

    return false;
}

bool config(int argc, char* argv[], vector< heuristicType* > &heuristicTypes, std::string& mp, std::string& tp, std::string& op, int& start, int& finish)
{
    //data to collect
    Mat image;
    Mat map;
    std::string outputName;
    int p=1;

    quiet=false;

    while(p<argc)
    {
        // print help and exit
        if(!strncmp(argv[p], "-h", 2) || !strncmp(argv[p], "-H", 2) || !strncmp(argv[p], "--help", 6))
        {
            cout << "Usage: PhiR2Framework -e <path/> -o <path/outputDir/> -s <density|ssd|entropy> <diff-intensity|diff-rgb|diff-cie1976|diff-cmc1984|diff-cie1994|diff-cie2000|diff-cie1994mix|diff-cie2000mix> <double> <gaussian|circular|inverted> <int>" << endl;
            exit(0);
        }
        else if(!strncmp(argv[p], "-quiet", 6))
        {
            quiet=true;
            p++;
        }
        // Check if this is the filename part -- step 1
        else if(!strncmp(argv[p], "-o", 2) || !strncmp(argv[p], "-O", 2))
        {
            // check if there is a file name
            if(argc>p+3)
            {
                op=argv[p+1];
                start=stoi(argv[p+2]);
                finish=stoi(argv[p+3]);
                p+=4;
            }
            else
            {
                return errorMessage(p,"Failed to load output, missing argument");
            }
        }
        // Check if this is the filename part -- step 1
        else if(!strncmp(argv[p], "-e", 2) || !strncmp(argv[p], "-E", 2))
        {
            // check if there is a file name
            if(argc>p+1)
            {
                mp=argv[p+1];
                p+=2;
            }
            else
            {
                return errorMessage(p,"Failed to load map, missing argument");
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
                return errorMessage(p,"Failed to load trajectory, missing argument");
            }
        }
        else if(!strncmp(argv[p], "-s", 2) || !strncmp(argv[p], "-S", 2))
        {
            // initialize heuristic config
            heuristicType* ht = new heuristicType();

            /// check if there is an appropriate strategy type
            if(argc>=p+3)
            {
                // store strategy
                std::string s(argv[p+1]);
                if(s.compare("SSD")==0 || s.compare("ssd")==0)
                    ht->strategy=SSD;
                else if(s.compare("DENSITY")==0 || s.compare("density")==0)
                    ht->strategy=DENSITY;
                else if(s.compare("SDENSITY")==0 || s.compare("sdensity")==0)
                    ht->strategy=SINGLE_COLOR_DENSITY;
                else if(s.compare("MEANSHIFT")==0 || s.compare("meanshift")==0)
                    ht->strategy=MEAN_SHIFT;
                else if(s.compare("CREATE")==0 || s.compare("create")==0)
                    ht->strategy=CREATE_OBSERVATIONS;
                else if(s.compare("TEMPLATE")==0 || s.compare("template")==0)
                    ht->strategy=TEMPLATE_MATCHING;
                else if(s.compare("FEATURE")==0 || s.compare("feature")==0)
                    ht->strategy=FEATURE_MATCHING;
                else if(s.compare("COLOR")==0 || s.compare("color")==0)
                    ht->strategy=COLOR_ONLY;
                else if(s.compare("ENTROPY")==0 || s.compare("entropy")==0)
                    ht->strategy=ENTROPY;
                else if(s.compare("MI")==0 || s.compare("mi")==0)
                    ht->strategy=MUTUAL_INFORMATION;
                else
                    return errorMessage(p+1, "Invalid strategy: " + s);
            } else
                return errorMessage(p+1, "Insuficient strategy information:");

            /// Validate amd store color difference type
            std::string color_diff(argv[p+2]);
            if(color_diff.compare("diff-intensity")==0 || color_diff.compare("DIFF-INTENSITY")==0)
                ht->colorDifference = INTENSITYC;
            if (color_diff.compare("diff-rgb")==0 || color_diff.compare("DIFF-RGB")==0)
                ht->colorDifference = RGBNORMA;
            if(color_diff.compare("diff-cie1976")==0   || color_diff.compare("DIFF-CIE1976")==0)
                ht->colorDifference = CIELAB1976;
            if(color_diff.compare("diff-cmc1984")==0   || color_diff.compare("DIFF-CMC1984")==0)
                ht->colorDifference = CMCLAB1984;
            if(color_diff.compare("diff-cie1994")==0   || color_diff.compare("DIFF-CIE1994")==0)
                ht->colorDifference = CIELAB1994;
            if(color_diff.compare("diff-cie2000")==0   || color_diff.compare("DIFF-CIE2000")==0)
                ht->colorDifference = CIELAB2000;
            if(color_diff.compare("diff-cie1994mix")==0|| color_diff.compare("DIFF-CIE1994MIX")==0)
                ht->colorDifference = CIELAB1994MIX;
            if(color_diff.compare("diff-cie2000mix")==0|| color_diff.compare("DIFF-CIE2000MIX")==0)
                ht->colorDifference = CIELAB2000MIX;

            //Check for correct initialization
            if(ht->colorDifference == -1)
                return errorMessage(p+2, "Invalid color difference: " + color_diff);

            /// Check if this is the threshold for color difference is valid
            try {
                ht->threshold = atof(argv[p+3]);
            }
            catch (const std::invalid_argument& ia) {
                cerr << "Invalid argument: " << ia.what() << '\n';
                return errorMessage(p+3, "Color threshold is a double, example: -t 2.3\n");
            }

            // Catching silly color threshold
            if(ht->threshold<0)
                return errorMessage(p+3, "Color limiar cannot be negative: " + to_string(ht->threshold));

            //increasing the string position
            p+=4;

            // Check if we must check for kernel data
            if(ht->strategy == DENSITY ||
               ht->strategy == SINGLE_COLOR_DENSITY ||
               ht->strategy == MEAN_SHIFT ||
               ht->strategy == ENTROPY ||
               ht->strategy==MUTUAL_INFORMATION)
            {
                if(argc>=p+1) {

                    /// store kernel type
                    std::string kernel(argv[p]);
                    if(kernel.compare("Gaussian")==0 || kernel.compare("gaussian")==0)
                        ht->kernelType=KGAUSSIAN;
                    else if(kernel.compare("Inverted")==0 || kernel.compare("inverted")==0)
                        ht->kernelType=KANTIELIP;
                    else if(kernel.compare("Circular")==0 || kernel.compare("circular")==0)
                        ht->kernelType=KCIRCULAR;
                    else
                        return errorMessage(p, "Invalid kernel type: " + kernel);

                    /// check if there is an appropriate kernel radius
                    try {
                        ht->radius = atof(argv[p+1]);
                    }
                    catch (const std::invalid_argument& ia) {
                        cerr << "Invalid radius: " << ia.what() << endl;
                        return errorMessage(p+1, ", it must be an int.");
                    }
                    // Catching silly radius error
                    if(ht->radius<=0)
                        return errorMessage(p+1, "Invalid radius: " + to_string(ht->radius) + ",it must greater than 0.");

                    // Move to next argument
                    p+=2;


                }
                else // Density with missing arguments
                    return errorMessage(p, "Lacking arguments: density requires a kernel type and radius.");
            }

            if(ht->strategy==SINGLE_COLOR_DENSITY)
            {
                if(argc>=p) {
                    std::string color(argv[p]);
                    if(color.compare("WHITE")==0 || color.compare("white")==0)
                        ht->color=WHITE;
                    else if(color.compare("BLACK")==0 || color.compare("black")==0)
                        ht->color=BLACK;
                    else if(color.compare("RED")==0 || color.compare("red")==0)
                        ht->color=RED;
                    else if(color.compare("BLUE")==0 || color.compare("blue")==0)
                        ht->color=BLUE;
                    else if(color.compare("GREEN")==0 || color.compare("green")==0)
                        ht->color=GREEN;
                    else
                        return errorMessage(p, "Invalid color name: " + color);
                }
                else // Density with missing arguments
                    return errorMessage(p, "Lacking arguments: single_color_density requires a color value.");
            }
            // Store heuristic
            heuristicTypes.push_back(ht);
        }
        else
            p++;
    }
    //errorMessage(0, "Input error.");
    return true;
}


int main(int argc, char* argv[])
{
    // Global variables
    connectionMode = SIMULATION;
    logMode = NONE;
    filename = "";

    // load config from command line
    std::string mapPath, trajPath, outputPath;
    int start=-1;
    int finish=-1;

    vector< heuristicType* > heuristicTypes;
    if(!config(argc, argv, heuristicTypes, mapPath, trajPath, outputPath, start, finish))
        exit(1);

    Robot* r;
    // r = new Robot();
    // r = new PioneerRobot();
    r = new DroneRobot(mapPath,trajPath,heuristicTypes,quiet,outputPath,start,finish);

    if(quiet){
        startRobotThread((void*)r);
    }else{
        pthread_t robotThread, glutThread;
        mutex = new pthread_mutex_t;
        pthread_mutex_unlock(mutex);

        pthread_create(&(robotThread),NULL,startRobotThread,(void*)r);
        pthread_create(&(glutThread),NULL,startGlutThread,(void*)r);

        pthread_join(robotThread, 0);
        pthread_join(glutThread, 0);
    }

    return 0;
}
