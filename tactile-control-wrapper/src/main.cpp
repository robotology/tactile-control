#include "tactileControlWrapper/HandControllerWrapper.h" 
#include <yarp/os/Network.h>

#include <sstream>
#include <string>

using yarp::os::Network;
using yarp::os::ResourceFinder;

int main(int argc, char * argv[])
{

    /* initializing yarp network */ 
    Network yarp;
    if (!yarp.checkNetwork()) {
        std::cerr << "Error: yarp server is not available. \n";
        return -1;
    }

    tactileControlWrapper::HandControllerWrapper handControllerWrapper;

    /* preparing and configuring the resource finder */
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("confTactileControlWrapper.ini");
    rf.setDefaultContext("tactileControlWrapper");
    rf.configure(argc, argv);

    /* starting module */
    handControllerWrapper.runModule(rf);

    return 0;
}
