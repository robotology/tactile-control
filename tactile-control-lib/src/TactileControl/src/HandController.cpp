#include "TactileControl/HandController.h"

#include "TactileControl/data/Parameters.h"

#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Bottle.h>

using tactileControl::HandController;

HandController::HandController(){

    controllerConfigured = false;
    taskRunning = false;

    dbgTag = "HandController: ";
}

bool HandController::open(){

    yarp::os::Property options;
    return open(options);
}

bool HandController::open(const yarp::os::Property &options){
    using std::cout;

    if (controllerConfigured) return false;

    // initialize data
    taskData = new TaskData();
    if (!taskData->init(options)){
        cout << dbgTag << "failed to initialize data\n";
        return false;
    }

    // initialize motor interfaces
    controllerUtil = new ControllerUtil();
    if (!controllerUtil->init(taskData)){
        cout << dbgTag << "failed to initialize motor interfaces\n";
        return false;
    }

    // initialize yarp ports
    portUtil = new PortUtil();
    if (!portUtil->init(taskData)){
        cout << dbgTag << "failed to initialize yarp ports\n";
        return false;
    }

    // start task thread
    taskThread = new TaskThread(taskData->getInt(PAR_COMMON_TASK_THREAD_PERIOD),taskData,controllerUtil,portUtil);
    if (!taskThread->start()) {
        cout << dbgTag << "could not start the task thread\n";
        return false;
    }
    taskThread->suspend();

    // start data collection thread
    dataCollectionThread = new DataCollectionThread(taskData->getInt(PAR_COMMON_DATA_COLLECTION_THREAD_PERIOD),taskData,controllerUtil,portUtil);
    if (!dataCollectionThread->start()) {
        cout << dbgTag << "could not start data collection thread\n";
        return false;
    }

    controllerConfigured = true;

    return true;
}

bool HandController::open(std::string context, std::string configFile){
    using yarp::os::ResourceFinder;
    using yarp::os::Bottle;

    // load main configuration file
    ResourceFinder mainConfigFileRF;
    mainConfigFileRF.setDefaultContext(context.c_str());
    mainConfigFileRF.setDefaultConfigFile((configFile + ".ini").c_str());
    char **fakeArgV;
    mainConfigFileRF.configure(0, fakeArgV, false);

    std::string icub = mainConfigFileRF.find(PAR_COMMON_ICUB).asString();
    std::string hand = mainConfigFileRF.find(PAR_COMMON_HAND).asString();

    // load icub-specific configuration file
    ResourceFinder iCubConfigFileRF;
    iCubConfigFileRF.setDefaultContext(context.c_str());
    iCubConfigFileRF.setDefaultConfigFile((configFile + "_" + icub + ".ini").c_str());
    char **fakeArgV;
    iCubConfigFileRF.configure(0, fakeArgV, false);

    // get settings common to both hands
    Bottle &iCubSpecificData = iCubConfigFileRF.findGroup("bothHands");
    // get hand-specific settings
    Bottle &iCubHandSpecificData = iCubConfigFileRF.findGroup(hand + "Hand");

    // put all settings into Property object
    yarp::os::Property options;
    options.fromString(mainConfigFileRF.toString() + iCubSpecificData.toString() + iCubHandSpecificData.toString());

    return open(options);
}



bool HandController::set(const yarp::os::ConstString &key,const yarp::os::Value &value){

    if (!controllerConfigured) return false;

    return taskData->set(key,value);
}

bool HandController::get(const yarp::os::ConstString &key,yarp::os::Value &value){

    if (!controllerConfigured) return false;

    return taskData->get(key, value);
 }

bool HandController::closeHand(bool wait){

    if (!controllerConfigured) return false;

    if (taskRunning == true){

        return false;

    } else {

        taskThread->addApproachTask();
        taskThread->addControlTask();
        bool success = startTask();

        if (wait == true){
            success = waitForGraspStabilization(15,1);
        }

        return success;
    }
}

bool HandController::isHandClose(){

    if (!controllerConfigured) return false;

    return taskData->graspIsStable;
}

bool HandController::openHand(bool fullyOpen, bool wait){

    if (!controllerConfigured) return false;

    if (taskRunning == true){

        taskRunning = false;
        taskThread->suspend();
        return taskThread->afterRun(fullyOpen,wait);

    } else {

        return controllerUtil->openHand(fullyOpen,wait);
    }

}

bool HandController::isHandOpen(){

    if (!controllerConfigured) return false;

    return controllerUtil->isMotionDone();
}

bool HandController::setMinForce(double minForce){

    if (!controllerConfigured) return false;

    taskData->set(PAR_CTRL_MIN_FORCE_ENABLED, yarp::os::Value("true"));
    taskData->set(PAR_CTRL_MIN_FORCE,minForce);

    return true;
}

bool HandController::disableMinForce(){

    if (!controllerConfigured) return false;

    taskData->set(PAR_CTRL_MIN_FORCE_ENABLED, yarp::os::Value("false"));

    return true;
}

bool HandController::setGripStrength(double gripStrength){

    if (!controllerConfigured) return false;

    taskData->set(PAR_CTRL_GRIP_STRENGTH, gripStrength);

    return true;
}

bool HandController::close(){

    if (!controllerConfigured) return false;

    std::cout << dbgTag << "Closing... \n";

    if (taskThread->isRunning()){
        taskThread->suspend();
    }
    if (dataCollectionThread->isRunning()){
        dataCollectionThread->suspend();
    }

    // stop threads
    taskThread->stop();
    dataCollectionThread->stop();

    taskData->release();

    // close controllers and ports
    controllerUtil->release();
    portUtil->release();

    delete(taskData);
    delete(controllerUtil);
    delete(portUtil);

    std::cout << dbgTag << "done \n";

    return true;
}

bool HandController::startTask(){

    if (!controllerConfigured) return false;

    if (taskRunning){

        return false;

    } else {

        if (!taskThread->initializeGrasping()) return false;
        taskRunning = true;
        taskThread->resume();
        return true;
    }
}


bool HandController::waitForGraspStabilization(double timeout, double delay){
    using yarp::os::Time;

    double startTime = Time::now();

    while (!taskData->graspIsStable && (Time::now() - startTime < timeout)) {
        Time::delay(delay);
    }

    return taskData->graspIsStable;
}


bool HandController::addStepTask(const std::vector<double> &targets){

    if (!controllerConfigured) return false;

    return taskThread->addStepTask(targets);
}

bool HandController::addApproachTask(){

    if (!controllerConfigured) return false;

    return taskThread->addApproachTask();
}

bool HandController::addControlTask(){

    if (!controllerConfigured) return false;

    return taskThread->addControlTask();
}
bool HandController::addControlTask(const std::vector<double> &targets){

    if (!controllerConfigured) return false;

    return taskThread->addControlTask(targets);
}

bool HandController::clearTaskList(){

    if (!controllerConfigured) return false;

    return taskThread->clearTaskList();
}

std::string HandController::getDataDescription(){

    if (!controllerConfigured) return "";

    return taskData->getDataDescription();
}

std::string HandController::getTaskListDescription(){

    if (!controllerConfigured) return "";

    return taskThread->getTaskListDescription();
}
