#include "TactileControl/TactileControl.h"

#include "TactileControl/data/Parameters.h"

#include <yarp/os/Time.h>

using tactileControl::HandController;

HandController::HandController(){

    taskRunning = false;

    dbgTag = "HandController: ";
}


bool HandController::open(){

    yarp::os::Property options;
    open(options);
}

bool HandController::open(const yarp::os::Property &options){
    using std::cout;

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

    dataCollectionThread = new DataCollectionThread(taskData->getInt(PAR_COMMON_DATA_COLLECTION_THREAD_PERIOD),taskData,controllerUtil,portUtil);
    if (!dataCollectionThread->start()) {
        cout << dbgTag << "could not start data collection thread\n";
        return false;
    }

    return true;
}

void HandController::set(const yarp::os::ConstString key, yarp::os::Value value){

    taskData->set(key,value);
}

bool HandController::closeHand(bool wait = true){

    if (taskRunning == true){

        return false;

    } else {

        taskThread->addApproachTask();
        taskThread->addControlTask();
        bool success = start();

        if (wait == true){
            success = waitForGraspStabilization(15,1);
        }

        return success;
    }
}

bool HandController::isHandClose(){
    
    return taskData->graspIsStable;
}

bool HandController::openHand(bool fullyOpen, bool wait = true){

    if (taskRunning == true){

        taskRunning = false;
        taskThread->suspend();
        taskThread->afterRun(true);

    } else {

        controllerUtil->openHand(fullyOpen,wait);
    }

    return true;
}

bool HandController::isHandOpen(){
    
    return controllerUtil->isMotionDone();
}

void HandController::setMinForce(double minForce){

    taskData->set(PAR_CTRL_MIN_FORCE_ENABLED,yarp::os::Value("true"));
    taskData->set(PAR_CTRL_MIN_FORCE,minForce);
}

void HandController::disableMinForce(){

    taskData->set(PAR_CTRL_MIN_FORCE_ENABLED,yarp::os::Value("false"));
}

void HandController::setGripStrength(double gripStrength){

    taskData->set(PAR_CTRL_GRIP_STRENGTH,gripStrength);
}

bool HandController::close(){
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

bool HandController::start(){

    if (taskRunning){

        return false;
    
    } else {

        if (!taskThread->initializeGrasping()) return false;
        taskRunning = true;
        taskThread->resume();
        return true;
    }
}


bool HandController::waitForGraspStabilization(double timeout, double delay{
    using yarp::os::Time;

    double startTime = Time::now();

    while (!taskData->graspIsStable && (Time::now() - startTime < timeout)) {
        Time::delay(delay);
    }

    return taskData->graspIsStable;
}