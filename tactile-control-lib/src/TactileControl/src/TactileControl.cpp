#include "TactileControl/TactileControl.h"

#include "TactileControl/data/Parameters.h"

using tactileControl::HandController;

HandController::HandController(){

    dbgTag = "HandController: ";
}


HandController::~HandController(){
    
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

void HandController::set(const yarp::os::Property &options){

}

void HandController::set(const yarp::os::ConstString key, yarp::os::Value value){
    
    taskData->set(key,value);
}

bool HandController::closeHand(bool wait = true){
    
}

bool HandController::isHandClose(){
    
}

bool HandController::openHand(bool wide, bool wait = true){
    
}

bool HandController::isHandOpen(){
    
}

void HandController::setMinForce(double minForce){

}

void HandController::setGripStrength(double gripStrength){

    taskData->set(PAR_CTRL_GRIP_STRENGTH,gripStrength);
}

bool HandController::close(){
    
}

