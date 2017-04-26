#include "TactileControl/TactileControl.h"

using tactileControl::HandController;

HandController::HandController(){

    dbg
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
    
}

bool HandController::close(){
    
}

