#include "tactileControlWrapper/HandControllerWrapper.h"

#include <yarp/os/Time.h>
#include <yarp/os/Property.h>

#include <vector>

using tactileControlWrapper::HandControllerWrapper;
using yarp::os::ResourceFinder;
using yarp::os::Value;
using std::cout;
using std::endl;

HandControllerWrapper::HandControllerWrapper() 
    : RFModule() {
        closing = false;
        tasksRunning = false;

        dbgTag = "HandControllerWrapper: ";
}

HandControllerWrapper::~HandControllerWrapper() {}

double HandControllerWrapper::getPeriod() { return moduleThreadPeriod/1000.0; }

bool HandControllerWrapper::configure(ResourceFinder &rf) {
    using std::string;
    using yarp::os::Property;

    cout << dbgTag << "Starting. \n";

    // get resoource finder data
    string portPrefix = rf.check("portPrefix", Value("tactileControlWrapper")).asString();
    string libConfigFileName = rf.check("libConfigFileName", Value("confTactileControlLib")).asString();
    string libConfigFileContext = rf.check("libConfigFileContext", Value("tactileControlWrapper")).asString();
    string hand = rf.check("hand", Value("left")).asString();
    moduleThreadPeriod = rf.check("moduleThreadPeriod", 1000).asInt();

    // open ports
    portPlantIdentificationRPC.open("/" + portPrefix + "/cmd:i");
    attach(portPlantIdentificationRPC);

    // initialize rpc commands utility
    rpcCmdUtil.init(&rpcCmdData);

    // initialize the hand controller
    handController.set(libConfigFileContext, libConfigFileName);
    handController.set("hand", Value(hand));
    if (!handController.open()){
        cout << dbgTag << "could not initialize the hand controller. \n";
        return false;
    }

    // initialize the object recognition manager
    if (!objRecManager.open(handController)){
        cout << dbgTag << "could not initialize the object recognition manager. \n";
        return false;
    }

    cout << dbgTag << "Started correctly. \n";

    return true;
}

bool HandControllerWrapper::updateModule() { 

    return !closing; 
}

bool HandControllerWrapper::interruptModule() {
    cout << dbgTag << "Interrupting... ";
    
    // Interrupt port
    portPlantIdentificationRPC.interrupt();

    cout << dbgTag << "interrupted correctly." << endl;

    return true;
}

bool HandControllerWrapper::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply){

    screenMsg.str(std::string());
    errMsg.str(std::string());

    bool success = rpcCmdUtil.processCommand(command);

    if (!success){

        errMsg << rpcCmdUtil.errMsg.str();

    }
    else {

        switch (rpcCmdUtil.mainCmd){

        case HELP:
            success = help();
            break;
        case SET:
            success = set(rpcCmdUtil.paramName, rpcCmdUtil.argValue);
            break;
        case GET:
            success = get(rpcCmdUtil.paramName);
            break;
        case TASK:
            success = task(rpcCmdUtil.taskCmdArg, rpcCmdUtil.task, rpcCmdUtil.argValue);
            break;
        case SHOW:
            success = show(rpcCmdUtil.viewCmdArg);
            break;
        case START:
            success = start();
            break;
        case OPEN:
            success = open(rpcCmdUtil.argValue, rpcCmdUtil.waitValue);
            break;
        case GRASP:
            success = grasp(rpcCmdUtil.waitValue);
            break;
        case IS_HAND_OPEN:
            success = isHandOpen();
            break;
        case IS_HAND_CLOSE:
            success = isHandClose();
            break;
        case SET_GRIP_STRENGTH:
            success = setGripStrength(rpcCmdUtil.argValue);
            break;
        case SET_MIN_FORCE:
            success = setMinForce(rpcCmdUtil.argValue);
            break;
        case DISABLE_MIN_FORCE:
            success = disableMinForce();
            break;
        case OBJECT_RECOGNITION:
            success = objectRecognition(rpcCmdUtil.objRecCmdArg, rpcCmdUtil.argValue);
            break;
        case QUIT:
            success = quit();
            break;
        }
    }

    if (success){
        reply.addString("ack");
        cout << screenMsg.str() << endl;
    }
    else {
        reply.addString("nack");
        cout << errMsg.str() << endl;
    }

    return true;
}

bool HandControllerWrapper::close() {

    // close hand controller
    handController.close();

    // close object recognition manager
    objRecManager.close();

    // close rpc port
    portPlantIdentificationRPC.close();

    return true;
}

bool HandControllerWrapper::open(const Value &paramValue, const Value &waitValue) {

    if (waitValue.isNull()){
        return handController.openHand(paramValue.asBool());
    }
    else {
        return handController.openHand(paramValue.asBool(), waitValue.asBool());
    }
}

bool HandControllerWrapper::start() {

    return handController.startTask();
}

bool HandControllerWrapper::grasp(const yarp::os::Value &waitValue) {

    if (waitValue.isNull()){
        return handController.closeHand();
    }
    else {
        return handController.closeHand(waitValue.asBool());
    }
}

bool HandControllerWrapper::quit() {

    closing = true;
    return true;
}

bool HandControllerWrapper::set(const yarp::os::ConstString &paramName, const Value &paramValue){

    if (handController.set(paramName, paramValue)){

        screenMsg << "'" << paramName << "' set to " << paramValue.toString();

        return true;
    }
    else {
        errMsg << "param '" << paramName << "' is not valid";

        return false;
    }

}

bool HandControllerWrapper::get(const yarp::os::ConstString &paramName){

    yarp::os::Value returnValue;
    if (!handController.get(paramName, returnValue)){
        screenMsg << "'" << paramName << "' not set";
    }
    else {
        screenMsg << "'" << paramName << "': " << returnValue.toString();
    }

    return true;
}

bool HandControllerWrapper::task(tactileControlWrapper::RPCTaskCmdArgName paramName, tactileControlWrapper::TaskName taskName, const Value &paramValue){

    std::vector<double> targets;
    bool success = false;

    switch (paramName){

    case ADD:

        if (!rpcCmdData.setTargets(paramValue, targets)){
            return false;
        }

        switch (taskName){

        case STEP:
            success = handController.addStepTask(targets);
            break;

        case APPROACH:
            success = handController.addApproachTask();
            break;

        case CONTROL:

            if (targets.empty()){
                success = handController.addControlTask();
            }
            else {
                success = handController.addControlTask(targets);
            }
            break;

        }

        break;

    case CLEAR:
        success = handController.clearTaskList();
        break;

    }

    return success;
}

bool HandControllerWrapper::show(tactileControlWrapper::RPCViewCmdArgName paramName){

    bool success = false;

    switch (paramName){

    case TASKS:
        screenMsg << handController.getTaskListDescription();
        success = true;
        break;

    case SETTINGS:
        screenMsg << handController.getDataDescription();
        success = true;
        break;
    }

    return success;
}

bool HandControllerWrapper::help(){

    screenMsg << rpcCmdData.showHelp();

    return true;
}

bool HandControllerWrapper::isHandOpen(){

    screenMsg << handController.isHandOpen();

    return true;
}
bool HandControllerWrapper::isHandClose(){

    screenMsg << handController.isHandClose();

    return true;
}

bool HandControllerWrapper::setGripStrength(const yarp::os::Value &paramValue){

    handController.setGripStrength(paramValue.asDouble());
    
    screenMsg << "Grip strength set to " << paramValue.toString();

    return true;
}

bool HandControllerWrapper::setMinForce(const yarp::os::Value &paramValue){

    handController.setMinForce(paramValue.asDouble());

    screenMsg << "Min force mode enabled and min force set to " << paramValue.toString();

    return true;

}

bool HandControllerWrapper::disableMinForce(){

    handController.disableMinForce();

    screenMsg << "Min force mode disabled";

    return true;
}

bool HandControllerWrapper::objectRecognition(tactileControlWrapper::RPCObjRecCmdArgName paramName, const yarp::os::Value &paramValue){

    bool success = false;

    switch (paramName){

    case LOAD_TRAINING_SET:
        success = objRecManager.loadTrainingSet(paramValue.asString());
        break;

    case SAVE_TRAINING_SET:
        success = objRecManager.saveTrainingSet(paramValue.asString());
        break;

    case LOAD_OBJECTS:
        success = objRecManager.loadObjects(paramValue.asString());
        break;

    case SAVE_OBJECTS:
        success = objRecManager.saveObjects(paramValue.asString());
        break;

    case LOAD_MODEL:
        success = objRecManager.loadModel(paramValue.asString());
        break;

    case SAVE_MODEL:
        success = objRecManager.saveModel(paramValue.asString());
        break;

    case VIEW_DATA:
        success = objRecManager.viewData();
        break;

    case DISCARD_LAST_FEATURES:
        success = objRecManager.discardLastFeatures();
        break;

    case CLEAR_COLLECTED_FEATURES:
        success = objRecManager.clearCollectedFeatures();
        break;

    case PROCESS_COLLECTED_DATA:
        success = objRecManager.processCollectedData();
        break;

    case ADD_NEW_OBJECT:
        success = objRecManager.addNewObject(paramValue.asString());
        break;

    case GET_READY:
        success = objRecManager.getReady(paramValue.asString());
        break;

    case READ_VISUAL_CLASSIFIER_SCORES:
        success = objRecManager.readVisualClassifierOutputScores();
        break;

    case RESET:
        success = objRecManager.reset();
        break;

    }

    return success;
}

