#include "TactileControl/task/ApproachTask.h"

#include "TactileControl/data/Parameters.h"

#include <vector>

using tactileControl::ApproachTask;


ApproachTask::ApproachTask(tactileControl::TaskData *taskData,tactileControl::ControllerUtil * controllerUtil,tactileControl::PortUtil * portUtil):Task(taskData,controllerUtil,portUtil,taskData->getDouble(PAR_APPR_DURATION)) {

    windowSize = taskData->getInt(PAR_APPR_WINDOW_SIZE);
    finalCheckThreshold = taskData->getDouble(PAR_APPR_THRESHOLD);
    double secondsForMovementTimeout = taskData->getDouble(PAR_APPR_TIMEOUT);
    taskData->getList(PAR_APPR_VELOCITY,jointVelocities);

    callsNumberForMovementTimeout = getNumThreadCalls(secondsForMovementTimeout);;

    positionIndex = 0;
    fingerPositions.resize(controlledJoints.size());
    fingerIsInContact.resize(controlledJoints.size(),false);
    fingerSetInPosition.resize(controlledJoints.size(),false);

    taskName = APPROACH;

    dbgTag = "Approach: ";
}

void ApproachTask::init(){
    using std::cout;

    // set velocity control mode
    controllerUtil->setControlMode(controlledJoints,VOCAB_CM_VELOCITY);

    // store current joints pwm limits and set the new ones
    if (taskData->getBool(PAR_APPR_PWM_LIMIT_ENABLED)){
        controllerUtil->saveHandJointsMaxPwmLimits();
        std::vector<double> jointPwmLimits;
        taskData->getList(PAR_APPR_MAX_PWM,jointPwmLimits);
        controllerUtil->setJointsMaxPwmLimit(controlledJoints,jointPwmLimits);
    }

    if (loggingEnabled){
        cout << "\n\n" << dbgTag << "TASK STARTED " << "\n\n";
    }

}

void ApproachTask::calculateControlInput(){
    using yarp::sig::Vector;

    // initialize the current angle in all the windows
    double encoderAngle;
    for(int i = 0; i < controlledJoints.size(); i++){
        encoderAngle = taskData->armEncoderAngles[controlledJoints[i]];
        fingerPositions[i].resize(windowSize,encoderAngle);
    }

    // log state
    optionalLogStream << " [state ";
    for(int i = 0; i < controlledJoints.size(); i++){
        optionalLogStream << (fingerIsInContact[i] == true ? 1 : 0) << " ";
    }
    optionalLogStream << "]";
    optionalLogStream << " [pDiff ";
    for(int i = 0; i < controlledJoints.size(); i++){
        optionalLogStream << taskData->armEncoderAngles[controlledJoints[i]] << "/" <<  fingerPositions[i][(positionIndex + 1)%windowSize] << " ";
    }
    optionalLogStream << "]";
    optionalLogStream << "[t/o " << (callsNumber > callsNumberForMovementTimeout  ? 1 : 0) << "]";
    

    // check if fingers are in contact
    double tempAngleDifference;
    int nextPositionIndex = (positionIndex + 1)%windowSize;
    for(int i = 0; i < controlledJoints.size(); i++){
            
        fingerPositions[i][positionIndex] = taskData->armEncoderAngles[controlledJoints[i]];
        tempAngleDifference = fingerPositions[i][positionIndex] - fingerPositions[i][nextPositionIndex];

        if (fingerIsInContact[i] == false && callsNumber > callsNumberForMovementTimeout && tempAngleDifference < finalCheckThreshold){
            fingerIsInContact[i] = true;
        } 

    }
    positionIndex = nextPositionIndex;

    // move fingers
    for(int i = 0; i < controlledJoints.size(); i++){	
        if (fingerIsInContact[i]){
            if (!fingerSetInPosition[i]){
                // set the joint control mode back to position
                controllerUtil->setControlMode(controlledJoints[i],VOCAB_CM_POSITION);
                fingerSetInPosition[i] = true;
            }
        } else {
            moveFinger(i);
        }
    }

}

void ApproachTask::moveFinger(int finger){

    inputCommandValue[finger] = jointVelocities[finger];
}


void ApproachTask::sendCommands(){

    for(int i = 0; i < inputCommandValue.size(); i++){
        if (!fingerIsInContact[i]) controllerUtil->sendVelocity(controlledJoints[i],inputCommandValue[i]);
    }
}


void ApproachTask::release(){

    if (taskData->getBool(PAR_APPR_PWM_LIMIT_ENABLED)){
        controllerUtil->restoreHandJointsMaxPwmLimits();
    }
}

bool ApproachTask::taskIsOver(){

    return callsNumber >= maxCallsNumber || eachFingerIsInContact();
}

bool ApproachTask::eachFingerIsInContact(){

    bool eachFingerIsInContact = true;

    for(int i = 0; eachFingerIsInContact && i < controlledJoints.size(); i++){
        eachFingerIsInContact = eachFingerIsInContact && fingerIsInContact[i];
    }

    return eachFingerIsInContact;
}

std::string ApproachTask::getTaskDescription(){

    return "Approach task";
}
