#include "TactileControl/task/ApproachTask.h"

#include "TactileControl/data/Parameters.h"

#include "yarp/os/Time.h"
#include "yarp/os/Stamp.h"
#include "yarp/os/Bottle.h"
#include "yarp/os/BufferedPort.h"

#include <vector>

using tactileControl::ApproachTask;


ApproachTask::ApproachTask(tactileControl::TaskData *taskData,tactileControl::ControllerUtil * controllerUtil,tactileControl::PortUtil * portUtil):Task(taskData,controllerUtil,portUtil,taskData->getDouble(PAR_APPR_DURATION)) {

    windowSize = taskData->getInt(PAR_APPR_WINDOW_SIZE);
    finalCheckThreshold = taskData->getDouble(PAR_APPR_THRESHOLD);
    double secondsForMovementTimeout = taskData->getDouble(PAR_APPR_TIMEOUT);
    taskData->getList(PAR_APPR_VELOCITY,jointVelocities);
    double errorThreshold = taskData->getDouble(PAR_APPR_THRESHOLD);
    awPolyEst = new iCub::ctrl::AWLinEstimator(windowSize,errorThreshold);

    callsNumberForMovementTimeout = getNumThreadCalls(secondsForMovementTimeout);;

    positionIndex = 0;
    fingerPositions.resize(controlledJoints.size());
    fingerIsInContact.resize(controlledJoints.size(),false);
    fingerSetInPosition.resize(controlledJoints.size(),false);

    taskName = APPROACH;

    dbgTag = "Approach: ";
}

void ApproachTask::init(){

    // set velocity control mode
    controllerUtil->setControlMode(controlledJoints,VOCAB_CM_VELOCITY);
    if (taskData->getBool(PAR_COMMON_USE_RING_LITTLE_FINGERS)){
        controllerUtil->setControlMode(RING_LITTLE_JOINT, VOCAB_CM_VELOCITY);
    }

    // store current joints' pwm limits and set the new ones
    if (taskData->getBool(PAR_APPR_PWM_LIMIT_ENABLED)){
        controllerUtil->saveHandJointsMaxPwmLimits();
        std::vector<double> jointPwmLimits;
        taskData->getList(PAR_APPR_MAX_PWM,jointPwmLimits);
        controllerUtil->setJointsMaxPwmLimit(controlledJoints,jointPwmLimits);
        if (taskData->getBool(PAR_COMMON_USE_RING_LITTLE_FINGERS)){
            controllerUtil->setJointMaxPwmLimit(RING_LITTLE_JOINT, taskData->getDouble(PAR_APPR_RING_LITTLE_MAX_PWM));
        }
    }


    if (loggingEnabled){
        yInfo() << dbgTag << "TASK STARTED";
    }

}

void ApproachTask::calculateControlInput(){
    using yarp::sig::Vector;

    // store proximal joints
    Vector pos(controlledJoints.size());
    for (int i = 0; i < controlledJoints.size(); i++){
        pos[i] = taskData->armEncoderAngles[controlledJoints[i]];
    }
    
    // estimate velocity
    iCub::ctrl::AWPolyElement awPolyEl(pos,yarp::os::Time::now());
    Vector estimatedSpeed = awPolyEst->estimate(awPolyEl);

    // log state
    optionalLogStream << " [state ";
    for (int i = 0; i < controlledJoints.size(); i++){
        optionalLogStream << (fingerIsInContact[i] == true ? 1 : 0) << " ";
    }
    optionalLogStream << "]";
    optionalLogStream << " [vel ";
    for (int i = 0; i < controlledJoints.size(); i++){
        optionalLogStream << estimatedSpeed[i] << "/" << finalCheckThreshold << " ";
    }
    optionalLogStream << "]";
    optionalLogStream << "[t/o " << (callsNumber > callsNumberForMovementTimeout ? 1 : 0) << "]";


    // check if fingers are in contact
    for (int i = 0; i < controlledJoints.size(); i++){
        if (fingerIsInContact[i] == false && callsNumber > callsNumberForMovementTimeout && estimatedSpeed[i] < finalCheckThreshold){
            fingerIsInContact[i] = true;
        }
    }


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

void ApproachTask::sendControlSignal(){

    for(int i = 0; i < inputCommandValue.size(); i++){
        if (!fingerIsInContact[i]) controllerUtil->sendVelocity(controlledJoints[i],inputCommandValue[i]);
    }

    // command the ring/little fingers if enabled
    if (taskData->getBool(PAR_COMMON_USE_RING_LITTLE_FINGERS)){
        controllerUtil->sendVelocity(RING_LITTLE_JOINT, taskData->getDouble(PAR_APPR_RING_LITTLE_VELOCITY));
    }
}


void ApproachTask::release(){

    if (taskData->getBool(PAR_APPR_PWM_LIMIT_ENABLED)){
        controllerUtil->restoreHandJointsMaxPwmLimits();
    }

    if (taskData->getBool(PAR_COMMON_USE_RING_LITTLE_FINGERS)){
        controllerUtil->setControlMode(RING_LITTLE_JOINT, VOCAB_CM_POSITION);
    }

    delete(awPolyEst);
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

