#include "TactileControl/task/ControlTask.h"

#include "TactileControl/data/Parameters.h"
#include "TactileControl/util/ICubUtil.h"
#include "TactileControl/util/CommonUtil.h"

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>
#include <yarp/os/LogStream.h>

#include <vector>
#include <cmath>
#include <algorithm>

using tactileControl::ControlTask;
using yarp::os::Bottle;
using yarp::os::Value;
using yarp::sig::Vector;
using yarp::sig::Matrix;


ControlTask::ControlTask(tactileControl::TaskData *taskData, tactileControl::ControllerUtil * controllerUtil, tactileControl::PortUtil * portUtil, tactileControl::MLUtil * mlUtil) :Task(taskData, controllerUtil, portUtil, taskData->getDouble(PAR_CTRL_DURATION)) {

    std::vector<double> targets(1,taskData->getDouble(PAR_CTRL_DEFAULT_FORCE_TARGET));

    constructorsCommon(targets,mlUtil);
}

ControlTask::ControlTask(tactileControl::TaskData *taskData, tactileControl::ControllerUtil * controllerUtil, tactileControl::PortUtil * portUtil, tactileControl::MLUtil * mlUtil, const std::vector<double> &targets) :Task(taskData, controllerUtil, portUtil, taskData->getDouble(PAR_CTRL_DURATION)) {

    constructorsCommon(targets,mlUtil);
}

void ControlTask::constructorsCommon(const std::vector<double> &targets,tactileControl::MLUtil *mlUtil){

    this->mlUtil = mlUtil;

    expandTargets(targets,forceTargetValue);

    taskName = CONTROL;

    dbgTag = "ControlTask: ";    
}


void ControlTask::init(){
    using iCub::ctrl::minJerkTrajGen;

    // initialize variables
    supervisorEnabled = taskData->getBool(PAR_CTRL_SUPERVISOR_ENABLED);
    supervisorControlMode = taskData->getSupervisorControlMode();
    gmmJointsRegressionEnabled = taskData->getBool(PAR_CTRL_GMM_JOINTS_REGRESSION_ENABLED);
    numFingers = taskData->getFingerNum();
    gmmCtrlModeIsSet = false;

    initLowLevelPID();

    if (supervisorEnabled){
        
        initHighLevelPID();

        minJerkTrackingModeInitialized = false;
        gmmJointsMinJerkTrackingModeInitialized = false;

        // initialize minimum jerk trajectories
        double minJerkTrajRefTime = taskData->getDouble(PAR_CTRL_MIN_JERK_TRACK_REF_TIME);
        double gmmMinJerkTrajRefTime = taskData->getDouble(PAR_CTRL_GMM_JOINTS_MIN_JERK_TRACK_REF_TIME);
        minJerkTrajectory = new minJerkTrajGen(1,taskThreadPeriod/1000.0,minJerkTrajRefTime);
        thAbdMinJerkTrajectory = new minJerkTrajGen(1,taskThreadPeriod/1000.0,gmmMinJerkTrajRefTime);
        thDistMinJerkTrajectory = new minJerkTrajGen(1,taskThreadPeriod/1000.0,gmmMinJerkTrajRefTime);
        indDistMinJerkTrajectory = new minJerkTrajGen(1,taskThreadPeriod/1000.0,gmmMinJerkTrajRefTime);
        midDistMinJerkTrajectory = new minJerkTrajGen(1,taskThreadPeriod/1000.0,gmmMinJerkTrajRefTime);

        disablePIDIntegralGain = taskData->getBool(PAR_CTRL_LOW_PID_INTEGRAL_DISABLED);

        if (disablePIDIntegralGain){
            controllerUtil->resetPIDIntegralGain(THUMB_ABDUCTION_JOINT);
        }

        if (supervisorControlMode == GMM_MODE && gmmJointsRegressionEnabled){
            setGMMJointsControlMode(VOCAB_CM_POSITION_DIRECT);
        }

        // in case the HAND_FREEZE supervisor control mode is activated, the initial object position is stored 
        initialObjectPosition = ICubUtil::getObjectPosition(numFingers,taskData->armEncoderAngles);

    }

    // set the controlled joints in open loop control mode
    controllerUtil->setControlMode(controlledJoints,VOCAB_CM_PWM);
    if (taskData->getBool(PAR_COMMON_USE_RING_LITTLE_FINGERS)){
        controllerUtil->setControlMode(RING_LITTLE_JOINT, VOCAB_CM_PWM);
    }

    // initialize machine learning data
    objectRecognitionTaskEnabled = taskData->getBool(PAR_ML_OBJECT_RECOGNITION_TASK_ENABLED);
    if (objectRecognitionTaskEnabled){
        startSqueezingTime = taskData->getDouble(PAR_ML_GRASP_STABILIZATION_TIME);
        startTactileDataCollectionTime = startSqueezingTime + taskData->getDouble(PAR_ML_OBJECT_SQUEEZING_TIME);
        startBendingProximalJointsTime = startTactileDataCollectionTime + taskData->getDouble(PAR_ML_TACTILE_DATA_COLLECTION_TIME);
        startBendingDistalJointsTime = startBendingProximalJointsTime + taskData->getDouble(PAR_ML_BENDING_PROXIMAL_JOINTS_TIME);
        taskCompleteTime = startBendingDistalJointsTime + taskData->getDouble(PAR_ML_BENDING_DISTAL_JOINTS_TIME);

        gripStrengthIncreased = false;
        featuresProcessingComplete = false;
        proximalJointsSetInOLMode = false;
        distalJointsSetInOLMode = false;
        averageTactileData.resize(2 * NUM_TAXELS, 0); // using 2 fingers
        averageTactileDataNormalized.resize(averageTactileData.size(), 0);
        objRecFeatures.resize(NUM_OBJ_REC_FEATURES);
    }

    std::stringstream logStream("");

    logStream << "\n\n" << dbgTag << "TASK STARTED - Target: ";
    for(int i = 0; i < forceTargetValue.size(); i++){
        logStream << forceTargetValue[i] << " ";
    }
    yInfo() << logStream.str();
}

void ControlTask::initLowLevelPID(){

    std::vector<double> pidKp,pidKi;
    double pidWp,pidWi,pidWd,pidN,pidWindUp,pidMinSatLim,pidMaxSatLim;

    // get all parameters
    taskData->getList(PAR_CTRL_LOW_PID_KP,pidKp);
    taskData->getList(PAR_CTRL_LOW_PID_KI,pidKi);
    pidWp = taskData->getDouble(PAR_CTRL_LOW_PID_WP);
    pidWi = taskData->getDouble(PAR_CTRL_LOW_PID_WI);
    pidWd = taskData->getDouble(PAR_CTRL_LOW_PID_WD);
    pidN = taskData->getDouble(PAR_CTRL_LOW_PID_N);
    pidWindUp = taskData->getDouble(PAR_CTRL_LOW_PID_WIND_UP_COEFF);
    pidMinSatLim = taskData->getDouble(PAR_CTRL_LOW_PID_MIN_SAT_LIM);
    pidMaxSatLim = taskData->getDouble(PAR_CTRL_LOW_PID_MAX_SAT_LIM);
 
    // calculate TT pid option
    std::vector<double> pidTt;
    pidTt.resize(controlledJoints.size());
    for(int i = 0; i < controlledJoints.size(); i++){
        pidTt[i] = calculateTt(pidKp[i],pidKi[i],0.0,pidWindUp);
    }
     
    // init PID
    pid.resize(controlledJoints.size());
    for(int i = 0; i < controlledJoints.size(); i++){
        initPID(pid[i],pidKp[i],pidKi[i],0.0,pidWp,pidWi,pidWd,pidN,pidTt[i],pidMinSatLim,pidMaxSatLim);
    }
}

void ControlTask::initHighLevelPID(){

    double highPidWp,highPidWi,highPidWd,highPidN,highPidWindUp,highPidMinSatLim,highPidMaxSatLim,highPidTt;

    // get all parameters
    highPidKp = taskData->getDouble(PAR_CTRL_HIGH_PID_KP);
    highPidKi = taskData->getDouble(PAR_CTRL_HIGH_PID_KI);
    highPidKd = taskData->getDouble(PAR_CTRL_HIGH_PID_KD);
    highPidWp = taskData->getDouble(PAR_CTRL_HIGH_PID_WP);
    highPidWi = taskData->getDouble(PAR_CTRL_HIGH_PID_WI);
    highPidWd = taskData->getDouble(PAR_CTRL_HIGH_PID_WD);
    highPidN = taskData->getDouble(PAR_CTRL_HIGH_PID_N);
    highPidWindUp = taskData->getDouble(PAR_CTRL_LOW_PID_WIND_UP_COEFF);
    highPidMinSatLim = taskData->getDouble(PAR_CTRL_HIGH_PID_MIN_SAT_LIM);
    highPidMaxSatLim = taskData->getDouble(PAR_CTRL_HIGH_PID_MAX_SAT_LIM);
    highPidTt = calculateTt(highPidKp,highPidKi,highPidKd,highPidWindUp);

    // init PID
    initPID(highPid,highPidKp,highPidKi,highPidKd,highPidWp,highPidWi,highPidWd,highPidN,highPidTt,highPidMinSatLim,highPidMaxSatLim);
}

void ControlTask::initPID(iCub::ctrl::parallelPID *&pid,double kp,double ki,double kd,double wp,double wi,double wd,double n,double tt,double minSatLim,double maxSatLim){
 
    // configure yarp Vector and Matrix structures to initialize PID
    Vector kpOptionVect(1,kp);
    Vector kiOptionVect(1,ki);
    Vector kdOptionVect(1,kd);
    Vector wpOptionVect(1,wp);
    Vector wiOptionVect(1,wi);
    Vector wdOptionVect(1,wd);
    Vector nOptionVect(1,n);
    Vector ttOptionVect(1,tt);
  
    Matrix pvSatLimMatrix(1,2);
    pvSatLimMatrix[0][0] = minSatLim;
    pvSatLimMatrix[0][1] = maxSatLim;

    // configure yarp Bottle to initialize PID
    Bottle pidOptions;
    addOption(pidOptions,"Wp",wp);
    addOption(pidOptions,"Wi",wi);
    addOption(pidOptions,"Wd",wd);
    addOption(pidOptions,"N",n);
    addOption(pidOptions,"satLim",minSatLim,maxSatLim);
    addOption(pidOptions,"Kp",kp);
    addOption(pidOptions,"Ki",ki);
    addOption(pidOptions,"Kd",kd);
    addOption(pidOptions,"Tt",tt);
 
    // initialize PID
    pid = new iCub::ctrl::parallelPID(taskThreadPeriod/1000.0,kpOptionVect,kiOptionVect,kdOptionVect,wpOptionVect,wiOptionVect,wdOptionVect,nOptionVect,ttOptionVect,pvSatLimMatrix);
    pid->setOptions(pidOptions);
 
}

void ControlTask::calculateControlInput(){
    using yarp::sig::Vector;

    // refresh data
    gmmJointsRegressionEnabled = taskData->getBool(PAR_CTRL_GMM_JOINTS_REGRESSION_ENABLED);
    supervisorControlMode = taskData->getSupervisorControlMode();

    if (supervisorEnabled){

        // if the joints regression is enabled/disabled while the task is being executed, control modes need to be changed accordingly.
        if (supervisorControlMode == GMM_MODE && gmmJointsRegressionEnabled){
            if (!gmmCtrlModeIsSet) setGMMJointsControlMode(VOCAB_CM_POSITION_DIRECT);
        } else {
            if (gmmCtrlModeIsSet) setGMMJointsControlMode(VOCAB_CM_POSITION);
        }

        // evaluate the supervisor control mode
        double targetObjectPosition,filteredTargetObjectPosition,currentTargetObjectPosition,handAperture,indMidPosDiff;
        double gmmThumbAbductionJoint,gmmThumbDistalJoint,gmmIndexDistalJoint,gmmMiddleDistalJoint,filteredThumbAbductionJoint,filteredThumbDistalJoint,filteredIndexDistalJoint,filteredMiddleDistalJoint;
        switch(supervisorControlMode){

        case FIXED_OBJECT_POS_MODE:

            // the target object position is the one set as parameter
            targetObjectPosition = taskData->getDouble(PAR_CTRL_TARGET_OBJECT_POSITION);

            break;

        case HAND_FREEZE_MODE:

            // the target object position is the one detected after the approach phase
            targetObjectPosition = initialObjectPosition;

            break;

        case GMM_MODE:
            
            // the target object position is the one given by the Gaussian mixture regression
            // run gmm regression and control the abduction/distal joints
            handAperture = ICubUtil::getHandAperture(numFingers,taskData->armEncoderAngles);
            indMidPosDiff = ICubUtil::getIndexMiddleDifference(numFingers,taskData->armEncoderAngles);
            manageGMMRegression(handAperture,indMidPosDiff,targetObjectPosition,gmmThumbAbductionJoint,gmmThumbDistalJoint,gmmIndexDistalJoint,gmmMiddleDistalJoint,filteredThumbAbductionJoint,filteredThumbDistalJoint,filteredIndexDistalJoint,filteredMiddleDistalJoint);

            break;

        default: // do nothing
            break;
        }

        // calculate the actual object position
        double objectPosition = ICubUtil::getObjectPosition(numFingers,taskData->armEncoderAngles);

        // object position min jerk tracking
        if (taskData->getBool(PAR_CTRL_MIN_JERK_TRACK_ENABLED)){

            if (minJerkTrackingModeInitialized == false){
                CommonUtil::initMinJerkTrajectory(minJerkTrajectory,taskData->getDouble(PAR_CTRL_MIN_JERK_TRACK_REF_TIME),objectPosition);
                minJerkTrackingModeInitialized = true;
            }

            CommonUtil::getMinJerkFilteredPosition(minJerkTrajectory,targetObjectPosition,filteredTargetObjectPosition);

            currentTargetObjectPosition = filteredTargetObjectPosition;

        } else {

            if (minJerkTrackingModeInitialized == true){
                minJerkTrackingModeInitialized = false;
            }

            // no filter is applied
            currentTargetObjectPosition = targetObjectPosition;
        }

        // compute supervisor control signal
        double svErr = targetObjectPosition - objectPosition;
        Vector svRef(1,currentTargetObjectPosition);
        Vector svFb(1,objectPosition);
        Vector svResult = highPid->compute(svRef,svFb);
        double highPidScaleFactor = taskData->getDouble(PAR_CTRL_HIGH_PID_SCALE);
        double svControlSignal = svResult[0] * highPidScaleFactor;

        // compute force target values
        double gripStrength = taskData->getDouble(PAR_CTRL_GRIP_STRENGTH);
        bool minForceEnabled = taskData->getBool(PAR_CTRL_MIN_FORCE_ENABLED);
        double minForce = taskData->getDouble(PAR_CTRL_MIN_FORCE);
        computeForceTargetValues(gripStrength,svControlSignal,minForceEnabled,minForce,forceTargetValue);

        // log supervisor info on screen
        if (numFingers == 2){
            optionalLogStream << " [P " << forceTargetValue[0] << " - " << forceTargetValue[1] << "][err " << svErr << "][u " << svControlSignal << "]" ;
        } else {
            optionalLogStream << " [P " << forceTargetValue[0] << " - " << forceTargetValue[1] << " - " << forceTargetValue[2] << "][err " << svErr << "][u " << svControlSignal << "]" ;
        }

        // compute the actual grip strength (used for logging)
        double actualGripStrength = ICubUtil::getGripStrength(numFingers,taskData->overallFingerForce);

        // log control data
        portUtil->sendControlData(taskId,taskData->getString(PAR_COMMON_EXPERIMENT_INFO),taskData->getString(PAR_COMMON_EXPERIMENT_OPTIONAL_INFO),gripStrength,actualGripStrength,svControlSignal,svErr,objectPosition,currentTargetObjectPosition,targetObjectPosition,forceTargetValue,inputCommandValue,controlledFingers,taskData);

        // log gaussian mixture model regression data
        if (supervisorControlMode == GMM_MODE){
            portUtil->sendGMMRegressionData(handAperture,indMidPosDiff,targetObjectPosition,objectPosition,currentTargetObjectPosition,gmmThumbDistalJoint,filteredThumbDistalJoint,gmmIndexDistalJoint,filteredIndexDistalJoint,gmmMiddleDistalJoint,filteredMiddleDistalJoint,gmmThumbAbductionJoint,filteredThumbAbductionJoint,gripStrength,actualGripStrength,taskData);
        }

        // take a snapshot of the current pose (GMM training set data collection)
        if (taskData->getBool(PAR_CTRL_GMM_BEST_POSE_LOG_ONE_SHOT)){
            portUtil->sendGMMData(gripStrength,taskData);
            taskData->set(PAR_CTRL_GMM_BEST_POSE_LOG_ONE_SHOT,Value("false"));
        }

        // check if grasp is stable
        if (timeElapsed() > TIME_TO_STABILIZE_GRASP){
            taskData->graspIsStable = true;
        }
    }

    // compute pwm values from force target values
    for(int i = 0; i < controlledJoints.size(); i++){

        Vector ref(1,forceTargetValue[i]);
        Vector fb(1,taskData->overallFingerForce[controlledFingers[i]]);
        Vector result = pid[i]->compute(ref,fb);

        inputCommandValue[i] = result[0];

        if (taskData->getBool(PAR_COMMON_DISABLE_PWM)){
            inputCommandValue[i] = 0.0;
        }
    }

    // command the ring/little fingers if enabled
    if (taskData->getBool(PAR_COMMON_USE_RING_LITTLE_FINGERS)){
        controllerUtil->sendPwm(RING_LITTLE_JOINT, pwmSign*taskData->getDouble(PAR_CTRL_RING_LITTLE_PWM));
    }

    if (objectRecognitionTaskEnabled){
        manageObjectRecognitionTask();
    }

}



void ControlTask::addOption(Bottle &bottle,const char *paramName,const Value paramValue){

    Bottle valueBottle,paramBottle;

    valueBottle.add(paramValue);

    paramBottle.addString(paramName);
    paramBottle.addList() = valueBottle;

    bottle.addList() = paramBottle;
}

void ControlTask::addOption(Bottle &bottle,const char *paramName,const Value paramValue1,const Value paramValue2){

    Bottle valueBottle,paramBottle;

    valueBottle.add(paramValue1);
    valueBottle.add(paramValue2);

    paramBottle.addString(paramName);
    paramBottle.addList() = valueBottle;

    bottle.addList() = paramBottle;
}

void ControlTask::setOptionVect(const std::vector<double> &option,std::vector<yarp::sig::Vector> &optionVect){

    optionVect.resize(controlledJoints.size());
    for(int i = 0; i < optionVect.size(); i++){
        optionVect[i].resize(1,option[i]);
    }
}

double ControlTask::calculateTt(double kp,double ki,double kd,double windUp){

    double tt,ti,td,minTt,maxTt;

    ti = kp/ki;
    td = kd/kp;
    minTt = windUp*ti;
    maxTt = ti;
    if (td < minTt){
        tt = minTt;
    } else if (td > maxTt){
        tt = maxTt;
    } else tt = td;

    return tt;
}

std::string ControlTask::getTaskDescription(){

    std::stringstream description("");

    description << "Control task: ";

    for(int i = 0; i < forceTargetValue.size(); i++){
        description << forceTargetValue[i] << " ";
    }

    return description.str();
}

void ControlTask::setGMMJointsControlMode(int controlMode){

        controllerUtil->setControlMode(THUMB_ABDUCTION_JOINT,controlMode);
        controllerUtil->setControlMode(THUMB_DISTAL_JOINT,controlMode);
        if (numFingers > 2) controllerUtil->setControlMode(INDEX_DISTAL_JOINT,controlMode);
        controllerUtil->setControlMode(MIDDLE_DISTAL_JOINT,controlMode);

        if (controlMode == VOCAB_CM_POSITION_DIRECT){
            gmmCtrlModeIsSet = true;
        } else {
            gmmCtrlModeIsSet = false;
        }

}

void ControlTask::manageGMMRegression(double handAperture,double indMidPosDiff,double &targetObjectPosition,double &gmmThumbAbductionJoint,double &gmmThumbDistalJoint,double &gmmIndexDistalJoint,double &gmmMiddleDistalJoint,double &filteredThumbAbductionJoint,double &filteredThumbDistalJoint,double &filteredIndexDistalJoint,double &filteredMiddleDistalJoint){
    using yarp::sig::Vector;

    /* Preparing gmm structures, features available for regression are:
    *   0: hand aperture
    *   1: index/middle fingers angle difference
    *   2: object position
    *   3: thumb distal joint
    *   4: index finger distal joint
    *   5: middle finger distal joint
    *   6: thumb abduction joint
    *   7: index/middle finger force balance
    */
    std::vector<int> qIndexes(2);
    // the features with indexes 0 and 1 will be used as input for the regression
    qIndexes[0] = 0; qIndexes[1] = 1;

    std::vector<int> rIndexes(6);
    // the features with indexes 2, 3, 4, 5, 6 and 7 will be used as output for the regression
    rIndexes[0] = 2; rIndexes[1] = 3; rIndexes[2] = 4; rIndexes[3] = 5; rIndexes[4] = 6; rIndexes[5] = 7;

    taskData->gmmDataStandard->buildQRStructures(qIndexes,rIndexes);

    yarp::sig::Vector queryPoint,output;
    // create the vector with the features used as input for the regression
    queryPoint.resize(2);
    queryPoint[0] = handAperture;
    queryPoint[1] = indMidPosDiff;

    // run the gaussian misture regression and get the output features
    taskData->gmmDataStandard->runGaussianMixtureRegression(queryPoint,output);

    targetObjectPosition = output[0];

    if (gmmJointsRegressionEnabled){

        double targetThumbAbductionJoint,targetThumbDistalJoint,targetIndexDistalJoint,targetMiddleDistalJoint;

        gmmThumbDistalJoint = output[1];
        gmmIndexDistalJoint = output[2];
        gmmMiddleDistalJoint = output[3];
        gmmThumbAbductionJoint = output[4] + taskData->getDouble(PAR_CTRL_THUMB_ABDUCTION_OFFSET);;

        /* Min jerk tracking */
        if (taskData->getBool(PAR_CTRL_GMM_JOINTS_MIN_JERK_TRACK_ENABLED)){
            if (gmmJointsMinJerkTrackingModeInitialized == false){

                // init min jerk trajectory
                double gmmMinJerkTrajRefTime = taskData->getDouble(PAR_CTRL_GMM_JOINTS_MIN_JERK_TRACK_REF_TIME);

                CommonUtil::initMinJerkTrajectory(thAbdMinJerkTrajectory,gmmMinJerkTrajRefTime,taskData->armEncoderAngleReferences[THUMB_ABDUCTION_JOINT]);
                CommonUtil::initMinJerkTrajectory(thDistMinJerkTrajectory,gmmMinJerkTrajRefTime,taskData->armEncoderAngleReferences[THUMB_DISTAL_JOINT]);
                CommonUtil::initMinJerkTrajectory(indDistMinJerkTrajectory,gmmMinJerkTrajRefTime,taskData->armEncoderAngleReferences[INDEX_DISTAL_JOINT]);
                CommonUtil::initMinJerkTrajectory(midDistMinJerkTrajectory,gmmMinJerkTrajRefTime,taskData->armEncoderAngleReferences[MIDDLE_DISTAL_JOINT]);

                gmmJointsMinJerkTrackingModeInitialized = true;
            }

            // compute next min jerk trajectory position
            CommonUtil::getMinJerkFilteredPosition(thAbdMinJerkTrajectory,gmmThumbAbductionJoint,filteredThumbAbductionJoint);
            CommonUtil::getMinJerkFilteredPosition(thDistMinJerkTrajectory,gmmThumbDistalJoint,filteredThumbDistalJoint);
            CommonUtil::getMinJerkFilteredPosition(indDistMinJerkTrajectory,gmmIndexDistalJoint,filteredIndexDistalJoint);
            CommonUtil::getMinJerkFilteredPosition(midDistMinJerkTrajectory,gmmMiddleDistalJoint,filteredMiddleDistalJoint);

            targetThumbAbductionJoint = filteredThumbAbductionJoint;
            targetThumbDistalJoint = filteredThumbDistalJoint;
            targetIndexDistalJoint = filteredIndexDistalJoint;
            targetMiddleDistalJoint = filteredMiddleDistalJoint;

        } else {
            if (gmmJointsMinJerkTrackingModeInitialized == true){
                gmmJointsMinJerkTrackingModeInitialized = false;
            }

            targetThumbAbductionJoint = gmmThumbAbductionJoint;
            targetThumbDistalJoint = gmmThumbDistalJoint;
            targetIndexDistalJoint = gmmIndexDistalJoint;
            targetMiddleDistalJoint = gmmMiddleDistalJoint;
        }

        // move joints
        controllerUtil->setJointAnglePositionDirect(THUMB_ABDUCTION_JOINT,targetThumbAbductionJoint);
        controllerUtil->setJointAnglePositionDirect(THUMB_DISTAL_JOINT,targetThumbDistalJoint);
        if (numFingers > 2) controllerUtil->setJointAnglePositionDirect(INDEX_DISTAL_JOINT,targetIndexDistalJoint);
        controllerUtil->setJointAnglePositionDirect(MIDDLE_DISTAL_JOINT,targetMiddleDistalJoint);
    }

}


void ControlTask::release(){

    for(int i = 0; i < controlledJoints.size(); i++){
        delete(pid[i]);
    }

    taskData->set(PAR_COMMON_EXPERIMENT_INFO,Value(""));

    if (supervisorEnabled){

        taskData->graspIsStable = false;

        delete(highPid);

        delete(minJerkTrajectory);
        delete(thAbdMinJerkTrajectory);
        delete(thDistMinJerkTrajectory);
        delete(indDistMinJerkTrajectory);
        delete(midDistMinJerkTrajectory);

        if (disablePIDIntegralGain) controllerUtil->restorePIDIntegralGain(THUMB_ABDUCTION_JOINT);

        if (supervisorControlMode == GMM_MODE && gmmJointsRegressionEnabled){
            setGMMJointsControlMode(VOCAB_CM_POSITION);
        }

        if (taskData->getBool(PAR_COMMON_USE_RING_LITTLE_FINGERS)){
            controllerUtil->setControlMode(RING_LITTLE_JOINT, VOCAB_CM_POSITION);
        }
    }
}

void ControlTask::computeForceTargetValues(double gripStrength,double svControlSignal,bool minForceEnabled,double minForce,std::vector<double> &forceTargetValue){

    if (numFingers == 2){

        forceTargetValue[0] = gripStrength - svControlSignal/2.0;
        forceTargetValue[1] = gripStrength + svControlSignal/2.0;

        if (minForceEnabled){
            forceTargetValue[0] = std::max(forceTargetValue[0],minForce);
            forceTargetValue[1] = std::max(forceTargetValue[1],minForce);
        }
    } else {

        forceTargetValue[0] = gripStrength - svControlSignal/3.0;
        forceTargetValue[1] = 0.5*(gripStrength + 2.0*svControlSignal/3.0);
        forceTargetValue[2] = 0.5*(gripStrength + 2.0*svControlSignal/3.0);

        if (minForceEnabled){
            forceTargetValue[0] = std::max(forceTargetValue[0],minForce);
            forceTargetValue[1] = std::max(forceTargetValue[1],minForce);
            forceTargetValue[2] = std::max(forceTargetValue[2],minForce);
        }
    }
}

bool ControlTask::manageObjectRecognitionTask(){

    tactileControl::ObjectRecognitionTaskState objRecState = getObjectRecognitionTaskState();

    switch (objRecState){

    case GRASP_STABILIZATION:
        // do nothing, just wait for stabilization
        break;

    case OBJECT_SQUEEZING:
        if (!gripStrengthIncreased){
            // increase the grip strenght
            taskData->set(PAR_CTRL_GRIP_STRENGTH, taskData->getDouble(PAR_ML_GRIP_STRENGTH_FOR_SQUEEZING));
            gripStrengthIncreased = true;
        }
        break;

    case TACTILE_DATA_COLLECTION:
        // sum up old and current tactile data (to be averaged later)
        for (int i = 0; i < NUM_TAXELS; i++){
            averageTactileData[i] += taskData->fingerTaxelsRawData[THUMB_FINGERTIP][i];
        }
        for (int i = 0; i < NUM_TAXELS; i++){
            averageTactileData[NUM_TAXELS + i] += taskData->fingerTaxelsRawData[MIDDLE_FINGERTIP][i];
        }
        break;

    case BEDING_PROXIMAL_JOINTS:
        if (!proximalJointsSetInOLMode){
            // set proximal joints in open loop mode
            controllerUtil->setControlMode(INDEX_PROXIMAL_JOINT, VOCAB_CM_PWM);
            controllerUtil->setControlMode(RING_LITTLE_JOINT, VOCAB_CM_PWM);
            proximalJointsSetInOLMode = true;
        }

        // send desired pwm to proximal joints
        controllerUtil->sendPwm(INDEX_PROXIMAL_JOINT, taskData->getDouble(PAR_ML_HAND_ENCLOSURE_INDEX_PROX_JOINT_PWM));
        controllerUtil->sendPwm(RING_LITTLE_JOINT, taskData->getDouble(PAR_ML_HAND_ENCLOSURE_RING_LITTLE_JOINT_PWM));

        break;

    case BENDING_DISTAL_JOINTS:
        if (!distalJointsSetInOLMode){
            // set proximal joints in open loop mode
            controllerUtil->setControlMode(INDEX_DISTAL_JOINT, VOCAB_CM_PWM);
            distalJointsSetInOLMode = true;
        }

        // send desired pwm to distal joints
        controllerUtil->sendPwm(INDEX_PROXIMAL_JOINT, taskData->getDouble(PAR_ML_HAND_ENCLOSURE_INDEX_PROX_JOINT_PWM));
        controllerUtil->sendPwm(RING_LITTLE_JOINT, taskData->getDouble(PAR_ML_HAND_ENCLOSURE_RING_LITTLE_JOINT_PWM));
        controllerUtil->sendPwm(RING_LITTLE_JOINT, taskData->getDouble(PAR_ML_HAND_ENCLOSURE_RING_LITTLE_JOINT_PWM));

        break;

    case TASK_COMPLETE:

        if (!featuresProcessingComplete){
            // compute tactile average
            double tactileDataCollectionCalls = getNumThreadCalls(taskData->getDouble(PAR_ML_TACTILE_DATA_COLLECTION_TIME));
            for (int i = 0; i < averageTactileData.size(); i++){
                averageTactileData[i] = averageTactileData[i] / tactileDataCollectionCalls;
            }
            ICubUtil::normalizeVector(averageTactileData, averageTactileDataNormalized);

            // put the tactile avarage into the features vector
            for (int i = 0; i < averageTactileDataNormalized.size(); i++){
                objRecFeatures[i] = averageTactileDataNormalized[i];
            }
            // put the encoders data into the features vector (the last component is not considered as it is always equal to 0)
            for (int i = 0; i < taskData->fingerEncodersRawData.size() - 1; i++){
                objRecFeatures[averageTactileDataNormalized.size() + i] = taskData->fingerEncodersRawData[i];
            }

            std::vector<double> featuresForClassification;

            switch (static_cast<tactileControl::ClassifierType>(taskData->getInt(PAR_ML_CLASSIFIER_TYPE))){

            case TACTILE_CLASSIFIER:
                featuresForClassification.swap(objRecFeatures);
                break;

            case MULTIMODAL_CLASSIFIER:
                mlUtil->testClassifierOneShot(objRecFeatures, taskData->tactileScores, false, TACTILE_CLASSIFIER);

                featuresForClassification.resize(taskData->visualScores.size() + taskData->tactileScores.size());
                for (int i = 0; i < taskData->visualScores.size(); i++){
                    featuresForClassification[i] = taskData->visualScores[i];
                }
                for (int i = 0; i < taskData->tactileScores.size(); i++){
                    featuresForClassification[taskData->visualScores.size() + i] = taskData->tactileScores[i];
                }
                break;
            }

            if (taskData->getBool(PAR_ML_DATA_COLLECTION_ENABLED)){
                mlUtil->addCollectedFeatures(featuresForClassification, taskData->getInt(PAR_ML_OBJECT_ID));
            }
            if (taskData->getBool(PAR_ML_OBJECT_CLASSIFICATION_ENABLED)){
                mlUtil->testClassifierOneShot(featuresForClassification, taskData->tactileScores, true, static_cast<tactileControl::ClassifierType>(taskData->getInt(PAR_ML_CLASSIFIER_TYPE)));
            }

            featuresProcessingComplete = true;
            taskData->objectRecognitionTaskComplete = true;
        }

        break;

    default:

        break;

    }

    return true;
}

tactileControl::ObjectRecognitionTaskState ControlTask::getObjectRecognitionTaskState(){

    tactileControl::ObjectRecognitionTaskState objRecState;

    double time = timeElapsed();

    if (time < startSqueezingTime){
        objRecState = GRASP_STABILIZATION;
    }
    else if (time < startTactileDataCollectionTime){
        objRecState = OBJECT_SQUEEZING;
    }
    else if (time < startBendingProximalJointsTime){
        objRecState = TACTILE_DATA_COLLECTION;
    }
    else if (time < startBendingDistalJointsTime){
        objRecState = BEDING_PROXIMAL_JOINTS;
    }
    else if (time < taskCompleteTime){
        objRecState = BENDING_DISTAL_JOINTS;
    }
    else {
        objRecState = TASK_COMPLETE;
    }

    return objRecState;
}

