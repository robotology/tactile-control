#include "TactileControl/util/ControllerUtil.h"

#include "TactileControl/data/Parameters.h"
#include "TactileControl/util/ICubUtil.h"

#include <vector>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>


using tactileControl::ControllerUtil;


ControllerUtil::ControllerUtil(){

    dbgTag = "ControllerUtil: ";
}

bool ControllerUtil::init(tactileControl::TaskData *taskData){
    using std::string;
    using std::cout;


    storedHandJointsControlMode.resize(NUM_HAND_JOINTS,VOCAB_CM_POSITION);
    storedHandJointsMaxPwmLimits.resize(NUM_HAND_JOINTS);

    taskData->getList(PAR_COMMON_OPEN_HAND_JOINTS,openHandJoints);

    numFingers = taskData->getFingerNum();

    string portPrefix = taskData->getString(PAR_COMMON_PORT_PREFIX);
    string iCub = taskData->getString(PAR_COMMON_ICUB);
    string hand = taskData->getString(PAR_COMMON_HAND);

    string portFullPrefix;
    if (taskData->getBool(PAR_COMMON_USING_TWO_HANDS)){
        portFullPrefix = "/" + portPrefix + "/" + hand + "_hand";
    } else {
        portFullPrefix = "/" + portPrefix;
    }


    /* ******* Joint interfaces                     ******* */
    string robotPart = hand + "_arm";
    yarp::os::Property options;
    options.put("robot", iCub.c_str()); 
    options.put("device", "remote_controlboard");
    options.put("part", robotPart.c_str());
    options.put("local", (portFullPrefix).c_str());
    options.put("remote", ("/" + iCub + "/" + robotPart).c_str());
    
    // Open driver
    if (!clientArm.open(options)) {
        cout << dbgTag << "could not open arm driver\n";
        return false;
    }
    // Open interfaces
    clientArm.view(iEncs);
    if (!iEncs) {
        cout << dbgTag << "could not open encoders interface\n";
        return false;
    }
    clientArm.view(iPwm);
    if (!iPwm) {
        cout << dbgTag << "could not open pwm interface\n";
        return false;
    }
    clientArm.view(iCtrl);
    if (!iCtrl) {
        cout << dbgTag << "could not open control mode interface\n";
        return false;
    }
    clientArm.view(iPos);
    if (!iPos) {
        cout << dbgTag << "could not open position interface\n";
        return false;
    }
    clientArm.view(iPosCtrl);
    if (!iPosCtrl) {
        cout << dbgTag << "could not open position control interface\n";
        return false;
    }
    clientArm.view(iVel);
    if (!iVel) {
        cout << dbgTag << "could not open velocity interface\n";
        return false;
    }
    clientArm.view(iPosDir);
    if (!iPosDir) {
        cout << dbgTag << "could not open position direct interface\n";
        return false;
    }
    clientArm.view(iPid);
    if (!iPid) {
        cout << dbgTag << "could not open pid interface\n";
        return false;
    }

    // initialize encoders data
    int numArmJoints;
    iPos->getAxes(&numArmJoints);
    taskData->initEncodersData(numArmJoints);
    armJointControlModes.resize(numArmJoints);
    for(int i = 0; i < armJointControlModes.size(); i++){
        iCtrl->getControlMode(i,&armJointControlModes[i]);
    }

    // Set reference speeds
    for(int i = 0; i < NUM_HAND_JOINTS; i++){
        iPos->setRefSpeed(FIRST_HAND_JOINT + i,taskData->getDouble(PAR_COMMON_REF_VELOCITY));
    }

    return true;
}


bool ControllerUtil::sendPwm(int joint,double pwm){

    double dutyCycle = (pwm/1333)*100;
    if (!iPwm->setRefDutyCycle(joint,dutyCycle)){
        std::cout << dbgTag << "could not send pwm\n";
        return false;
    }
    return true;
}

bool ControllerUtil::sendVelocity(int joint,double velocity){
    
    if (!iVel->velocityMove(joint,velocity)){
        std::cout << dbgTag << "could not send velocity\n";
        return false;
    }
    return true;
}


bool ControllerUtil::getArmEncoderAngles(std::vector<double> &armEncoderAngles,bool wait){
    using yarp::os::Time;
    
    yarp::sig::Vector armEncoderAnglesVector;
    armEncoderAnglesVector.resize(NUM_HAND_JOINTS);

    bool encodersDataAcquired = iEncs->getEncoders(armEncoderAnglesVector.data());

    while(wait && !encodersDataAcquired) {

        Time::delay(0.1);

        encodersDataAcquired = iEncs->getEncoders(armEncoderAnglesVector.data());
    }

    if (encodersDataAcquired){
        
        for(int i = 0; i < armEncoderAngles.size(); i++){
            armEncoderAngles[i] = armEncoderAnglesVector[i];
        }
        return true;
    }

    return false;
}

bool ControllerUtil::getArmEncoderAngleReferences(std::vector<double> &armEncoderAngleReferences,bool wait){

    using yarp::os::Time;
    
    for(int i = 0; i < armEncoderAngleReferences.size(); i++){

        switch(armJointControlModes[i]){

        case VOCAB_CM_POSITION:
            iPosCtrl->getTargetPosition(i,&armEncoderAngleReferences[i]);
            break;
        case VOCAB_CM_PWM:
            iEncs->getEncoder(i,&armEncoderAngleReferences[i]);
            break;
        case VOCAB_CM_VELOCITY:
            iEncs->getEncoder(i,&armEncoderAngleReferences[i]);
            break;
        case VOCAB_CM_POSITION_DIRECT:
            iPosDir->getRefPosition(i,&armEncoderAngleReferences[i]);
            break;
        }
    }

    return true;
}



bool ControllerUtil::saveCurrentControlMode(){

    for(int i = 0; i < NUM_HAND_JOINTS; i++){
        if (!iCtrl->getControlMode(FIRST_HAND_JOINT + i,&storedHandJointsControlMode[i])){
            std::cout << dbgTag << "could not get current control mode\n";
            return false;
        }
    }
    return true;
}

bool ControllerUtil::restorePreviousControlMode(){

    for(int i = 0; i < NUM_HAND_JOINTS; i++){
        if (!iCtrl->setControlMode(FIRST_HAND_JOINT + i,storedHandJointsControlMode[i])){
            std::cout << dbgTag << "could not set control mode\n";
            return false;
        }
    }
    return true;

}

bool ControllerUtil::setControlMode(int joint,int controlMode){

    if (!iCtrl->setControlMode(joint,controlMode)){
        std::cout << dbgTag << "could not set control mode\n";
        return false;
    } else {
        armJointControlModes[joint] = controlMode;
    }
    return true;
}

bool ControllerUtil::setControlMode(const std::vector<int> &jointsList,int controlMode){

    for(int i = 0; i < jointsList.size(); i++){
        if (!iCtrl->setControlMode(jointsList[i],controlMode)){
            std::cout << dbgTag << "could not set control mode\n";
            return false;
        } else {
            armJointControlModes[jointsList[i]] = controlMode;
        }
    }
    return true;
}

bool ControllerUtil::waitMotionDone(double timeout, double delay) {
    using yarp::os::Time;

    double startTime = Time::now();

    bool motionDone = isMotionDone();

    while (!motionDone && (Time::now() - startTime < timeout)) {
        Time::delay(delay);
        motionDone = isMotionDone();
    }

    return motionDone;
}

bool ControllerUtil::isMotionDone(){

    bool done;
    iPos->checkMotionDone(&done);

    return done;
}

bool ControllerUtil::getEncoderAngle(int joint,double &encoderData){
    
    bool ok;

    ok = iEncs->getEncoder(joint,&encoderData);

    if (!ok){
        std::cout << dbgTag << "could not get encoder value\n";
    }
    return ok;
}


bool ControllerUtil::openHand(bool fullyOpen,bool wait) {
    using tactileControl::ICubUtil;

    iVel->stop();

    for(int i = 0; i < NUM_HAND_JOINTS; i++){

        double jointAngle;

        if (fullyOpen && ICubUtil::isDistal(FIRST_HAND_JOINT + i) || i == INDEX_DISTAL_JOINT && numFingers == 2){

            jointAngle = STRAIGHT_DISTAL_ANGLE;

        } else {

            jointAngle = openHandJoints[i];
        }

        iPos->positionMove(FIRST_HAND_JOINT + i,jointAngle);

    }

    if (wait == true){
        // Wait for the hand to be open
        std::cout << dbgTag << "Opening hand... \t";
        waitMotionDone(10, 1);
        std::cout << "done \n";
    }


    return true;
}


bool ControllerUtil::setJointMaxPwmLimit(int joint,double maxPwm){

    yarp::dev::Pid pid;

    if (iPid->getPid(joint,&pid)){

        pid.max_output = maxPwm;

        iPid->setPid(joint,pid);

        return true;
    }

    return false;
}

bool ControllerUtil::setJointsMaxPwmLimit(const std::vector<int> &jointsList,const std::vector<double> &maxPwmList){

    bool ok = true;

    for(int i = 0; i < jointsList.size(); i++){

        ok = ok && setJointMaxPwmLimit(jointsList[i],maxPwmList[i]);
    }

    return ok;
}

bool ControllerUtil::saveHandJointsMaxPwmLimits(){

    for(int i = 0; i < NUM_HAND_JOINTS; i++){

        yarp::dev::Pid pid;

        if (iPid->getPid(FIRST_HAND_JOINT + i,&pid)){

            storedHandJointsMaxPwmLimits[i] = pid.max_output;

        } else {

            std::cout << dbgTag << "could not get pid from joint " << FIRST_HAND_JOINT + i << "\n";

            return false;
        }
    }

    return true;
}

bool ControllerUtil::restoreHandJointsMaxPwmLimits(){


    for(int i = 0; i < NUM_HAND_JOINTS; i++){

        yarp::dev::Pid pid;

        if (iPid->getPid(FIRST_HAND_JOINT + i,&pid)){

            pid.max_output = storedHandJointsMaxPwmLimits[i];

            iPid->setPid(FIRST_HAND_JOINT + i,pid);
        } else {

            std::cout << dbgTag << "could not get pid from joint " << FIRST_HAND_JOINT + i << "\n";

            return false;
        }
    }

    return true;
}

bool ControllerUtil::resetPIDIntegralGain(int joint){

    yarp::dev::Pid pid;

    if (iPid->getPid(joint,&pid)){

        storedPIDIntegralGain = pid.ki;

        pid.setKi(0);

        iPid->setPid(joint,pid);

        return true;

    } else {

        std::cout << dbgTag << "could not get pid from joint " << joint << "\n";

        return false;
    }

    return false;
}

bool ControllerUtil::restorePIDIntegralGain(int joint){

    yarp::dev::Pid pid;

    if (iPid->getPid(joint,&pid)){

        pid.setKi(storedPIDIntegralGain);

        iPid->setPid(joint,pid);

        return true;

    } else {

        std::cout << dbgTag << "could not get pid from joint " << joint << "\n";

        return false;
    }

    return false;
}

bool ControllerUtil::setJointAngle(int joint,double angle){

    iPos->positionMove(joint,angle);

    return true;
}

bool ControllerUtil::setJointAnglePositionDirect(int joint,double angle){

    iPosDir->setPosition(joint,angle);

    return true;
}

bool ControllerUtil::release(){

    if (!clientArm.close()){

        std::cout << dbgTag << "could not close arm driver\n";

        return false;
    }

    return true;
}

