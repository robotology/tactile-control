#include "TactileControl/data/TaskData.h"

#include "TactileControl/data/Parameters.h"
#include "TactileControl/data/Enums.h"
#include "TactileControl/util/ICubUtil.h"



using tactileControl::TaskData;


TaskData::TaskData() {
    
    dbgTag = "TaskData: ";
}

bool TaskData::init(const yarp::os::Property &options) {

    using yarp::os::Value;

    this->options = new yarp::os::Property(options);

    /*** SET DEFAULT VALUES ***/

    Value trueValue("true");
    Value falseValue("false");
    setDefault(PAR_COMMON_TASK_THREAD_PERIOD,20);
    setDefault(PAR_COMMON_DATA_COLLECTION_THREAD_PERIOD,15);
    setDefault(PAR_COMMON_PORT_PREFIX,Value("stableGrasp"));
    setDefault(PAR_COMMON_HAND,Value("left"));
    setDefault(PAR_COMMON_ICUB,Value("iCubGenova01"));
    setDefault(PAR_COMMON_CONTROLLED_JOINTS,*Value::makeList("9 11 13"));
    setDefault(PAR_COMMON_PWM_SIGN,1);
    setDefault(PAR_COMMON_FINGER_SENSITIVITY,*Value::makeList("1.0 0.6 0.6"));
    setDefault(PAR_COMMON_OPEN_HAND_JOINTS,*Value::makeList("23 3 40 2 40 3 40 1"));
    setDefault(PAR_COMMON_DISABLE_PWM,falseValue);
    setDefault(PAR_COMMON_TACT_MEDIAN_WINDOW_SIZE,50);
    setDefault(PAR_COMMON_REF_VELOCITY,100.0);
    setDefault(PAR_COMMON_USING_TWO_HANDS,trueValue);
    setDefault(PAR_COMMON_EXPERIMENT_INFO,Value("experimentInfo"));
    setDefault(PAR_COMMON_EXPERIMENT_OPTIONAL_INFO,Value("experimentExtra"));
    setDefault(PAR_COMMON_USE_TACTILE_WEIGHTED_SUM,trueValue);
    setDefault(PAR_COMMON_ENABLE_SCREEN_LOGGING,falseValue);
    setDefault(PAR_COMMON_SCREEN_LOGGING_RATE,5);

    setDefault(PAR_STEP_DURATION,10);

    setDefault(PAR_RAMP_DURATION,10);
    setDefault(PAR_RAMP_DURATION_AFTER_STABILIZATION,5);
    setDefault(PAR_RAMP_SLOPE,-0.0025);
    setDefault(PAR_RAMP_INTERCEPT,-90.0);

    setDefault(PAR_APPR_DURATION,5);
    setDefault(PAR_APPR_VELOCITY,*Value::makeList("20 20 20"));
    setDefault(PAR_APPR_MAX_PWM,*Value::makeList("450 350 350"));
    setDefault(PAR_APPR_PWM_LIMIT_ENABLED,trueValue);
    setDefault(PAR_APPR_WINDOW_SIZE,25);
    setDefault(PAR_APPR_THRESHOLD,1.5);
    setDefault(PAR_APPR_TIMEOUT,0.7);

    setDefault(PAR_CTRL_DURATION,10);
    setDefault(PAR_CTRL_DEFAULT_FORCE_TARGET,20.0);
    setDefault(PAR_CTRL_LOW_PID_KP,*Value::makeList("4.8 4.8 4.8"));
    setDefault(PAR_CTRL_LOW_PID_KI,*Value::makeList("10.6 10.6 10.6"));
    setDefault(PAR_CTRL_LOW_PID_WP,1.0);
    setDefault(PAR_CTRL_LOW_PID_WI,1.0);
    setDefault(PAR_CTRL_LOW_PID_WD,1.0);
    setDefault(PAR_CTRL_LOW_PID_N,1.0);
    setDefault(PAR_CTRL_LOW_PID_WIND_UP_COEFF,0.5);
    setDefault(PAR_CTRL_LOW_PID_MIN_SAT_LIM,-2666.0);
    setDefault(PAR_CTRL_LOW_PID_MAX_SAT_LIM,2666.0);
    setDefault(PAR_CTRL_LOW_PID_SCALE,0.0);
    setDefault(PAR_CTRL_LOW_PID_INTEGRAL_DISABLED,trueValue);
    setDefault(PAR_CTRL_SUPERVISOR_ENABLED,trueValue);
    setDefault(PAR_CTRL_HIGH_PID_KP,30);
    setDefault(PAR_CTRL_HIGH_PID_KI,0.5);
    setDefault(PAR_CTRL_HIGH_PID_KD,0.8);
    setDefault(PAR_CTRL_HIGH_PID_WP,1.0);
    setDefault(PAR_CTRL_HIGH_PID_WI,1.0);
    setDefault(PAR_CTRL_HIGH_PID_WD,1.0);
    setDefault(PAR_CTRL_HIGH_PID_N,1.0);
    setDefault(PAR_CTRL_HIGH_PID_WIND_UP_COEFF,0.5);
    setDefault(PAR_CTRL_HIGH_PID_MIN_SAT_LIM,-100000.0);
    setDefault(PAR_CTRL_HIGH_PID_MAX_SAT_LIM,100000.0);
    setDefault(PAR_CTRL_HIGH_PID_SCALE,1.0);
    setDefault(PAR_CTRL_GRIP_STRENGTH,50.0);
    setDefault(PAR_CTRL_THUMB_ABDUCTION_OFFSET,-30.0);
    setDefault(PAR_CTRL_MIN_JERK_TRACK_ENABLED,trueValue);
    setDefault(PAR_CTRL_MIN_JERK_TRACK_REF_TIME,2.0);
    setDefault(PAR_CTRL_SUPERVISOR_MODE,GMM_MODE);
    setDefault(PAR_CTRL_GMM_BEST_POSE_LOG_ONE_SHOT,falseValue);
    setDefault(PAR_CTRL_GMM_JOINTS_REGRESSION_ENABLED,falseValue);
    setDefault(PAR_CTRL_GMM_JOINTS_MIN_JERK_TRACK_ENABLED,trueValue);
    setDefault(PAR_CTRL_GMM_JOINTS_MIN_JERK_TRACK_REF_TIME,4.0);
    setDefault(PAR_CTRL_MIN_FORCE_ENABLED,falseValue);
    setDefault(PAR_CTRL_MIN_FORCE,0.0);

    
    /*** INITIALIZE DATA ***/

    fingerTaxelsData.resize(NUM_FINGERS);
    for(int i = 0; i < fingerTaxelsData.size(); i++){
        fingerTaxelsData[i].resize(NUM_TAXELS,0.0);
    }
    fingerTaxelsRawData.resize(NUM_FINGERS);
    for(size_t i = 0; i < fingerTaxelsRawData.size(); i++){
        fingerTaxelsRawData[i].resize(NUM_TAXELS,0.0);
    }
    previousOverallFingerForce.resize(NUM_FINGERS);
    for(size_t i = 0; i < previousOverallFingerForce.size(); i++){
        previousOverallFingerForce[i].resize(getInt(PAR_COMMON_TACT_MEDIAN_WINDOW_SIZE),0.0);
    }
    previousForceIndex.resize(NUM_FINGERS,0);
    overallFingerForce.resize(NUM_FINGERS,0.0);
    overallFingerForceBySimpleSum.resize(NUM_FINGERS,0.0);
    overallFingerForceByWeightedSum.resize(NUM_FINGERS,0.0);
    overallFingerForceMedian.resize(NUM_FINGERS,0.0);

    fingerEncodersRawData.resize(NUM_HAND_ENCODERS,0.0);

    // control task data
    gmmDataStandard = new GMMData(STANDARD_GMM);
    graspIsStable - false;

    return true;
}

bool TaskData::initEncodersData(int numArmJoints){

    armEncoderAngles.resize(numArmJoints,0.0);      
    armEncoderAngleReferences.resize(numArmJoints,0.0);      
}

void TaskData::set(const yarp::os::ConstString &key,const yarp::os::Value &value,bool overwrite){

    if (overwrite || !options->check(key)){
        options->put(key,value);
    }
}

void TaskData::setDefault(const yarp::os::ConstString &key,const yarp::os::Value &value){

    set(key,value,false);
}

void TaskData::setToList(const yarp::os::ConstString &key,const yarp::os::Value &value,int index){

    using yarp::os::Bottle;
    using yarp::os::Value;

    Bottle newData;
    Bottle *oldData = options->find(key).asList();

    newData.clear();

    for(int i = 0; i < oldData->size(); i++){
        if (i == index){
            newData.add(value);
        }
        else {
            newData.add(oldData->get(i));
        }
    }

    options->put(key,*Value::makeList(newData.toString().c_str()));
}




int TaskData::getInt(const yarp::os::ConstString &key){

    return options->find(key).asInt();
}

double TaskData::getDouble(const yarp::os::ConstString &key){

    return options->find(key).asDouble();
}

std::string TaskData::getString(const yarp::os::ConstString &key){

    return options->find(key).asString();
}

bool TaskData::getBool(const yarp::os::ConstString &key){

    return getString(key) == "true";
}

yarp::os::Value TaskData::getValueFromList(const yarp::os::ConstString &key,int index){

    return options->find(key).asList()->get(index);
}

int TaskData::getIntFromList(const yarp::os::ConstString &key,int index){

    return getValueFromList(key,index).asInt();
}

double TaskData::getDoubleFromList(const yarp::os::ConstString &key,int index){

    return getValueFromList(key,index).asDouble();
}

std::string TaskData::getStringFromList(const yarp::os::ConstString &key,int index){

    return getValueFromList(key,index).asString();
}

bool TaskData::getBoolFromList(const yarp::os::ConstString &key,int index){

    return getValueFromList(key,index).asString() == "true";
}

void TaskData::getList(const yarp::os::ConstString &key,std::vector<yarp::os::Value> &list){

    yarp::os::Bottle *data = options->find(key).asList();

    list.resize(data->size());

    for(int i = 0; i < data->size(); i++){
        list[i] = data->get(i);
    }
}

void TaskData::getList(const yarp::os::ConstString &key,std::vector<int> &list){

    yarp::os::Bottle *data = options->find(key).asList();

    list.resize(data->size());

    for(int i = 0; i < data->size(); i++){
        list[i] = data->get(i).asInt();
    }
}

void TaskData::getList(const yarp::os::ConstString &key,std::vector<double> &list){

    yarp::os::Bottle *data = options->find(key).asList();

    list.resize(data->size());

    for(int i = 0; i < data->size(); i++){
        list[i] = data->get(i).asDouble();
    }
}

void TaskData::getList(const yarp::os::ConstString &key,std::vector<std::string> &list){

    yarp::os::Bottle *data = options->find(key).asList();

    list.resize(data->size());

    for(int i = 0; i < data->size(); i++)
        list[i] = data->get(i).asString();
}

void TaskData::getList(const yarp::os::ConstString &key,std::vector<bool> &list){

    yarp::os::Bottle *data = options->find(key).asList();

    list.resize(data->size());

    for(int i = 0; i < data->size(); i++){
        list[i] = data->get(i).asString() == "true";
    }
}

void TaskData::getControlledFingers(std::vector<int> &controlledFingers){

    std::vector<int> controlledJoints;

    getList(PAR_COMMON_CONTROLLED_JOINTS,controlledJoints);
    controlledFingers.resize(controlledJoints.size());

    for(int i = 0; i < controlledFingers.size(); i++){
        controlledFingers[i] = ICubUtil::getFingerFromJoint(controlledJoints[i]);
    }

}

int TaskData::getFingerNum(){

    std::vector<int> controlledFingers;

    getControlledFingers(controlledFingers);
    return controlledFingers.size();
}


tactileControl::SupervisorMode TaskData::getSupervisorControlMode(){
    
    return static_cast<tactileControl::SupervisorMode>(getInt(PAR_CTRL_SUPERVISOR_MODE));
}

void TaskData::release(){

    delete(options);
    delete(gmmDataStandard);
}