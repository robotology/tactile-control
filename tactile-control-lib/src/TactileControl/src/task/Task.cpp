#include "TactileControl/task/Task.h"

#include "TactileControl/data/Parameters.h"

#include <yarp/os/LogStream.h>

#include <algorithm>
#include <ctime>
#include <string>
#include <sstream>


using tactileControl::Task;

Task::Task(tactileControl::TaskData *taskData,tactileControl::ControllerUtil *controllerUtil,tactileControl::PortUtil *portUtil,double taskDuration){

    this->taskData = taskData;
    this->controllerUtil = controllerUtil;
    this->portUtil = portUtil;

    taskData->getList(PAR_COMMON_CONTROLLED_JOINTS,controlledJoints);
    taskData->getControlledFingers(controlledFingers);

    inputCommandValue.resize(taskData->getFingerNum(),0.0);

    isFirstCall = true;
    taskThreadPeriod = taskData->getInt(PAR_COMMON_TASK_THREAD_PERIOD);
    callsNumber = 0;
    maxCallsNumber = getNumThreadCalls(taskDuration);
    loggingBreak = std::max(1,static_cast<int>((1000.0/taskData->getInt(PAR_COMMON_SCREEN_LOGGING_RATE))/taskThreadPeriod));
    loggingEnabled = taskData->getBool(PAR_COMMON_ENABLE_SCREEN_LOGGING);
    isClean = false;
    pwmSign = taskData->getInt(PAR_COMMON_PWM_SIGN);
    optionalLogStream.str(std::string());
}


bool Task::manage(bool keepActive){
    
    // initialize the task
    if (isFirstCall){
        createTaskId();
        init();
        isFirstCall = false;
    }

    // calculate pwm or velocities to send to the robot
    calculateControlInput();

    // send the control signal to the robot
    sendControlSignal();

    // send task data to port
    portUtil->sendInfoData(taskData);

    printScreenLog();

    saveProgress();

    if (taskIsOver() && !keepActive){
        clean();
        return false;
    }

    return true;
}

int Task::getNumThreadCalls(double seconds){

    return static_cast<int>(seconds*1000/taskThreadPeriod);
}

double Task::timeElapsed(){

    return callsNumber*taskThreadPeriod/1000.0;
}

void Task::expandTargets(const std::vector<double> &targets,std::vector<double> &expandedTargets){

    expandedTargets.resize(controlledJoints.size());
    for(int i = 0; i < expandedTargets.size(); i++){
        expandedTargets[i] = (i >= targets.size() ? targets[targets.size()-1] : targets[i]);
    }
}

void Task::createTaskId(){

    // create a unique identifier for the task
    time_t now = time(0);
    tm *ltm = localtime(&now);
    char myDate[15];
    strftime(myDate,15,"%m%d%H%M%S",ltm);
    
    taskId = std::string(myDate);
}

void Task::sendControlSignal(){

    for(int i = 0; i < inputCommandValue.size(); i++){
        controllerUtil->sendPwm(controlledJoints[i],pwmSign*inputCommandValue[i]);
    }
}

void Task::printScreenLog(){

    if (loggingEnabled && callsNumber%loggingBreak == 0){

        std::stringstream standardLogStream("");

        standardLogStream << dbgTag << "Fng: ";
    
        for(int i = 0; i < controlledFingers.size(); i++){
            standardLogStream << taskData->overallFingerForce[controlledFingers[i]] << "(" << controlledFingers[i] << ") ";
        }
        standardLogStream << "\t   In: ";

        for(int i = 0; i < controlledJoints.size(); i++){
            standardLogStream << inputCommandValue[i] << "(" << controlledJoints[i] << ") ";
        }
    
        yInfo() << standardLogStream.str() << optionalLogStream.str();
    }

    optionalLogStream.str(std::string());
}


void Task::clean(){

    if (!isClean){
        release();
        isClean = true;
    }
}


void Task::saveProgress(){

    callsNumber++;
}

bool Task::taskIsOver(){

    return callsNumber >= maxCallsNumber;
}

