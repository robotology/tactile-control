#include "TactileControl/task/StepTask.h"

#include "TactileControl/data/Parameters.h"

#include <sstream>

using tactileControl::StepTask;


StepTask::StepTask(TaskData *taskData,ControllerUtil * controllerUtil,PortUtil * portUtil,const std::vector<double> &targets):Task(taskData,controllerUtil,portUtil,taskData->getDouble(PAR_STEP_DURATION)){

    expandTargets(targets,constantPwm);

    taskName = STEP;

    dbgTag = "StepTask: ";
}

void StepTask::init(){
    using std::cout;

    controllerUtil->setControlMode(controlledJoints,VOCAB_CM_OPENLOOP);

    if (loggingEnabled){

        cout << "\n\n";
        cout << dbgTag << "TASK STARTED - Target: ";
        for(int i = 0; i < constantPwm.size(); i++){
            cout << constantPwm[i] << " ";
        }
        cout << "\n\n";
    }

}

void StepTask::calculateControlInput(){

    for(int i = 0; i < constantPwm.size(); i++){
        inputCommandValue[i] = constantPwm[i];
    }
}

std::string StepTask::getTargetsDescription(){

    std::stringstream description("");

    for(int i = 0; i < constantPwm.size(); i++){
        description << constantPwm[i] << " ";
    }
    
    return description.str();
}
