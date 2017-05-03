#include "TactileControl/thread/TaskThread.h"

#include "TactileControl/task/StepTask.h"
#include "TactileControl/task/ApproachTask.h"
#include "TactileControl/task/ControlTask.h"

using tactileControl::TaskThread;


TaskThread::TaskThread(int period, tactileControl::TaskData *taskData,tactileControl::ControllerUtil *controllerUtil,tactileControl::PortUtil *portUtil) : RateThread(period) {

        this->taskData = taskData;
        this->controllerUtil = controllerUtil;
        this->portUtil = portUtil;
        
        dbgTag = "TaskThread: ";
}


bool TaskThread::threadInit() {
    using std::cout;

    cout << dbgTag << "Initialising task thread. \n";

    currentTaskIndex = 0;

    // this prevents the run() method to be executed between the taskThread->start() and the taskThread->suspend() calls
    runEnabled = false;

    cout << dbgTag << "Task thread initialised correctly. \n";

    return true;
}


void TaskThread::run() {

    if (runEnabled){

        if (currentTaskIndex < taskList.size()){

            // if it is the last task of the list, keep it active until further tasks are added
            bool keepActive = (currentTaskIndex == taskList.size() - 1);

            bool taskIsActive = taskList[currentTaskIndex]->manage(keepActive);

            if (!taskIsActive){
                currentTaskIndex++;
            }
        }
    }
}


bool TaskThread::initializeGrasping(){

    if (!controllerUtil->saveCurrentControlMode()) return false;

    runEnabled = true;

    return true;
}


bool TaskThread::afterRun(bool openHand){

    if (!controllerUtil->restorePreviousControlMode()) return false;

    if (openHand) {
        if (!controllerUtil->openHand(false)) return false;
    }

    runEnabled = false;
    taskList[currentTaskIndex]->clean();
    currentTaskIndex = 0;
    
    // clear task list
    clearTaskList();

    return true;
}

void TaskThread::threadRelease() {

    if (runEnabled == true){
        taskList[currentTaskIndex]->clean();
        clearTaskList();
    }
}

bool TaskThread::addStepTask(const std::vector<double> &targets){

    taskList.push_back(new StepTask(taskData,controllerUtil,portUtil,targets));
}

bool TaskThread::addApproachTask(){

    taskList.push_back(new ApproachTask(taskData,controllerUtil,portUtil));
}

bool TaskThread::addControlTask(){

    taskList.push_back(new ControlTask(taskData,controllerUtil,portUtil));
}

bool TaskThread::addControlTask(const std::vector<double> &targets){

    taskList.push_back(new ControlTask(taskData,controllerUtil,portUtil,targets));
}

bool TaskThread::clearTaskList(){

    for (int i = 0; i < taskList.size(); i++){	
        delete(taskList[i]);
    }
    taskList.clear();
}

