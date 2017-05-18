#ifndef TACTILE_CONTROL_TASK_H
#define TACTILE_CONTROL_TASK_H

#include "TactileControl/data/TaskData.h"
#include "TactileControl/util/ControllerUtil.h"
#include "TactileControl/util/PortUtil.h"
#include "TactileControl/data/Enums.h"

#include <string>
#include <sstream>
#include <vector>

namespace tactileControl {

    class Task {

    private:

        bool isFirstCall;
        bool isClean;
        int pwmSign;

    protected:

        tactileControl::TaskData *taskData;
        tactileControl::ControllerUtil *controllerUtil;
        tactileControl::PortUtil *portUtil;

        tactileControl::TaskName taskName;

        std::vector<int> controlledJoints;
        std::vector<int> controlledFingers;
        std::vector<double> inputCommandValue;
        std::stringstream optionalLogStream;
        std::string taskId;
        int loggingBreak;
        bool loggingEnabled;
        int callsNumber;
        int maxCallsNumber;
        int taskThreadPeriod;

        /* ******* Debug attributes.                ******* */
        std::string dbgTag;


    public:

        Task(tactileControl::TaskData *taskData,tactileControl::ControllerUtil *controllerUtil,tactileControl::PortUtil *portUtil,double taskDuration);

        tactileControl::TaskName getTaskName(){ return taskName; }

        bool manage(bool keepActive);

        void clean();

        virtual std::string getTaskDescription(){ return "Unspecified task";}

    protected:

        int getNumThreadCalls(double seconds);

        double timeElapsed();

        void expandTargets(const std::vector<double> &targets,std::vector<double> &expandedTargets);

    private:

        void createTaskId();

        virtual void init(){};

        virtual void calculateControlInput() = 0;

        virtual void sendControlSignal();

        virtual void printScreenLog();

        virtual void saveProgress();

        virtual bool taskIsOver();

        virtual void release(){};

    };

} //namespace tactileControl

#endif

