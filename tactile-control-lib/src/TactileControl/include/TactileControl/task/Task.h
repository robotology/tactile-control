#ifndef TACTILE_CONTROL_TASK_H
#define TACTILE_CONTROL_TASK_H

#include "TactileControl/data/TaskData.h"
#include "TactileControl/util/ControllerUtil.h"
#include "TactileControl/util/PortUtil.h"
#include "TactileControl/data/Enums.h"


#include <string>

namespace tactileControl {

    class Task {

        private:

            bool isFirstCall;
            int taskThreadPeriod;
            int callsNumber;
            int maxCallsNumber;
            int loggingBreak;
            bool loggingEnabled;
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
            std::string optionalLogString;
            std::string taskId;


        //    /* ******* Debug attributes.                ******* */
            std::string dbgTag;


        public:

            Task(tactileControl::TaskData *taskData,tactileControl::ControllerUtil *controllerUtil,tactileControl::PortUtil *portUtil,double taskDuration);

            tactileControl::TaskName getTaskName(){ return taskName; }

            bool manage(bool keepActive);

            void clean();

        protected:

            int getNumThreadCalls(double seconds);

            double timeElapsed();

        private:

            void createTaskId();

            virtual void init(){};

            virtual void calculateControlInput() = 0;

            virtual void sendCommands();

            virtual void printScreenLog();

            virtual void saveProgress();

            virtual bool taskIsOver();

            virtual void release(){};

    };

} //namespace tactileControl

#endif

