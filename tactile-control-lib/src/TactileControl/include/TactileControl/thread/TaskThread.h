#ifndef TACTILECONTROL_TASKTHREAD_H
#define TACTILECONTROL_TASKTHREAD_H

#include "TactileControl/data/TaskData.h"
#include "TactileControl/util/ControllerUtil.h"
#include "TactileControl/util/PortUtil.h"
#include "TactileControl/task/Task.h"

#include <yarp/os/RateThread.h>

#include <vector>
#include <string>


namespace tactileControl {

    class TaskThread : public yarp::os::RateThread {

        private:

            tactileControl::TaskData *taskData;
            tactileControl::ControllerUtil *controllerUtil;
            tactileControl::PortUtil *portUtil;

            /* ****** Tasks management                              ****** */
            std::vector<Task*> taskList;
            int currentTaskIndex;
            bool runEnabled;

            ///* ****** Debug attributes                              ****** */
            std::string dbgTag;

        public:


            TaskThread(int period, tactileControl::TaskData *taskData,tactileControl::ControllerUtil *controllerUtil,tactileControl::PortUtil *portUtil);

            virtual bool threadInit();

            virtual void run();

            virtual void threadRelease();

            bool initializeGrasping();
            bool afterRun(bool fullyOpen,bool wait);

            bool addStepTask(const std::vector<double> &targets);

            bool addApproachTask();

            bool addControlTask();
            bool addControlTask(const std::vector<double> &targets);

            bool clearTaskList();

            std::string getTaskListDescription();

    };

} // namespace tactileControl

#endif // TACTILECONTROL_TASKTHREAD_H

