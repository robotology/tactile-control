#ifndef TACTILECONTROL_TASKTHREAD_H
#define TACTILECONTROL_TASKTHREAD_H

#include "TactileControl/data/TaskData.h"
#include "TactileControl/util/ControllerUtil.h"
#include "TactileControl/util/PortUtil.h"
#include "TactileControl/task/Task.h"

#include <yarp/os/RateThread.h>

#include <vector>

//#include "iCub/plantIdentification/task/Task.h"
//#include "iCub/plantIdentification/data/RPCCommandsData.h"
//#include "iCub/plantIdentification/data/LogData.h"
//#include "iCub/plantIdentification/util/ControllersUtil.h"
//#include "iCub/plantIdentification/util/PortsUtil.h"
//#include "iCub/plantIdentification/util/MLUtil.h"
//#include "iCub/plantIdentification/PlantIdentificationEnums.h"
//
//#include <yarp/os/ResourceFinder.h>
//#include <yarp/os/Value.h>
//#include <yarp/sig/Vector.h>
//
//#include <fstream>
//#include <string>
//#include <deque>

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
            bool afterRun(bool openHand);

            bool addStepTask(const std::vector<double> &targets);

            bool addApproachTask();

            bool addControlTask(const std::vector<double> &targets);

            bool clearTaskList();


    };

} // namespace tactileControl

#endif // TACTILECONTROL_TASKTHREAD_H

