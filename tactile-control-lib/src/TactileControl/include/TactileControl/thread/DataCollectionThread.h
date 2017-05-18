#ifndef TACTILECONTROL_DATACOLLECTION_H
#define TACTILECONTROL_DATACOLLECTION_H

#include "TactileControl/data/TaskData.h"
#include "TactileControl/util/ControllerUtil.h"
#include "TactileControl/util/PortUtil.h"

#include <yarp/os/RateThread.h>


namespace tactileControl {
        
    class DataCollectionThread : public yarp::os::RateThread {


    private:

        tactileControl::TaskData *taskData;
        tactileControl::ControllerUtil *controllerUtil;
        tactileControl::PortUtil *portUtil;

        /* ****** Debug attributes                              ****** */
        std::string dbgTag;

    public:

        DataCollectionThread(int period, tactileControl::TaskData *taskData,tactileControl::ControllerUtil *controllerUtil,tactileControl::PortUtil *portUtil);

        virtual void run();

        bool updateRobotData();

    private:

        bool processTactileData();


    };

} //namespace tactileControl

#endif // TACTILECONTROL_DATACOLLECTION_H

