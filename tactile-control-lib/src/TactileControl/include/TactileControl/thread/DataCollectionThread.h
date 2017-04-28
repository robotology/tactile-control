#ifndef TACTILECONTROL_DATACOLLECTION_H
#define TACTILECONTROL_DATACOLLECTION_H

#include "tactileControl/data/TaskData.h"
#include "tactileControl/util/ControllerUtil.h"
#include "tactileControl/util/PortUtil.h"

#include <yarp/os/RateThread.h>

//#include "iCub/plantIdentification/action/WaveAction.h"
//#include "iCub/plantIdentification/PlantIdentificationEnums.h"
//
//#include <yarp/os/ResourceFinder.h>
//
//#include <vector>
//#include <string>

namespace tactileControl {
        
    class DataCollectionThread : public yarp::os::RateThread {
           

        private:

            tactileControl::TaskData *taskData;
            tactileControl::ControllerUtil *controllerUtil;
            tactileControl::PortUtil *portUtil;

            /* ****** Debug attributes                              ****** */
            std::string dbgTag;

        public:

            DataCollectionThread(int period,tactileControl::ControllerUtil *controllerUtil,tactileControl::PortUtil *portUtil,tactileControl::TaskData *taskData);

            virtual void run();

            bool updateRobotData();

        private:

            bool processTactileData();


    };

} //namespace tactileControl

#endif // TACTILECONTROL_DATACOLLECTION_H

