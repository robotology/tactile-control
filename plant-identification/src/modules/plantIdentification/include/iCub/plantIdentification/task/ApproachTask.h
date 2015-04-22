#ifndef __ICUB_PLANTIDENTIFICATION_APPROACHTASK_H__
#define __ICUB_PLANTIDENTIFICATION_APPROACHTASK_H__

#include "iCub/plantIdentification/task/Task.h"
#include "iCub/plantIdentification/data/TaskData.h"
#include "iCub/plantIdentification/util/ControllersUtil.h"
#include "iCub/plantIdentification/util/PortsUtil.h"

#include <iCub/ctrl/pids.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>

namespace iCub {
    namespace plantIdentification {

        class ApproachTask : public Task {

            private:

				iCub::plantIdentification::ApproachTaskData *approachData;
				
				std::vector<bool> fingerIsInContact;

            public:

                ApproachTask(iCub::plantIdentification::ControllersUtil *controllersUtil,iCub::plantIdentification::PortsUtil *portsUtil,iCub::plantIdentification::TaskCommonData *commonData,iCub::plantIdentification::ApproachTaskData *approachData);

				virtual void init();

				virtual void buildLogData(LogData &logData);

				virtual void calculateControlInput();

				virtual void sendCommands();

				virtual void release();

				virtual bool taskIsOver();

				bool eachFingerIsInContact();

        };
    } //namespace plantIdentification
} //namespace iCub

#endif
