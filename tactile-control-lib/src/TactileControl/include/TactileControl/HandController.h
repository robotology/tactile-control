#ifndef TACTILECONTROL_HANDCONTROLLER_H
#define TACTILECONTROL_HANDCONTROLLER_H

#include "TactileControl/data/TaskData.h"
#include "TactileControl/util/ControllerUtil.h"
#include "TactileControl/util/PortUtil.h"
#include "TactileControl/thread/TaskThread.h"
#include "TactileControl/thread/DataCollectionThread.h"

#include <yarp/os/Property.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/Value.h>

namespace tactileControl {

/**
* \class tactileControl::HandController
* \headerfile template-lib.h <TemplateLib/templatelib.h>
*
* \brief A class from TactileControl namespace.
*
*/

    class HandController {

    public:

        tactileControl::TaskData *taskData;
        tactileControl::ControllerUtil *controllerUtil;
        tactileControl::PortUtil *portUtil;

        bool taskRunning;

        /* ****** Threads                                 ****** */
        tactileControl::TaskThread *taskThread;
        tactileControl::DataCollectionThread *dataCollectionThread;

        /* ****** Debug attributes                              ****** */
        std::string dbgTag;

    public:
        /**
        * Constructor
        */
        HandController();

        /**
        * Destructor
        */
        ~HandController();

        /**
        * Initializes the hand controller. Returns true in case of success.
        */
        bool open();

        /**
        * Sets the given properties and initializes the hand controller. Returns true in case of success.
        */
        bool open(const yarp::os::Property &options);

        /**
        * Sets the given property.
        */
        void set(const yarp::os::ConstString key,const yarp::os::Value &value);

        /**
        * Gets the given property. Returns true in case of success.
        */
        bool get(const yarp::os::ConstString key,yarp::os::Value &value);

        /**
        * Starts the grasping task (including the approach phase). If wait == true it waits for the grasp to be stable, otherwise it returns immediately. Returns true in case of success.
        */
        bool closeHand(bool wait = true);

        /**
        * Checks if the grasp is stable.
        */
        bool isHandClose();

        /**
        * Opens the hand and stops any running task. If wait == true it waits for the hand to be open, otherwise it returns immediately. Returns true in case of success.
        */
        bool openHand(bool fullyOpen, bool wait = true);

        /**
        * Checks if the hand is open.
        */
        bool isHandOpen();

        /**
        * Sets the minimum value of force at each fingertip used for grasping.
        */
        void setMinForce(double minForce);

        /**
        * Disables the minimum force functionality.
        */
        void disableMinForce();

        /**
        * Sets the grip strength.
        */
        void setGripStrength(double gripStrength);

        /**
        * Add the step task to the task list. Returns true in case of success.
        */
        bool addStepTask(const std::vector<double> &targets);

        /**
        * Add the approach task to the task list. Returns true in case of success.
        */
        bool addApproachTask();

        /**
        * Add the control task to the task list. Returns true in case of success.
        */
        bool addControlTask();

        /**
        * Add the control task to the task list. Returns true in case of success.
        */
        bool addControlTask(const std::vector<double> &targets);

        /**
        * Clears the task list. Returns true in case of success.
        */
        bool clearTaskList();

        /**
        * Starts all the tasks in the task list. Returns true in case of success.
        */
        bool startTask();

        /**
        * Returns a text description of the data
        */
        std::string getDataDescription();

        /**
        * Returns a text description of the task list
        */
        std::string getTaskListDescription();

        /**
        * Closes the hand controller and releases all the resources. Returns true in case of success.
        */
        bool close();


    private:

        bool waitForGraspStabilization(double timeout, double delay);


    };

} // namespace tactileControl

#endif // TACTILECONTROL_HANDCONTROLLER_H