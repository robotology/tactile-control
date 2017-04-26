#ifndef TACTILECONTROL_HANDCONTROLLER_H
#define TACTILECONTROL_HANDCONTROLLER_H

#include "TactileControl/data/TaskData.h"
#include "TactileControl/util/ControllerUtil.h"
#include "TactileControl/util/PortUtil.h"

#include <yarp/os/Property.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/Value.h>

namespace tactileControl {

/**
* \class TactileControl::TactileControl
* \headerfile template-lib.h <TemplateLib/templatelib.h>
*
* \brief A class from TactileControl namespace.
*
* This class that does a summation.
*/

class HandController {

public:

    tactileControl::TaskData *taskData;

    tactileControl::ControllerUtil *controllerUtil;

    tactileControl::PortUtil *portUtil;

    /* ****** Debug attributes                              ****** */
    std::string dbgTag;

public:
    /**
    * Constructor
    */
    HandController();

    /**
    * Destructory
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
    * Sets the given properties.
    */
    void set(const yarp::os::Property &options);

    /**
    * Sets the given property.
    */
    void set(const yarp::os::ConstString key, yarp::os::Value value);

    /**
    * Starts the grasping task (including the approach phase). If wait == true it waits for the grasp to be stable, otherwise it returns immediately. Returns true in case of success.
    */
    bool closeHand(bool wait = true);

    /**
    * Checks if the grasp is stable.
    */
    bool isHandClose();

    /**
    * Opens the hand. If wait == true it waits for the hand to be open, otherwise it returns immediately. Returns true in case of success.
    */
    bool openHand(bool wide, bool wait = true);

    /**
    * Checks if the hand is open.
    */
    bool isHandOpen();

    /**
    * Sets the minimum value of force at each fingertip used for grasping.
    */
    void setMinForce(double minForce);

    /**
    * Sets the grip strength.
    */
    void setGripStrength(double gripStrength);

    /**
    * Closes the hand controller and releases all the resources. Returns true in case of success.
    */
    bool close();

};


///**
//* \class TactileControl::anotherClass
//* \headerfile template-lib.h <TemplateLib/templatelib.h>
//*
//* \brief A derived class from TactileControl namespace.
//*
//* This class performs a difference.
//*/
//class differenceClass : public summationClass
//{
//public:
//    /**
//    * Constructor
//    */
//    differenceClass();
//
//    /**
//    * Destructory
//    */
//    virtual ~differenceClass();
//
//    /**
//    * A method that does something
//    */
//    virtual double doSomething(double op1, double op2);
//};


} // namespace tactileControl

#endif // TACTILECONTROL_HANDCONTROLLER_H
