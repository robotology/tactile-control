#ifndef TACTILECONTROL_CONTROLLERUTIL_H
#define TACTILECONTROL_CONTROLLERUTIL_H

#include "TactileControl/data/TaskData.h"

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <vector>

namespace tactileControl {

    class ControllerUtil {

    private:

        int numFingers;
        std::vector<int> storedHandJointsControlMode;
        std::vector<double> storedHandJointsMaxPwmLimits;
        double storedPIDIntegralGain;
        std::vector<double> openHandJoints;
        std::vector<int> armJointControlModes;

        yarp::dev::PolyDriver clientArm;
        yarp::dev::IEncoders *iEncs;
        yarp::dev::IPWMControl *iPwm;
        yarp::dev::IControlMode2 *iCtrl;
        yarp::dev::IPositionControl *iPos;
        yarp::dev::IVelocityControl *iVel;
        yarp::dev::IPositionDirect *iPosDir;
        yarp::dev::IPositionControl2 *iPosCtrl;
        yarp::dev::IPidControl *iPid;

        tactileControl::TaskData *taskData;

        /* ****** Debug attributes                              ****** */
        std::string dbgTag;

    public:

        ControllerUtil();

        bool init(tactileControl::TaskData *taskData);

        bool sendPwm(int joint,double pwm);

        bool sendVelocity(int joint,double velocity);

        bool getArmEncoderAngles(std::vector<double> &armEncoderAngles,bool wait = false);

        bool getArmEncoderAngleReferences(std::vector<double> &armEncoderAngleReferences,bool wait = false);

        bool saveCurrentControlMode();

        bool restorePreviousControlMode();

        bool setControlMode(int joint,int controlMode);

        bool setControlMode(const std::vector<int> &jointsList,int controlMode);

        bool waitMotionDone(double timeout, double delay);

        bool isMotionDone();

        bool getEncoderAngle(int joint,double &encoderData);

        bool openHand(bool fullyOpen,bool wait);

        bool setJointMaxPwmLimit(int joint,double maxPwm);

        bool setJointsMaxPwmLimit(const std::vector<int> &jointsList,const std::vector<double> &maxPwmList);

        bool saveHandJointsMaxPwmLimits();

        bool restoreHandJointsMaxPwmLimits();

        bool resetPIDIntegralGain(int joint);

        bool restorePIDIntegralGain(int joint);

        bool setJointAngle(int joint,double angle);

        bool setJointAnglePositionDirect(int joint,double angle);

        bool release();

    };

} //namespace tactileControl

#endif // TACTILECONTROL_CONTROLLERUTIL_H

