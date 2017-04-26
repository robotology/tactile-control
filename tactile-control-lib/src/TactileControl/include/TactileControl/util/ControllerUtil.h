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

            yarp::dev::PolyDriver clientArm;
            yarp::dev::IEncoders *iEncs;
            yarp::dev::IOpenLoopControl *iOLC;
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

            bool getHandEncoderAngles(std::vector<double> &handEncoderAngles,bool wait = false);

            bool getHandEncoderAngleReferences(std::vector<double> &handEncoderAngleReferences,bool wait = false);

            bool saveCurrentControlMode();

            bool restorePreviousControlMode();

            bool setTaskControlMode(std::vector<int> &jointsList,int controlMode);

            bool waitMotionDone(double timeout, double delay);

            bool getEncoderAngle(int joint,double &encoderData);

            bool openHand(bool fingersAreStraight);

            bool setJointMaxPwmLimit(int joint,double maxPwm);

            bool setJointsMaxPwmLimit(const std::vector<int> &jointsList,const std::vector<double> &maxPwmList);

            bool saveHandJointsMaxPwmLimits();

            bool restoreHandJointsMaxPwmLimits();

            bool resetPIDIntegralGain(double joint);

            bool restorePIDIntegralGain(double joint);

            bool setJointAngle(int joint,double angle);

            bool setJointAnglePositionDirect(int joint,double angle);

            bool release();

    };

} //namespace tactileControl

#endif // TACTILECONTROL_CONTROLLERUTIL_H

