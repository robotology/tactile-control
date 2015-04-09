#ifndef __ICUB_OBJECTGRASPING_CONTROLLERSUTIL_H__
#define __ICUB_OBJECTGRASPING_CONTROLLERSUTIL_H__

#include <iCub/objectGrasping/ObjectGraspingEnums.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

#include <vector>

namespace iCub {
    namespace objectGrasping {

        class ControllersUtil {

            private:

                yarp::dev::PolyDriver clientArm;
                yarp::dev::IEncoders *iEncs;
				yarp::dev::IOpenLoopControl *iOLC;
				yarp::dev::IControlMode2 *iCtrl;
				yarp::dev::IPositionControl *iPos;
				yarp::dev::IVelocityControl *iVel;

				yarp::sig::Vector armStoredPosition;
				int armJointsNum;
				std::vector<int> jointsStoredControlMode;
				std::vector<int> handJointsToMove;

				/* ****** Debug attributes                              ****** */
                std::string dbgTag;

            public:

				ControllersUtil();

				bool init(yarp::os::ResourceFinder &rf);

				bool sendPwm(int joint,double pwm);

				bool saveCurrentArmPosition();

				bool saveCurrentControlMode();

				bool setTaskControlModes(std::vector<int> &jointsList,int controlMode);

				bool setArmInStartPosition();

				bool setArmInGraspPosition();

				bool raiseArm();

				bool restorePreviousArmPosition();

				bool restorePreviousControlMode();

				bool release();

				bool openHand();

			private:

				bool waitMoveDone(const double &i_timeout, const double &i_delay);

				bool setControlMode(int joint,int controlMode,bool checkCurrent);
};
    } //namespace objectGrasping
} //namespace iCub

#endif

