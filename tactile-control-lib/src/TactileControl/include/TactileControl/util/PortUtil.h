#ifndef TACTILECONTROL_PORTUTIL_H
#define TACTILECONTROL_PORTUTIL_H

#include "TactileControl/data/TaskData.h"

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <string>

#include <vector>

namespace tactileControl {

    class PortUtil {

        private:

            /* ******* Ports                            ******* */
            yarp::os::BufferedPort<yarp::sig::Vector> portSkinRawIn;
            yarp::os::BufferedPort<yarp::sig::Vector> portSkinCompIn;
            yarp::os::BufferedPort<yarp::sig::Vector> portHandEncodersRawIn;

            yarp::os::BufferedPort<yarp::os::Bottle> portInfoDataOut;
            yarp::os::BufferedPort<yarp::os::Bottle> portControlDataOut;
            yarp::os::BufferedPort<yarp::os::Bottle> portGMMDataOut;
            yarp::os::BufferedPort<yarp::os::Bottle> portGMMRegressionDataOut;
            yarp::os::BufferedPort<yarp::os::Bottle> portGripStrengthDataOut;

            /* ******* Debug attributes.                ******* */
            std::string dbgTag;

        public:

            PortUtil();

            bool init(tactileControl::TaskData *taskData);

            bool sendInfoData(tactileControl::TaskData *commonData);

            bool sendControlData(std::string taskId,std::string experimentDescription,std::string previousExperimentDescription,double targetGripStrength,double actualGripStrength,double u,double error,double svCurrentPosition,double actualCurrentTargetPose,double finalTargetPose,double estimatedFinalPose,double svKp,double svKi,double svKd,double thumbEnc,double indexEnc,double middleEnc,double enc8,const std::vector<double> &pressureTarget,const std::vector<double> &actualPressure,const std::vector<double> &pwm,const std::vector<int> &fingersList);

            bool sendGMMData(double gripStrength,double indexMiddleFingerPressureBalance,tactileControl::TaskData *taskData);

            bool sendGMMRegressionData(double handAperture,double indMidPosDiff,double targetHandPosition,double actualHandPosition,double filteredHandPosition,double targetThumbDistalJoint,double filteredThumbDistalJoint,double targetIndexDistalJoint,double filteredIndexDistalJoint,double targetMiddleDistalJoint,double filteredMiddleDistalJoint,double targetThumbAbductionJoint,double filteredThumbAbductionJoint, double targetIndMidForceBalance, double actualIndMidForceBalance,double targetGripStrength,double actualGripStrength,tactileControl::TaskData *taskData);

            bool sendGripStrengthData(std::string experimentDescription,std::string previousExperimentDescription,double targetGripStrength,double actualGripStrength,tactileControl::TaskData *taskData);

            bool readFingerSkinRawData(std::vector<std::vector<double> > &fingerTaxelsRawData,const std::vector<double> &fingersSensitivityScale);

            bool readFingerSkinCompData(std::vector<std::vector<double> > &fingerTaxelsData,const std::vector<double> &fingersSensitivityScale);

            bool readFingerEncodersRawData(std::vector<double> &fingerEncodersRawData);

            bool release();

    };

} //namespace tactileControl

#endif // TACTILECONTROL_PORTUTIL_H

