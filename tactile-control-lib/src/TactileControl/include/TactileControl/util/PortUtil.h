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
        yarp::os::BufferedPort<yarp::os::Bottle> portVisualScoresIn;

        yarp::os::BufferedPort<yarp::os::Bottle> portInfoDataOut;
        yarp::os::BufferedPort<yarp::os::Bottle> portControlDataOut;
        yarp::os::BufferedPort<yarp::os::Bottle> portGMMDataOut;
        yarp::os::BufferedPort<yarp::os::Bottle> portGMMRegressionDataOut;
        yarp::os::BufferedPort<yarp::os::Bottle> portGripStrengthDataOut;
        yarp::os::BufferedPort<yarp::os::Bottle> portSpeakerOut;

        /* ******* Debug attributes.                ******* */
        std::string dbgTag;

    public:

        PortUtil();

        bool init(tactileControl::TaskData *taskData);

        bool sendInfoData(tactileControl::TaskData *taskData);

        bool sendControlData(std::string taskId,std::string experimentInfo,std::string experimentOptionalInfo,double targetGripStrength,double actualGripStrength,double u,double error,double svCurrentPosition,double currentTargetObjectPosition,double targetObjectPosition,const std::vector<double> &forceTargetValue,const std::vector<double> &pwm,const std::vector<int> &controlledFingers,tactileControl::TaskData *taskData);

        bool sendGMMData(double gripStrength,tactileControl::TaskData *taskData);

        bool sendGMMRegressionData(double handAperture,double indMidPosDiff,double targetObjectPosition,double objectPosition,double currentTargetObjectPosition,double gmmThumbDistalJoint,double filteredThumbDistalJoint,double gmmIndexDistalJoint,double filteredIndexDistalJoint,double gmmMiddleDistalJoint,double filteredMiddleDistalJoint,double gmmThumbAbductionJoint,double filteredThumbAbductionJoint,double targetGripStrength,double actualGripStrength,tactileControl::TaskData *taskData);

        bool sendGripStrengthData(std::string experimentInfo,std::string experimentOptionalInfo,double targetGripStrength,double actualGripStrength,tactileControl::TaskData *taskData);

        bool sendStringToSpeaker(std::string objectLabel);

        bool readFingerSkinRawData(std::vector<std::vector<double> > &fingerTaxelsRawData);

        bool readFingerSkinCompData(std::vector<std::vector<double> > &fingerTaxelsData,const std::vector<double> &fingersSensitivityScale);

        bool readFingerEncodersRawData(std::vector<double> &fingerEncodersRawData);

        bool readVisualClassifierAvgScores(std::vector<double> &visualScores);

        bool release();

    };

} //namespace tactileControl

#endif // TACTILECONTROL_PORTUTIL_H

