#ifndef TACTILECONTROL_CONTROLTASK_H
#define TACTILECONTROL_CONTROLTASK_H

#include "TactileControl/task/Task.h"
#include "TactileControl/data/TaskData.h"
#include "TactileControl/data/Enums.h"
#include "TactileControl/util/ControllerUtil.h"
#include "TactileControl/util/PortUtil.h"
#include "TactileControl/util/MLUtil.h"

#include <iCub/ctrl/pids.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>


#include <string>

namespace tactileControl {

    class ControlTask : public Task {

    private:

        std::vector<iCub::ctrl::parallelPID*> pid;
        std::vector<double> forceTargetValue;
        int numFingers;

        tactileControl::MLUtil *mlUtil;

        // variables used for supervisor mode
        iCub::ctrl::parallelPID *highPid;
        bool supervisorEnabled;
        tactileControl::SupervisorMode supervisorControlMode;
        double highPidKp,highPidKi,highPidKd;
        bool minJerkTrackingModeInitialized;
        bool gmmJointsMinJerkTrackingModeInitialized;
        iCub::ctrl::minJerkTrajGen* minJerkTrajectory;
        iCub::ctrl::minJerkTrajGen* thAbdMinJerkTrajectory;
        iCub::ctrl::minJerkTrajGen* thDistMinJerkTrajectory;
        iCub::ctrl::minJerkTrajGen* indDistMinJerkTrajectory;
        iCub::ctrl::minJerkTrajGen* midDistMinJerkTrajectory;
        bool disablePIDIntegralGain;
        bool gmmJointsRegressionEnabled;
        bool gmmCtrlModeIsSet;
        double initialObjectPosition;

        // variables used for object recognition task
        bool objectRecognitionTaskEnabled;
        double startSqueezingTime;
        double startTactileDataCollectionTime;
        double startBendingProximalJointsTime;
        double startBendingDistalJointsTime;
        double taskCompleteTime;

        double gripStrengthIncreased;
        double featuresProcessingComplete;
        bool proximalJointsSetInOLMode;
        bool distalJointsSetInOLMode;
        std::vector<double> averageTactileData;
        std::vector<double> averageTactileDataNormalized;
        std::vector<double> objRecFeatures;
        std::vector<double> multimodalFeatures;

    public:

        ControlTask(tactileControl::TaskData *taskData, tactileControl::ControllerUtil * controllerUtil, tactileControl::PortUtil * portUtil,tactileControl::MLUtil * mlUtil);
        ControlTask(tactileControl::TaskData *taskData, tactileControl::ControllerUtil * controllerUtil, tactileControl::PortUtil * portUtil, tactileControl::MLUtil * mlUtil, const std::vector<double> &targets);

        virtual void init();

        virtual void calculateControlInput();

        virtual std::string getTaskDescription();

        virtual void release();

    private:

        void constructorsCommon(const std::vector<double> &targets,tactileControl::MLUtil *mlUtil);

        void initLowLevelPID();

        void initHighLevelPID();

        void initPID(iCub::ctrl::parallelPID *&pid,double kp,double ki,double kd,double wp,double wi,double wd,double n,double tt,double minSatLim,double maxSatLim);

        void addOption(yarp::os::Bottle &bottle,const char *paramName,const yarp::os::Value paramValue);

        void addOption(yarp::os::Bottle &bottle,const char *paramName,const yarp::os::Value paramValue1,const yarp::os::Value paramValue2);

        void setOptionVect(const std::vector<double> &option,std::vector<yarp::sig::Vector> &optionVect);

        double calculateTt(double kp,double ki,double kd,double windUp);

        void setGMMJointsControlMode(int controlMode);

        void manageGMMRegression(double handAperture,double indMidPosDiff,double &targetObjectPosition,double &gmmThumbAbductionJoint,double &gmmThumbDistalJoint,double &gmmIndexDistalJoint,double &gmmMiddleDistalJoint,double &filteredThumbAbductionJoint,double &filteredThumbDistalJoint,double &filteredIndexDistalJoint,double &filteredMiddleDistalJoint);

        void computeForceTargetValues(double gripStrength,double svControlSignal,bool minForceEnabled,double minForce,std::vector<double> &forceTargetValue);

        bool manageObjectRecognitionTask();

        tactileControl::ObjectRecognitionTaskState getObjectRecognitionTaskState();
    };

} //namespace tactileControl

#endif // TACTILECONTROL_CONTROLTASK_H

