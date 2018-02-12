#ifndef TACTILECONTROL_APPROACHTASK_H
#define TACTILECONTROL_APPROACHTASK_H

#include "TactileControl/task/Task.h"
#include "TactileControl/data/TaskData.h"
#include "TactileControl/util/ControllerUtil.h"
#include "TactileControl/util/PortUtil.h"

#include "iCub/ctrl/adaptWinPolyEstimator.h"

#include <vector>

namespace tactileControl {

    class ApproachTask : public Task {

    private:
            
        std::vector<bool> fingerIsInContact;
        std::vector<bool> fingerSetInPosition;
        int callsNumberForMovementTimeout;
        std::vector<double> jointVelocities;
        std::vector<std::vector<double> > fingerPositions;
        int windowSize;
        double finalCheckThreshold;
        int positionIndex;

        iCub::ctrl::AWLinEstimator *awPolyEst;

    public:

        ApproachTask(tactileControl::TaskData *taskData,tactileControl::ControllerUtil * controllerUtil,tactileControl::PortUtil * portUtil);

        virtual void init();

        virtual void calculateControlInput();

        virtual void sendControlSignal();

        virtual void release();

        virtual bool taskIsOver();

        virtual std::string getTaskDescription();

    private:

        void moveFinger(int finger);

        bool eachFingerIsInContact();

    };

} //namespace tactileControl

#endif // TACTILECONTROL_APPROACHTASK_H

