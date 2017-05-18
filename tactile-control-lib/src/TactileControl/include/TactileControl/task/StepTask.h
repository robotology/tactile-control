#ifndef TACTILECONTROL_STEPTASK_H
#define TACTILECONTROL_STEPTASK_H

#include "TactileControl/task/Task.h"
#include "TactileControl/data/TaskData.h"
#include "TactileControl/util/ControllerUtil.h"
#include "TactileControl/util/PortUtil.h"

#include <string>

namespace tactileControl {

    class StepTask : public Task {

    private:

        std::vector<double> constantPwm;

    public:

        StepTask(tactileControl::TaskData *taskData,tactileControl::ControllerUtil * controllerUtil,tactileControl::PortUtil * portUtil,const std::vector<double> &targets);

        virtual void init();

        virtual std::string getTaskDescription();

        virtual void calculateControlInput();

     };

} //namespace tactileControl

#endif // TACTILECONTROL_STEPTASK_H

