#ifndef TACTILECONTROL_ENUMS_H
#define TACTILECONTROL_ENUMS_H

namespace tactileControl {

    enum ForceCalculationMode {

        SIMPLE_SUM,
        WEIGHTED_SUM
    };

    enum TaskName {

        STEP,
        APPROACH,
        CONTROL,
        RAMP,
        NONE
    };

    enum SupervisorMode {

        FIXED_OBJECT_POS_MODE = 0,
        HAND_FREEZE_MODE = 1,
        GMM_MODE = 2
    }

    enum GMMType {

        STANDARD_GMM
    }

}

#endif // TACTILECONTROL_ENUMS_H
