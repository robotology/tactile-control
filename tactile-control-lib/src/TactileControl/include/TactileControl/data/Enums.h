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

}

#endif // TACTILECONTROL_ENUMS_H
