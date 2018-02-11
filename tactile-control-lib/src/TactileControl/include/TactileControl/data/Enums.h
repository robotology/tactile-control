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
    };

    enum GMMType {

        STANDARD_GMM
    };

    enum PropertyWritingMode {

        ALWAYS_WRITE, // it always writes the property, even if it is already present
        WRITE_ONLY_IF_NOT_PRESENT, // it writes the property only if the property is not present (it sets a default, used during the initialization)
        WRITE_ONLY_IF_PRESENT // it writes the property only if the property is already present (it writes only if the key is legal, used after the initialization)
    };

    enum ObjectRecognitionTaskState {

        GRASP_STABILIZATION,
        OBJECT_SQUEEZING,
        TACTILE_DATA_COLLECTION,
        BEDING_PROXIMAL_JOINTS,
        BENDING_DISTAL_JOINTS,
        TASK_COMPLETE

    };

    enum ClassifierType {

        TACTILE_CLASSIFIER = 0,
        MULTIMODAL_CLASSIFIER = 1
    };
}

#endif // TACTILECONTROL_ENUMS_H
