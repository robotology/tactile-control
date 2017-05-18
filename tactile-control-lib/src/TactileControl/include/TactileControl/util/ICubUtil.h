#ifndef TACTILECONTROL_ICUBUTIL_H
#define TACTILECONTROL_ICUBUTIL_H

#include "TactileControl/data/Enums.h"

#include <vector>

namespace tactileControl {

    class ICubUtil {

    public:

        static int getFingerFromJoint(int joint);

        static bool isDistal(int joint);

        static bool isThumb(int joint);

        static bool isIndexFinger(int joint);

        static bool isMiddleFinger(int joint);

        static bool isRingOrLittleFinger(int joint);

        static double getGripStrength(int numFingers,const std::vector<double> &overallFingerForce);

        static double getObjectPosition(int numFingers,const std::vector<double> &armEncoderAngles);

        static double getHandAperture(int numFingers,const std::vector<double> &armEncoderAngles);

        static double getIndexMiddleDifference(int numFingers,const std::vector<double> &armEncoderAngles);

        static double getForce(const std::vector<double>& fingerTaxelsData,tactileControl::ForceCalculationMode forceCalculationMode);

    private:

        static double getForceBySimpleSum(const std::vector<double>& fingerTaxelsData);

        static double getForceByWeightedSum(const std::vector<double>& fingerTaxelsData);

        static void getUnitVector(int index,std::vector<double>& unitVector);

    };

} //namespace tactileControl

#endif // TACTILECONTROL_ICUBUTIL_H

