#ifndef TACTILECONTROL_ICUBUTIL_H
#define TACTILECONTROL_ICUBUTIL_H

#include "TactileControl/data/Enums.h"

#include <vector>

//#include "iCub/plantIdentification/PlantIdentificationEnums.h"
//#include "iCub/plantIdentification/data/TaskData.h"
//#include "iCub/plantIdentification/util/ControllersUtil.h"
//#include "iCub/plantIdentification/util/PortsUtil.h"
//
//#include <yarp/os/Bottle.h>
//#include <yarp/os/Value.h>
//#include <yarp/sig/Vector.h>
//#include <yarp/sig/Matrix.h>
//
//
//#include <fstream>

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
 
        //static void printBottleIntoFile(std::ofstream &file,yarp::os::Bottle &bottle);

        //static void normalizeVector(const std::vector<double> &inputVector,std::vector<double> &outputVector);

    private:

        static double getForceBySimpleSum(const std::vector<double>& fingerTaxelsData);

        static double getForceByWeightedSum(const std::vector<double>& fingerTaxelsData);

        static void getUnitVector(int index,std::vector<double>& unitVector);




        //static void addOption(yarp::os::Bottle &bottle,const char *paramName,yarp::os::Value paramValue);

        //static void addOption(yarp::os::Bottle &bottle,const char *paramName,yarp::os::Value paramValue1,yarp::os::Value paramValue2);

        //static void addOption(yarp::os::Bottle &bottle,const char *paramName,yarp::os::Value paramValue1,yarp::os::Value paramValue2,yarp::os::Value paramValue3);

        //static void addOption(yarp::os::Bottle &bottle,const char *paramName,double paramValueList[],int numElem);

        //static void addOption(yarp::os::Bottle &bottle,const char *paramName,yarp::os::Value paramValueList[],int numElem);

    };
} //namespace tactileControl

#endif // TACTILECONTROL_ICUBUTIL_H

