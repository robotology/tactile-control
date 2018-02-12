#ifndef TACTILECONTROL_TASKDATA_H
#define TACTILECONTROL_TASKDATA_H

#include "TactileControl/data/Enums.h"
#include "TactileControl/data/GMMData.h"

#include <yarp/os/Property.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/Value.h>

#include <string>
#include <vector>

namespace tactileControl {

    class TaskData {

    public:

        std::vector<std::vector<double> > fingerTaxelsData;
        std::vector<std::vector<double> > fingerTaxelsRawData;
        std::vector<std::vector<double> > previousOverallFingerForce;
        std::vector<int> previousForceIndex;
        std::vector<double> overallFingerForce;
        std::vector<double> overallFingerForceBySimpleSum;
        std::vector<double> overallFingerForceByWeightedSum;
        std::vector<double> overallFingerForceMedian;

        std::vector<double> fingerEncodersRawData;
        std::vector<double> armEncoderAngles;
        std::vector<double> armEncoderAngleReferences;

        // control task data
        tactileControl::GMMData* gmmDataStandard;
        bool graspIsStable;

        // tactile scores (classifier output)
        std::vector<double> tactileScores;
        std::vector<double> visualScores;

        bool objectRecognitionTaskComplete;

    private:

        /* ****** Debug attributes                              ****** */
        std::string dbgTag;

    public:

        yarp::os::Property *options;

        TaskData();

        bool init(const yarp::os::Property &options);
        bool initEncodersData(int numArmJoints);

        bool set(const yarp::os::ConstString &key, const yarp::os::Value &value, tactileControl::PropertyWritingMode propertyWritingMode = WRITE_ONLY_IF_PRESENT);
        void setDefault(const yarp::os::ConstString &key,const yarp::os::Value &value);
        void setToList(const yarp::os::ConstString &key,const yarp::os::Value &value,int index);
        bool get(const yarp::os::ConstString &key,yarp::os::Value &value);

        yarp::os::Value getValue(const yarp::os::ConstString &key);
        int getInt(const yarp::os::ConstString &key);
        double getDouble(const yarp::os::ConstString &key);
        std::string getString(const yarp::os::ConstString &key);
        bool getBool(const yarp::os::ConstString &key);

        yarp::os::Value getValueFromList(const yarp::os::ConstString &key,int index);
        int getIntFromList(const yarp::os::ConstString &key,int index);
        double getDoubleFromList(const yarp::os::ConstString &key,int index);
        std::string getStringFromList(const yarp::os::ConstString &key,int index);
        bool getBoolFromList(const yarp::os::ConstString &key,int index);

        void getList(const yarp::os::ConstString &key,std::vector<yarp::os::Value> &list);
        void getList(const yarp::os::ConstString &key,std::vector<int> &list);
        void getList(const yarp::os::ConstString &key,std::vector<double> &list);
        void getList(const yarp::os::ConstString &key,std::vector<std::string> &list);
        void getList(const yarp::os::ConstString &key,std::vector<bool> &list);

        std::string getDataDescription();

        // parameters not directly specified
        void getControlledFingers(std::vector<int> &controlledFingers);
        int getFingerNum();
        tactileControl::SupervisorMode getSupervisorControlMode();

        void release();

    private:

        std::string getParameterDescription(const yarp::os::ConstString &key);

    };

} //namespace tactileControl

#endif // TACTILECONTROL_TASKDATA_H

