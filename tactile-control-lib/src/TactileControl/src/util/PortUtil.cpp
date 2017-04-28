#include "TactileControl/util/PortUtil.h"

#include "TactileControl/data/Parameters.h"

#include <yarp/os/Network.h>

using tactileControl::PortUtil;
using std::string;


PortUtil::PortUtil(){

    dbgTag = "PortUtil: ";
}

bool PortUtil::init(tactileControl::TaskData *taskData){
    using std::cout;
    using yarp::os::Network;

    string portPrefix = taskData->getString(PAR_COMMON_PORT_PREFIX);
    string iCub = taskData->getString(PAR_COMMON_ICUB);
    string hand = taskData->getString(PAR_COMMON_HAND);

    string portFullPrefix;
    if (taskData->getBool(PAR_COMMON_USING_TWO_HANDS)){
        portFullPrefix = "/" + portPrefix + "/" + hand + "_hand";
    } else {
        portFullPrefix = "/" + portPrefix;
    }

    // icub ports
    string icubSkinRawPortName = "/icub/skin/" + hand + "_hand";
    string icubSkinCompPortName = "/icub/skin/" + hand + "_hand_comp";
    string icubHandEncodersRawPortName = "/icub/" + hand + "_hand/analog:o";

    // input ports
    string moduleSkinRawPortName = portFullPrefix + "/tactile_raw:i";
    string moduleSkinCompPortName = portFullPrefix + "/tactile_comp:i";
    string moduleHandEncodersRawPortName = portFullPrefix + "/encoders_raw:i";

    // output ports
    string infoDataPortName = portFullPrefix + "/info";
    string controlDataPortName = portFullPrefix + "/control";
    string gmmDataPortName = portFullPrefix + "/gmm:o";
    string gmmRegressionDataPortName = portFullPrefix + "/gmmRegression:o";
    string gripStrengthDataPortName = portFullPrefix + "/grip_strength:o";

    // opening ports
    if (!portSkinRawIn.open(moduleSkinRawPortName)){
        cout << dbgTag << "could not open " << moduleSkinRawPortName << " port \n";
        return false;
    }
    if (!portSkinCompIn.open(moduleSkinCompPortName)){
        cout << dbgTag << "could not open " << moduleSkinCompPortName << " port \n";
        return false;
    }
    if (!portHandEncodersRawIn.open(moduleHandEncodersRawPortName)){
        cout << dbgTag << "could not open " << moduleHandEncodersRawPortName << " port \n";
        return false;
    }
    if (!portInfoDataOut.open(infoDataPortName)){
        cout << dbgTag << "could not open " << infoDataPortName << " port \n";
        return false;
    }
    if (!portControlDataOut.open(controlDataPortName)){
        cout << dbgTag << "could not open " << controlDataPortName << " port \n";
        return false;
    }
    if (!portGMMDataOut.open(gmmDataPortName)){
        cout << dbgTag << "could not open " << gmmDataPortName << " port \n";
        return false;
    }
    if (!portGMMRegressionDataOut.open(gmmRegressionDataPortName)){
        cout << dbgTag << "could not open " << gmmRegressionDataPortName << " port \n";
        return false;
    }
    if (!portGripStrengthDataOut.open(gripStrengthDataPortName)){
        cout << dbgTag << "could not open " << gripStrengthDataPortName << " port \n";
        return false;
    }

    // connecting ports
    if (!Network::connect(icubSkinRawPortName,moduleSkinRawPortName)){
        cout << dbgTag << "could not connect port " << icubSkinRawPortName << " to " <<  moduleSkinRawPortName << "\n";
        return false;
    }
    if (!Network::connect(icubSkinCompPortName,moduleSkinCompPortName)){
        cout << dbgTag << "could not connect port " << icubSkinCompPortName << " to " <<  moduleSkinCompPortName << "\n";
        return false;
    }
    if (!Network::connect(icubHandEncodersRawPortName,moduleHandEncodersRawPortName)){
        cout << dbgTag << "could not connect port " << icubHandEncodersRawPortName << " to " <<  moduleHandEncodersRawPortName << "\n";
        return false;
    }

    return true;
}


bool PortUtil::sendInfoData(tactileControl::TaskData *taskData){

    using yarp::os::Bottle;

    Bottle& infoBottle = portInfoDataOut.prepare();
    infoBottle.clear();

    for(size_t i = 0; i < taskData->overallFingerForceByWeightedSum.size(); i++){
        infoBottle.add(taskData->overallFingerForceByWeightedSum[i]);
    }
    for(size_t i = 0; i < taskData->overallFingerForceBySimpleSum.size(); i++){
        infoBottle.add(taskData->overallFingerForceBySimpleSum[i]);
    }

    portInfoDataOut.write();

    return true;
}

bool PortUtil::sendControlData(string taskId,string experimentDescription,string previousExperimentDescription,double targetGripStrength,double actualGripStrength,double u,double error,double svCurrentPosition,double actualCurrentTargetPose,double finalTargetPose,double estimatedFinalPose,double svKp,double svKi,double svKd,double thumbEnc,double indexEnc,double middleEnc,double enc8,const std::vector<double> &pressureTarget,const std::vector<double> &actualPressure,const std::vector<double> &pwm,const std::vector<int> &fingersList){

    using yarp::os::Bottle;

    Bottle& ctrlBottle = portControlDataOut.prepare();
    ctrlBottle.clear();

    double fingerJoint;

    ctrlBottle.addString(taskId);// index 1
    ctrlBottle.addString(experimentDescription);// index 2
    ctrlBottle.addString(previousExperimentDescription);// index 3
    ctrlBottle.addInt(pressureTarget.size());// index 4
    ctrlBottle.addDouble(targetGripStrength);// index 5
    ctrlBottle.addDouble(actualGripStrength);// index 6
    ctrlBottle.addDouble(u);// index 7
    ctrlBottle.addDouble(error);// index 8
    ctrlBottle.addDouble(actualCurrentTargetPose);// index 9
    ctrlBottle.addDouble(finalTargetPose);// index 10
    ctrlBottle.addDouble(svCurrentPosition);// index 11
    ctrlBottle.addDouble(estimatedFinalPose);// index 12
    ctrlBottle.addDouble(svKp);// index 13
    ctrlBottle.addDouble(svKi);// index 14
    ctrlBottle.addDouble(svKd);// index 15
    ctrlBottle.addDouble(thumbEnc);// index 16
    ctrlBottle.addDouble(indexEnc);// index 17
    ctrlBottle.addDouble(middleEnc);// index 18
    ctrlBottle.addDouble(enc8);// index 19
    for(int i = 0; i < pressureTarget.size(); i++){
        
        ctrlBottle.addInt(fingersList[i]);// index 20 ... 24 ...
        ctrlBottle.addDouble(pwm[i]);// index 21 ... 25 ...
        ctrlBottle.addDouble(pressureTarget[i]);// index 22 ... 26 ...
        ctrlBottle.addDouble(actualPressure[fingersList[i]]);// index 23 ... 27...
    }

    portControlDataOut.write();

    return true;
}


bool PortUtil::sendGripStrengthData(std::string experimentDescription,std::string previousExperimentDescription,double targetGripStrength,double actualGripStrength,tactileControl::TaskData *taskData){

    using yarp::os::Bottle;

    Bottle& gripStrengthBottle = portGripStrengthDataOut.prepare();
    gripStrengthBottle.clear();

    gripStrengthBottle.addString(experimentDescription);// index 1
    gripStrengthBottle.addString(previousExperimentDescription);// index 2
    gripStrengthBottle.addDouble(targetGripStrength);// index 3
    gripStrengthBottle.addDouble(actualGripStrength);// index 4

    // fingers overall pressure (5 values) (indexes  5-9)
    for(int i = 0; i < taskData->overallFingerForce.size(); i++){
        gripStrengthBottle.addDouble(taskData->overallFingerForce[i]);
    }

    // compensated taxels feedback (60 values) (indexes  10-69)
    for(int i = 0; i < taskData->fingerTaxelsData.size(); i++){
        for(int j = 0; j < taskData->fingerTaxelsData[i].size(); j++){
            gripStrengthBottle.addDouble(taskData->fingerTaxelsData[i][j]);
        }
    }

    portGripStrengthDataOut.write();

    return true;
}

bool PortUtil::sendGMMData(double gripStrength,double indexMiddleFingerPressureBalance,tactileControl::TaskData *taskData){

    using yarp::os::Bottle;

    Bottle& objGMMBottle = portGMMDataOut.prepare();
    objGMMBottle.clear();

    // grip strength (1 value, index 1)
    objGMMBottle.addDouble(gripStrength);

    // index / middle finger pressure balance (1 value, index 2)
    objGMMBottle.addDouble(indexMiddleFingerPressureBalance);

    // compensated taxels feedback (60 values, indexes 3-62)
    for(int i = 0; i < taskData->fingerTaxelsData.size(); i++){
        for(int j = 0; j < taskData->fingerTaxelsData[i].size(); j++){
            objGMMBottle.addDouble(taskData->fingerTaxelsData[i][j]);
        }
    }

    // fingers overall pressure (5 values, indexes 63-67)
    for(int i = 0; i < taskData->overallFingerForceByWeightedSum.size(); i++){
        objGMMBottle.addDouble(taskData->overallFingerForceByWeightedSum[i]);
    }

    // arm encoders (16 values) (indexes 68-83)
    for(int i = 0; i < taskData->armEncodersAngles.size(); i++){
        objGMMBottle.addDouble(taskData->armEncodersAnglesReferences[i]);
    }

    // raw taxels feedback (60 values) (indexes 84-143)
    for(int i = 0; i < taskData->fingerTaxelsRawData.size(); i++){
        for(int j = 0; j < taskData->fingerTaxelsRawData[i].size(); j++){
            objGMMBottle.addDouble(taskData->fingerTaxelsRawData[i][j]);
        }
    }

    portGMMDataOut.write();

    return true;

}

bool PortUtil::sendGMMRegressionData(double handAperture,double indMidPosDiff,double targetHandPosition,double actualHandPosition,double filteredHandPosition,double targetThumbDistalJoint,double filteredThumbDistalJoint,double targetIndexDistalJoint,double filteredIndexDistalJoint,double targetMiddleDistalJoint,double filteredMiddleDistalJoint,double targetThumbAbductionJoint,double filteredThumbAbductionJoint, double targetIndMidForceBalance, double actualIndMidForceBalance,double targetGripStrength,double actualGripStrength,tactileControl::TaskData *taskData){

    using yarp::os::Bottle;

    Bottle& objGMMRegressionBottle = portGMMRegressionDataOut.prepare();
    objGMMRegressionBottle.clear();

    // experiment description (index 1)
    objGMMRegressionBottle.addString(taskData->getString(PAR_COMMON_EXPERIMENT_INFO));
    // previous experiment description (index 2)
    objGMMRegressionBottle.addString(taskData->getString(PAR_COMMON_EXPERIMENT_OPTIONAL_INFO));

    // hand aperture (query variable) (index 3)
    objGMMRegressionBottle.addDouble(handAperture);

    // index/middle finger position difference (query variable) (index 4)
    objGMMRegressionBottle.addDouble(indMidPosDiff);

    // target hand position (output variable) (index 5)
    objGMMRegressionBottle.addDouble(targetHandPosition);
    // filtered hand position (index 6)
    objGMMRegressionBottle.addDouble(filteredHandPosition);
    // actual hand position (index 7)
    objGMMRegressionBottle.addDouble(actualHandPosition);

    // target thumb distal joint (output variable) (index 8)
    objGMMRegressionBottle.addDouble(targetThumbDistalJoint);
    // filtered thumb distal joint (index 9)
    objGMMRegressionBottle.addDouble(filteredThumbDistalJoint);
    // actual thumb distal joint (index 10)
    objGMMRegressionBottle.addDouble(taskData->armEncoderAngles[THUMB_DISTAL_JOINT]);

    // target index distal joint (output variable) (index 11)
    objGMMRegressionBottle.addDouble(targetIndexDistalJoint);
    // filtered index distal joint (index 12)
    objGMMRegressionBottle.addDouble(filteredIndexDistalJoint);
    // actual index distal joint (index 13)
    objGMMRegressionBottle.addDouble(taskData->armEncoderAngles[INDEX_DISTAL_JOINT]);

    // target middle distal joint (output variable) (index 14)
    objGMMRegressionBottle.addDouble(targetMiddleDistalJoint);
    // filtered middle distal joint (index 15)
    objGMMRegressionBottle.addDouble(filteredMiddleDistalJoint);
    // actual middle distal joint (index 16)
    objGMMRegressionBottle.addDouble(taskData->armEncoderAngles[MIDDLE_DISTAL_JOINT]);

    // target thumb abduction joint (output variable) (index 17)
    objGMMRegressionBottle.addDouble(targetThumbAbductionJoint);
    // filtered thumb abduction joint (index 18)
    objGMMRegressionBottle.addDouble(filteredThumbAbductionJoint);
    // actual thumb abduction joint (index 19)
    objGMMRegressionBottle.addDouble(taskData->armEncoderAngles[THUMB_ABDUCTION_JOINT]);

    // target index/middle force balance (output variable) (index 20)
    objGMMRegressionBottle.addDouble(targetIndMidForceBalance);
    // actual index/middle force balance (index 21)
    objGMMRegressionBottle.addDouble(actualIndMidForceBalance);

    // target grip strength (output variable) (index 22)
    objGMMRegressionBottle.addDouble(targetGripStrength);
    // actual grip strength (index 23)
    objGMMRegressionBottle.addDouble(actualGripStrength);

    // arm encoders (16 values, indexes 24-39)
    for(int i = 0; i < taskData->armEncoderAngles.size(); i++){
        objGMMRegressionBottle.addDouble(taskData->armEncoderAngles[i]);
    }

    // fingers overall pressure (5 values, indexes 40-44)
    for(int i = 0; i < taskData->overallFingerForceByWeightedSum.size(); i++){
        objGMMRegressionBottle.addDouble(taskData->overallFingerForceByWeightedSum[i]);
    }

    // compensated taxels feedback (60 values, indexes 45-104)
    for(int i = 0; i < taskData->fingerTaxelsData.size(); i++){
        for(size_t j = 0; j < taskData->fingerTaxelsData[i].size(); j++){
            objGMMRegressionBottle.addDouble(taskData->fingerTaxelsData[i][j]);
        }
    }
    
    // raw taxels feedback [60] (105-164)
    for(int i = 0; i < taskData->fingerTaxelsRawData.size(); i++){
        for(int j = 0; j < taskData->fingerTaxelsRawData[i].size(); j++){
            objGMMRegressionBottle.addDouble(taskData->fingerTaxelsRawData[i][j]);
        }
    }

    portGMMRegressionDataOut.write();

    return true;

}


bool PortUtil::readFingerSkinRawData(std::vector<std::vector<double> > &fingerTaxelsRawData){

    yarp::sig::Vector *iCubSkinData = portSkinRawIn.read(false);
    
    if (iCubSkinData) {
        for(int i = 0; i < NUM_FINGERS; i++){
            for (int j = 0; j < fingerTaxelsRawData[i].size(); j++){
                fingerTaxelsRawData[i][j] = (*iCubSkinData)[12*i + j];
            }
        }
    } else {
        return false;
    }

    return true;
}

bool PortUtil::readFingerSkinCompData(std::vector<std::vector<double> > &fingerTaxelsData,const std::vector<double> &fingersSensitivityScale){
    
    yarp::sig::Vector *iCubSkinData = portSkinCompIn.read(false);

    if (iCubSkinData) {
        for(size_t i = 0; i < NUM_FINGERS; i++){
            for (size_t j = 0; j < fingerTaxelsData[i].size(); j++){
                fingerTaxelsData[i][j] = fingersSensitivityScale[i] * (*iCubSkinData)[12*i + j];
            }
        }
    } else {
        return false;
    }

    return true;
}

bool PortUtil::readFingerEncodersRawData(std::vector<double> &fingerEncodersRawData){

    using yarp::sig::Vector;

    Vector *iCubEncRawData = portHandEncodersRawIn.read(false);
    
    if (iCubEncRawData) {
        for (size_t i = 0; i < fingerEncodersRawData.size(); i++){
            fingerEncodersRawData[i] = (*iCubEncRawData)[i];
        }
    } else {
        return false;
    }

    return true;
}


bool PortUtil::release(){

    portSkinRawIn.interrupt();
    portSkinCompIn.interrupt();
    portHandEncodersRawIn.interrupt();
    portInfoDataOut.interrupt();
    portControlDataOut.interrupt();
    portGMMDataOut.interrupt();
    portGMMRegressionDataOut.interrupt();
    portGripStrengthDataOut.interrupt();

    portSkinRawIn.close();
    portSkinCompIn.close();
    portHandEncodersRawIn.close();
    portInfoDataOut.close();
    portControlDataOut.close();
    portGMMDataOut.close();
    portGMMRegressionDataOut.close();
    portGripStrengthDataOut.close();

    return true;
}