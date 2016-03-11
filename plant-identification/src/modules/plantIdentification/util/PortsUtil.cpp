#include "iCub/plantIdentification/util/PortsUtil.h"

#include <yarp/os/Network.h>
#include <yarp/os/Value.h>

using std::string;

using yarp::os::Value;

using iCub::plantIdentification::PortsUtil;
using iCub::plantIdentification::LogData;

PortsUtil::PortsUtil(){

	dbgTag = "PortsUtil: ";
}

bool PortsUtil::init(yarp::os::ResourceFinder &rf){
	using yarp::os::Network;
    using std::cout;

	

	string whichHand = rf.check("whichHand", Value("right")).asString().c_str();
    string moduleSkinRawPortName = "/plantIdentification/skin/" + whichHand + "_hand_raw:i";
    string moduleSkinCompPortName = "/plantIdentification/skin/" + whichHand + "_hand_comp:i";
    string moduleHandEncodersRawPortName = "/plantIdentification/encoders/" + whichHand + "_hand_raw:i";
    string policyActionsPortName = "/plantIdentification/policyActions:i";
    string icubSkinRawPortName = "/icub/skin/" + whichHand + "_hand";
    string icubSkinCompPortName = "/icub/skin/" + whichHand + "_hand_comp";
    string icubHandEncodersRawPortName = "/icub/" + whichHand + "_hand/analog:o";
    string logDataPortName = "/plantIdentification/log:o";
    string infoDataPortName = "/plantIdentification/info";
    string controlDataPortName = "/plantIdentification/control";
    string objectRecognitionDataPortName = "/plantIdentification/object_recognition_log:o";

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
	if (!portPolicyActionsIn.open(policyActionsPortName)){
        cout << dbgTag << "could not open " << policyActionsPortName << " port \n";
        return false;
    }
	if (!portLogDataOut.open(logDataPortName)){
        cout << dbgTag << "could not open " << logDataPortName << " port \n";
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
	if (!portObjRecognDataOut.open(objectRecognitionDataPortName)){
        cout << dbgTag << "could not open " << objectRecognitionDataPortName << " port \n";
        return false;
    }

	// connecting ports
	if (!Network::connect(icubSkinRawPortName,moduleSkinRawPortName)){
        cout << dbgTag << "could not connect ports: " << icubSkinRawPortName << " -> " <<  moduleSkinRawPortName << "\n";
        return false;
    }
	if (!Network::connect(icubSkinCompPortName,moduleSkinCompPortName)){
        cout << dbgTag << "could not connect ports: " << icubSkinCompPortName << " -> " <<  moduleSkinCompPortName << "\n";
        return false;
    }
	if (!Network::connect(icubHandEncodersRawPortName,moduleHandEncodersRawPortName)){
        cout << dbgTag << "could not connect ports: " << icubHandEncodersRawPortName << " -> " <<  moduleHandEncodersRawPortName << "\n";
        return false;
    }


	return true;
}

bool PortsUtil::sendLogData(LogData &logData){

	using yarp::os::Bottle;

	Bottle& logBottle = portLogDataOut.prepare();
	logBottle.clear();

	logData.toBottle(logBottle);

	portLogDataOut.write();

	return true;
}

bool PortsUtil::sendInfoData(iCub::plantIdentification::TaskCommonData *commonData){

	using yarp::os::Bottle;

	Bottle& infoBottle = portInfoDataOut.prepare();
	infoBottle.clear();

	for(size_t i; i < commonData->overallFingerPressureByWeightedSum.size(); i++){
		infoBottle.add(commonData->overallFingerPressureByWeightedSum[i]);
	}
	for(size_t i; i < commonData->overallFingerPressureBySimpleSum.size(); i++){
		infoBottle.add(commonData->overallFingerPressureBySimpleSum[i]);
	}

	portInfoDataOut.write();

	return true;
}

bool PortsUtil::sendControlData(string taskId,string experimentDescription,string previousExperimentDescription,double targetGripStrength,double actualGripStrength,double u,double error,double svCurrentPosition,double actualCurrentTargetPose,double finalTargetPose,double estimatedFinalPose,double svKp,double svKi,double svKd,double thumbEnc,double indexEnc,double middleEnc,double enc8,std::vector<double> &pressureTarget,std::vector<double> &actualPressure,std::vector<double> &pwm,std::vector<int> &fingersList){

	using yarp::os::Bottle;

	Bottle& ctrlBottle = portControlDataOut.prepare();
	ctrlBottle.clear();

	double fingerJoint;

	ctrlBottle.addString(taskId);//1
	ctrlBottle.addString(experimentDescription);//2
	ctrlBottle.addString(previousExperimentDescription);//3
	ctrlBottle.addInt(pressureTarget.size());//4
	ctrlBottle.addDouble(targetGripStrength);//5
	ctrlBottle.addDouble(actualGripStrength);//6
	ctrlBottle.addDouble(u);//7
	ctrlBottle.addDouble(error);//8
	ctrlBottle.addDouble(actualCurrentTargetPose);//9
	ctrlBottle.addDouble(finalTargetPose);//10
	ctrlBottle.addDouble(svCurrentPosition);//11
	ctrlBottle.addDouble(estimatedFinalPose);//12
	ctrlBottle.addDouble(svKp);//13
	ctrlBottle.addDouble(svKi);//14
	ctrlBottle.addDouble(svKd);//15
	ctrlBottle.addDouble(thumbEnc);//16
	ctrlBottle.addDouble(indexEnc);//17
	ctrlBottle.addDouble(middleEnc);//18
	ctrlBottle.addDouble(enc8);//19
	for(int i = 0; i < pressureTarget.size(); i++){
		// TODO use function getProximalJointFromFingerNumber
//		if (fingersList[i] == 0) fingerJoint == 11;
//		else if (fingersList[i] == 1) fingerJoint == 13;
//		else fingerJoint == 9;
		
		ctrlBottle.addInt(fingersList[i]);// 20 ... 24 ...
//		ctrlBottle.addDouble(armEncodersAngles[fingerJoint]);// 8 ... 12 ...
		ctrlBottle.addDouble(pwm[i]);// 21 ... 25 ...
		ctrlBottle.addDouble(pressureTarget[i]);// 22 ... 26 ...
		ctrlBottle.addDouble(actualPressure[fingersList[i]]);// 23 ... 27...
	}

	portControlDataOut.write();

	return true;
}

bool PortsUtil::sendObjectRecognitionData(string taskId,int objectId,iCub::plantIdentification::ObjectRecognitionTask objRecTask,int extraCode1,int extraCode2,int skipPreviousRepetition,string experimentDescription,string previousExperimentDescription,iCub::plantIdentification::TaskCommonData *commonData){

	using yarp::os::Bottle;

	Bottle& objRecognBottle = portObjRecognDataOut.prepare();
	objRecognBottle.clear();

    // text data
	objRecognBottle.addString(experimentDescription);
	objRecognBottle.addString(previousExperimentDescription);

	// no text data
	// general information (6) (1-6)
	objRecognBottle.addInt(objectId);
	objRecognBottle.addInt(objRecTask);
	objRecognBottle.addInt(extraCode1);
	objRecognBottle.addInt(extraCode2);
	objRecognBottle.addString(taskId);
	objRecognBottle.addInt(skipPreviousRepetition);

	// logging raw tactile data (24) (7-30)
//    objRecognBottle.addString("midRawTact");
	for(size_t j = 0; j < commonData->fingerTaxelsRawData[1].size(); j++){
		objRecognBottle.addDouble(commonData->fingerTaxelsRawData[1][j]);
	}
//    objRecognBottle.addString("thmbRawTact");
	for(size_t j = 0; j < commonData->fingerTaxelsRawData[4].size(); j++){
		objRecognBottle.addDouble(commonData->fingerTaxelsRawData[4][j]);
	}
	// logging compensated tactile data (24) (31-54)
//    objRecognBottle.addString("midCompTact");
	for(size_t j = 0; j < commonData->fingerTaxelsData[1].size(); j++){
		objRecognBottle.addDouble(commonData->fingerTaxelsData[1][j]);
	}
//    objRecognBottle.addString("thmbCompTact");
	for(size_t j = 0; j < commonData->fingerTaxelsData[4].size(); j++){
		objRecognBottle.addDouble(commonData->fingerTaxelsData[4][j]);
	}
	// logging processed tactile data (2) (55-56)
//    objRecognBottle.addString("overallTactVal");
	objRecognBottle.addDouble(commonData->overallFingerPressureByWeightedSum[1]);
	objRecognBottle.addDouble(commonData->overallFingerPressureByWeightedSum[4]);


	// logging raw encoders (16) (57-72)
//    objRecognBottle.addString("handRawAng");
	for(size_t i = 0; i < commonData->fingerEncodersRawData.size(); i++){
		objRecognBottle.addInt(commonData->fingerEncodersRawData[i]);
	}
//    objRecognBottle.addString("armAng");
	// logging encoders (16) (73-88)
	for(size_t i = 0; i < commonData->armEncodersAngles.size(); i++){
		objRecognBottle.addDouble(commonData->armEncodersAngles[i]);
	}

	portObjRecognDataOut.write();

	return true;
}


bool PortsUtil::readFingerSkinRawData(std::vector<std::vector<double> > &fingerTaxelsRawData){

	using yarp::sig::Vector;

	Vector *iCubSkinData = portSkinRawIn.read(false);
    
	//TODO generalize fingers number
    if (iCubSkinData) {
		for(size_t i = 0; i < 5; i++){
			for (size_t j = 0; j < fingerTaxelsRawData[i].size(); j++){
				fingerTaxelsRawData[i][j] = (*iCubSkinData)[12*i + j];
			}
		}
	}

	return true;
}

bool PortsUtil::readFingerSkinCompData(std::vector<std::vector<double> > &fingerTaxelsData){

	using yarp::sig::Vector;

	Vector *iCubSkinData = portSkinCompIn.read(false);
    
	//TODO generalize fingers number
    if (iCubSkinData) {
		for(size_t i = 0; i < 5; i++){
			for (size_t j = 0; j < fingerTaxelsData[i].size(); j++){
				fingerTaxelsData[i][j] = (*iCubSkinData)[12*i + j];
			}
		}
	}

	return true;
}

bool PortsUtil::readFingerEncodersRawData(std::vector<double> &fingerEncodersRawData){

	using yarp::sig::Vector;

	Vector *iCubEncRawData = portHandEncodersRawIn.read(false);
    
    if (iCubEncRawData) {
		for (size_t i = 0; i < fingerEncodersRawData.size(); i++){
			fingerEncodersRawData[i] = (*iCubEncRawData)[i];
		}
	}

	return true;
}

bool PortsUtil::readPolicyActionsData(std::vector<double> &policyActionsData){

	using yarp::sig::Vector;

	Vector *iCubPolicyActionsData = portPolicyActionsIn.read(false);
    
    if (iCubPolicyActionsData) {
		for (size_t i = 0; i < policyActionsData.size(); i++){
			policyActionsData[i] = (*iCubPolicyActionsData)[i];
		}
		return true;
	} else {
		return false;
	}
}

bool PortsUtil::release(){

	portLogDataOut.interrupt();
	portSkinCompIn.interrupt();

	portLogDataOut.close();
	portSkinCompIn.close();

	return true;
}

/* *********************************************************************************************************************** */

