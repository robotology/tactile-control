#include "TactileControl/thread/DataCollectionThread.h"

#include "TactileControl/util/ICubUtil.h"
#include "TactileControl/data/Parameters.h"
#include "TactileControl/data/Enums.h"

#include <cmath>
#include <gsl/gsl_sort.h>
#include <gsl/gsl_statistics.h>
#include <algorithm>

using tactileControl::DataCollectionThread;


DataCollectionThread::DataCollectionThread(int period, tactileControl::TaskData *taskData,tactileControl::ControllerUtil *controllerUtil,tactileControl::PortUtil *portUtil)
    : RateThread(period){

    this->controllerUtil = controllerUtil;
    this->portUtil = portUtil;
    this->taskData = taskData;

    dbgTag = "DataCollectionThread: ";
}


void DataCollectionThread::run(){

    // update tactile and encoders data
    updateRobotData();
}


bool DataCollectionThread::updateRobotData(){
    using std::cout;
    
    std::vector<double> fingersSensitivityScale(5);
    taskData->getList(PAR_COMMON_FINGER_SENSITIVITY,fingersSensitivityScale);

    // read raw tactile data
    portUtil->readFingerSkinRawData(taskData->fingerTaxelsRawData);

    // read compensated tactile data
    portUtil->readFingerSkinCompData(taskData->fingerTaxelsData,fingersSensitivityScale);

    // read raw encoders data
    portUtil->readFingerEncodersRawData(taskData->fingerEncodersRawData);
    
    // read processed encoders data
    controllerUtil->getArmEncoderAngles(taskData->armEncoderAngles);
    controllerUtil->getArmEncoderAngleReferences(taskData->armEncoderAngleReferences);

    processTactileData();

    return true;
}

bool DataCollectionThread::processTactileData(){

    bool useTactileWeightedSum = taskData->getBool(PAR_COMMON_USE_TACTILE_WEIGHTED_SUM);

    for(int i = 0; i < taskData->fingerTaxelsData.size(); i++){

        // compute force at the finger (two methods are used)
        taskData->overallFingerForceBySimpleSum[i] = ICubUtil::getForce(taskData->fingerTaxelsData[i],SIMPLE_SUM);
        taskData->overallFingerForceByWeightedSum[i] = ICubUtil::getForce(taskData->fingerTaxelsData[i],WEIGHTED_SUM);

        if (useTactileWeightedSum){
            taskData->overallFingerForce[i] = taskData->overallFingerForceByWeightedSum[i];
        } else {
            taskData->overallFingerForce[i] = taskData->overallFingerForceBySimpleSum[i];
        }

        // force cannot be negative
        taskData->overallFingerForce[i] = std::max(0.0,taskData->overallFingerForce[i]);

        // calculate force median
        taskData->previousOverallFingerForce[i][taskData->previousForceIndex[i]] = taskData->overallFingerForce[i];
        taskData->previousForceIndex[i] = (taskData->previousForceIndex[i] + 1)%taskData->previousOverallFingerForce[i].size();

        std::vector<double> previousOverallFingerForceCopy(taskData->previousOverallFingerForce[i]);

        gsl_sort(&previousOverallFingerForceCopy[0],1,previousOverallFingerForceCopy.size());
        taskData->overallFingerForceMedian[i] = gsl_stats_median_from_sorted_data(&previousOverallFingerForceCopy[0],1,previousOverallFingerForceCopy.size());

    }

    return true;
}