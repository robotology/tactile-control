#include "TactileControl/util/ICubUtil.h"

#include "TactileControl/data/Parameters.h"

#include <math.h> 

using tactileControl::ICubUtil;


int ICubUtil::getFingerFromJoint(int joint){

    if (isThumb(joint)) return THUMB_FINGERTIP;
    if (isIndexFinger(joint)) return INDEX_FINGERTIP;
    if (isMiddleFinger(joint)) return MIDDLE_FINGERTIP;
    if (isRingOrLittleFinger(joint)) return RING_FINGERTIP;

    return -1;
}

bool ICubUtil::isDistal(int joint){

    return joint == THUMB_DISTAL_JOINT || joint == INDEX_DISTAL_JOINT || joint == MIDDLE_DISTAL_JOINT;
}

bool ICubUtil::isThumb(int joint){
    
    return joint == THUMB_ABDUCTION_JOINT || joint == THUMB_PROXIMAL_JOINT || joint == THUMB_DISTAL_JOINT;
}

bool ICubUtil::isIndexFinger(int joint){

    return joint == INDEX_PROXIMAL_JOINT || joint == INDEX_DISTAL_JOINT;
}

bool ICubUtil::isMiddleFinger(int joint){

    return joint == MIDDLE_PROXIMAL_JOINT || joint == MIDDLE_DISTAL_JOINT;
}

bool ICubUtil::isRingOrLittleFinger(int joint){

    return joint == RING_LITTLE_JOINT;
}

double ICubUtil::getGripStrength(int numFingers,const std::vector<double> &overallFingerForce){

    double thumbForce = overallFingerForce[THUMB_FINGERTIP];
    double indexFingerForce = overallFingerForce[INDEX_FINGERTIP];
    double middleFingerForce = overallFingerForce[MIDDLE_FINGERTIP];

    if (numFingers == 2){
        return (thumbForce + middleFingerForce)/2;
    } else {
        return 2.0/3.0 * thumbForce + 1.0/3.0 * (indexFingerForce + middleFingerForce);
    }
}

double ICubUtil::getObjectPosition(int numFingers,const std::vector<double> &armEncoderAngles){

    if (numFingers == 2){
        return (armEncoderAngles[MIDDLE_PROXIMAL_JOINT] - armEncoderAngles[THUMB_PROXIMAL_JOINT])/2;
    } else {
        return ((armEncoderAngles[MIDDLE_PROXIMAL_JOINT] + armEncoderAngles[INDEX_PROXIMAL_JOINT])/2 - armEncoderAngles[THUMB_PROXIMAL_JOINT])/2;
    }
}

double ICubUtil::getHandAperture(int numFingers,const std::vector<double> &armEncoderAngles){

    if (numFingers == 2){
        return 180 - armEncoderAngles[MIDDLE_PROXIMAL_JOINT] - armEncoderAngles[THUMB_PROXIMAL_JOINT];
    } else {
        return 180 - (armEncoderAngles[MIDDLE_PROXIMAL_JOINT] + armEncoderAngles[INDEX_PROXIMAL_JOINT])/2 - armEncoderAngles[THUMB_PROXIMAL_JOINT];
    }
}

double ICubUtil::getIndexMiddleDifference(int numFingers,const std::vector<double> &armEncoderAngles){

    if (numFingers == 2){
        return 0;
    } else {
        return armEncoderAngles[MIDDLE_PROXIMAL_JOINT] - armEncoderAngles[INDEX_PROXIMAL_JOINT];
    }
}




double ICubUtil::getForce(const std::vector<double>& fingerTaxelsData,tactileControl::ForceCalculationMode forceCalculationMode){

    switch(forceCalculationMode){

    case SIMPLE_SUM:
        return getForceBySimpleSum(fingerTaxelsData);
        break;
    case WEIGHTED_SUM:
        return getForceByWeightedSum(fingerTaxelsData);
        break;
    default:
        return 0;
    }

}

double ICubUtil::getForceBySimpleSum(const std::vector<double>& fingerTaxelsData){

    double partialSum = 0.0;

    for(int i = 0; i < fingerTaxelsData.size(); i++){
        partialSum += fingerTaxelsData[i];
    }

    return partialSum;
}

double ICubUtil::getForceByWeightedSum(const std::vector<double>& fingerTaxelsData){

    double partialXSum = 0.0;
    double partialYSum = 0.0;
    double partialZSum = 0.0;
    std::vector<double> unitVector(3);

    for(int i = 0; i < fingerTaxelsData.size(); i++){

        getUnitVector(i,unitVector);

        partialXSum += fingerTaxelsData[i]*unitVector[0];
        partialYSum += fingerTaxelsData[i]*unitVector[1];
        partialZSum += fingerTaxelsData[i]*unitVector[2];
    }

    return sqrt(partialXSum*partialXSum + partialYSum*partialYSum + partialZSum*partialZSum);
}

void ICubUtil::getUnitVector(int index,std::vector<double>& unitVector){

    switch(index){

    case 0:
        unitVector[0] = -1.0;
        unitVector[1] = 0.0;
        unitVector[2] = 0.0;
        break;

    case 1:
        unitVector[0] = -0.39956;
        unitVector[1] = 0.0;
        unitVector[2] = 0.91671;
        break;

    case 2:
        unitVector[0] = -0.39956;
        unitVector[1] = 0.0;
        unitVector[2] = 0.91671;
        break;

    case 3:
        unitVector[0] = -1.0;
        unitVector[1] = 0.0;
        unitVector[2] = 0.0;
        break;

    case 4:
        unitVector[0] = -0.78673;
        unitVector[1] = 0.60316;
        unitVector[2] = 0.13140;
        break;

    case 5:
        unitVector[0] = -0.30907;
        unitVector[1] = 0.47765;
        unitVector[2] = 0.82239;
        break;

    case 6:
        unitVector[0] = 0.0;
        unitVector[1] = 1.0;
        unitVector[2] = 0.0;
        break;

    case 7:
        unitVector[0] = 0.30907;
        unitVector[1] = 0.47765;
        unitVector[2] = 0.82239;
        break;

    case 8:
        unitVector[0] = 0.78673;
        unitVector[1] = 0.60316;
        unitVector[2] = 0.13140;
        break;

    case 9:
        unitVector[0] = 1.0;
        unitVector[1] = 0.0;
        unitVector[2] = 0.0;
        break;

    case 10:
        unitVector[0] = 0.39956;
        unitVector[1] = 0.0;
        unitVector[2] = 0.91671;
        break;

    case 11:
        unitVector[0] = 0.39956;
        unitVector[1] = 0.0;
        unitVector[2] = 0.91671;
        break;
    }

}

void ICubUtil::normalizeVector(const std::vector<double> &inputVector, std::vector<double> &outputVector){

    double avarage = 0;

    for (int i = 0; i < inputVector.size(); i++){
        avarage += inputVector[i];
    }
    avarage /= inputVector.size();

    std::vector<double> zeroAvarage(inputVector.size());
    for (int i = 0; i < zeroAvarage.size(); i++){
        zeroAvarage[i] = inputVector[i] - avarage;
    }

    double stdDev = 0;
    for (int i = 0; i < zeroAvarage.size(); i++){
        stdDev += zeroAvarage[i] * zeroAvarage[i];
    }
    stdDev = sqrt(stdDev / zeroAvarage.size());

    outputVector.resize(zeroAvarage.size());
    for (int i = 0; i < outputVector.size(); i++){
        outputVector[i] = zeroAvarage[i] / stdDev;
    }
}