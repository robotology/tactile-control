#include "TactileControl/util/ICubUtil.h"

#include "TactileControl/data/Parameters.h"

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

double ICubUtil::getGripStrength(const std::vector<double> &overallFingerForce){

    double thumbForce = overallFingerForce[THUMB_FINGERTIP];
    double indexFingerForce = overallFingerForce[INDEX_FINGERTIP];
    double middleFingerForce = overallFingerForce[MIDDLE_FINGERTIP];

    return 2.0/3.0 * thumbForce + 1.0/3.0 * (indexFingerForce + middleFingerForce);
}

