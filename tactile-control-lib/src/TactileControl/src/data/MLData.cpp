#include "TactileControl/data/MLData.h"

using tactileControl::MLData;


MLData::MLData() {
    
    trainingSetLoaded = false;
    classifierTrained = false;

    xTr.resize(0, 0);
    yTr.resize(0, 0);

    dbgTag = "MLData: ";
}

bool MLData::init(){

    objectsMap.clear();
    wrapper = new gurls::KernelRLSWrapper<double>("myWrapper");

    return true;
}

bool MLData::reset(){

    trainingSetLoaded = false;
    classifierTrained = false;

    objectsMap.clear();

    xTr.resize(0, 0);
    yTr.resize(0, 0);

    return true;
}

bool MLData::release(){

    delete(wrapper);

    return true;
}