#include "TactileControl/ObjectRecognitionManager.h"

#include <yarp/os/LogStream.h>

using tactileControl::ObjectRecognitionManager;

ObjectRecognitionManager::ObjectRecognitionManager(){

    managerInitialized = false;

    dbgTag = "ObjectRecognitionManager: ";
}

bool ObjectRecognitionManager::open(const tactileControl::HandController &handController){

    if (handController.controllerInitialized){
        mlUtil = handController.mlUtil;
        portUtil = handController.portUtil;
        taskData = handController.taskData;
        managerInitialized = true;
    }
    else {
        yError() << dbgTag << "failed to initialize object recognition manager";
        return false;
    }

    return true;
}

bool ObjectRecognitionManager::loadTrainingSet(std::string fileSuffix){

    if (!managerInitialized) return false;

    return mlUtil->loadTrainingSetFromFile(fileSuffix);
}

bool ObjectRecognitionManager::saveTrainingSet(std::string fileSuffix){

    if (!managerInitialized) return false;

    return mlUtil->saveTrainingSetToFile(fileSuffix);
}

bool ObjectRecognitionManager::loadObjects(std::string fileSuffix){

    if (!managerInitialized) return false;

    return mlUtil->loadObjectNamesFromFile(fileSuffix);
}

bool ObjectRecognitionManager::saveObjects(std::string fileSuffix){

    if (!managerInitialized) return false;

    return mlUtil->saveObjectNamesToFile(fileSuffix);
}

bool ObjectRecognitionManager::loadModel(std::string fileSuffix){

    if (!managerInitialized) return false;

    return mlUtil->loadModelFromFile(fileSuffix);
}

bool ObjectRecognitionManager::saveModel(std::string fileSuffix){

    if (!managerInitialized) return false;

    return mlUtil->saveModelToFile(fileSuffix);
}

bool ObjectRecognitionManager::viewData(){

    if (!managerInitialized) return false;

    return mlUtil->viewData();
}

bool ObjectRecognitionManager::train(){

    if (!managerInitialized) return false;

    return mlUtil->trainClassifier();
}

bool ObjectRecognitionManager::discardLastFeatures(){

    if (!managerInitialized) return false;

    return mlUtil->discardLastCollectedFeatures();
}

bool ObjectRecognitionManager::clearCollectedFeatures(){

    if (!managerInitialized) return false;

    return mlUtil->clearCollectedFeatures();
}

bool ObjectRecognitionManager::processCollectedData(){

    if (!managerInitialized) return false;

    return mlUtil->processCollectedData();
}

bool ObjectRecognitionManager::addNewObject(std::string objectName){

    if (!managerInitialized) return false;

    return mlUtil->addNewObject(objectName);
}

bool ObjectRecognitionManager::getReady(std::string fileSuffix){

    if (!managerInitialized) return false;

    bool ok = loadObjects(fileSuffix);
    if (ok) ok = loadTrainingSet(fileSuffix);
    if (ok) ok = train();

    return ok;
}

bool ObjectRecognitionManager::readVisualClassifierOutputScores(){

    return portUtil->readVisualClassifierAvgScores(taskData->visualScores);
}

bool ObjectRecognitionManager::reset(){

    if (!managerInitialized) return false;

    return mlUtil->reset();
}

bool ObjectRecognitionManager::close(){

    if (!managerInitialized) return false;

    mlUtil->release();

    yInfo() << dbgTag << "object recognition manager succesfully closed";

    return true;
}