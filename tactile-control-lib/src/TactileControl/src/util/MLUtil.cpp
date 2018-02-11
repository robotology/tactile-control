#include "TactileControl/util/MLUtil.h"

#include "TactileControl/data/Parameters.h"

#include <yarp/os/LogStream.h>

using tactileControl::MLUtil;


MLUtil::MLUtil(){
    using std::string;
    using std::pair;

    objRecDataMap.insert(pair<ClassifierType, MLData*>(TACTILE_CLASSIFIER, new MLData()));
    objRecDataMap.insert(pair<ClassifierType, MLData*>(MULTIMODAL_CLASSIFIER, new MLData()));

    dbgTag = "MLUtil: ";
}

bool MLUtil::init(tactileControl::TaskData *taskData, tactileControl::PortUtil *portUtil){

    this->portUtil = portUtil;
    this->taskData = taskData;

    std::string pathPrefix = taskData->getString(PAR_ML_TRAINING_DATA_PATH);

    modelFileName = pathPrefix + "model_";
    objectNamesFileName = pathPrefix + "objectNames_";
    trainingSetXFileName = pathPrefix + "trainingSetX_";
    trainingSetYFileName = pathPrefix + "trainingSetY_";

    for (std::map<ClassifierType, MLData*>::iterator it = objRecDataMap.begin(); it != objRecDataMap.end(); ++it){
        objRecDataMap[it->first]->init();
    }

    return true;
}


bool MLUtil::loadTrainingSetFromFile(std::string fileSuffix){

    MLData* mlData = objRecDataMap[getClassifierType()];

    yInfo() << dbgTag << "loading training data...";

    mlData->xTr.readCSV(trainingSetXFileName + fileSuffix + ".dat");
    mlData->yTr.readCSV(trainingSetYFileName + fileSuffix + ".dat");

    yInfo() << dbgTag << " ...done";

    mlData->trainingSetLoaded = true;

    return true;
}


bool MLUtil::saveTrainingSetToFile(std::string fileSuffix){

    MLData* mlData = objRecDataMap[getClassifierType()];

    yInfo() << dbgTag << "saving training data...";

    mlData->xTr.saveCSV(trainingSetXFileName + fileSuffix + ".dat");
    mlData->yTr.saveCSV(trainingSetYFileName + fileSuffix + ".dat");

    yInfo() << dbgTag  << " ...done";

    return true;
}

bool MLUtil::loadObjectNamesFromFile(std::string fileSuffix){

    MLData* mlData = objRecDataMap[getClassifierType()];

    yInfo() << dbgTag << "loading object names list...";

    std::string objectName;
    std::string fileName = objectNamesFileName + fileSuffix + ".dat";
    std::ifstream objectNamesFile(fileName.c_str());
    mlData->objectsMap.clear();
    int i = 1;
    while (std::getline(objectNamesFile, objectName).good()){
        mlData->objectsMap.insert(std::pair<int, std::string>(i, objectName));
        i++;
    }
    objectNamesFile.close();

    yInfo() << dbgTag << " ...done";

    return true;
}

bool MLUtil::saveObjectNamesToFile(std::string fileSuffix){

    MLData* mlData = objRecDataMap[getClassifierType()];

    yInfo() << dbgTag << "saving object names list...";

    std::string fileName = objectNamesFileName + fileSuffix + ".dat";
    std::ofstream objectNamesFile(fileName.c_str());

    for (int i = 1; i <= mlData->objectsMap.size(); i++){

        objectNamesFile << mlData->objectsMap[i] << std::endl;

    }

    objectNamesFile.close();

    yInfo() << dbgTag << " ...done";

    return true;

}

bool MLUtil::loadModelFromFile(std::string fileSuffix){

    MLData* mlData = objRecDataMap[getClassifierType()];

    yInfo() << dbgTag << "loading model...";

    mlData->wrapper->loadOpt(modelFileName + fileSuffix + ".dat");

    yInfo() << dbgTag << " ...done";

    mlData->classifierTrained = true;

    return true;
}

bool MLUtil::saveModelToFile(std::string fileSuffix){

    MLData* mlData = objRecDataMap[getClassifierType()];

    yInfo() << dbgTag << "saving model...";

    mlData->wrapper->saveModel(modelFileName + fileSuffix + ".dat");

    yInfo() << dbgTag << " ...done";

    return true;
}

bool MLUtil::viewData(){

    MLData* mlData = objRecDataMap[getClassifierType()];

    yInfo();

    yInfo() << dbgTag << "-- X training --";
    for (int i = 0; i < mlData->xTr.rows(); i++){
        for (int j = 0; j < mlData->xTr.cols(); j++){
            std::cout << mlData->xTr(i, j) << "\t";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;


    yInfo() << dbgTag << "-- Y training --";
    for (int i = 0; i < mlData->yTr.rows(); i++){
        for (int j = 0; j < mlData->yTr.cols(); j++){
            std::cout << mlData->yTr(i, j) << "\t";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;


    yInfo() << dbgTag << "-- Collected features --";
    for (int i = 0; i < collectedFeatures.size(); i++){
        for (int j = 0; j < collectedFeatures[i].size(); j++){
            std::cout << collectedFeatures[i][j] << "\t";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;


    yInfo() << dbgTag << "-- Object names map --";
    for (std::map<int, std::string>::iterator it = mlData->objectsMap.begin(); it != mlData->objectsMap.end(); ++it){

        std::cout << it->first << "\t" << mlData->objectsMap[it->first] << std::endl;
    }


    std::cout << std::endl << std::endl;

    return true;
}


bool MLUtil::trainClassifier(){

    MLData* mlData = objRecDataMap[getClassifierType()];

    if (mlData->trainingSetLoaded){

        yInfo() << dbgTag << "Training started...";

        mlData->wrapper->train(mlData->xTr, mlData->yTr);

        yInfo() << dbgTag << "...finished!";

        mlData->classifierTrained = true;

        return true;

    }
    else {

        return false;

    }
}

bool MLUtil::testClassifierOneShot(const std::vector<double> &features, std::vector<double> &outputScores, bool sayResult, tactileControl::ClassifierType classifierType){

    if (objRecDataMap[classifierType]->trainingSetLoaded && objRecDataMap[classifierType]->classifierTrained){

        gurls::gMat2D<double> input(1, features.size());

        for (int i = 0; i < features.size(); i++){
            input(0, i) = features[i];
        }


        gurls::gMat2D<double> *output = objRecDataMap[classifierType]->wrapper->eval(input);

        // store output scores
        outputScores.resize(output->cols());
        for (int i = 0; i < output->cols(); i++){
            outputScores[i] = (*output)(0, i);
        }

        int numObjects = output->cols();
        int prediction;
        std::vector<int> predictions;
        getStandardPredictionsFrom1vsAll(*output, predictions);
        prediction = predictions[0];

        if (sayResult){
            sendDetectedObjectToPort(prediction + 1, classifierType);
        }
    }
    else {

        return false;

    }


}



int MLUtil::getArgMin(const gurls::gMat2D<double> &mat, int rowNum){

    int currIndex = -1;
    double currMax = -1000.0;

    for (int i = 0; i < mat.cols(); i++){
        if (mat(rowNum, i) > currMax){
            currMax = mat(rowNum, i);
            currIndex = i;
        }
    }

    return currIndex;
}

bool MLUtil::getStandardPredictionsFrom1vsAll(const gurls::gMat2D<double> &predictions1vsAll, std::vector<int> &predictions){

    predictions.resize(predictions1vsAll.rows());

    for (int i = 0; i < predictions1vsAll.rows(); i++){

        predictions[i] = getArgMin(predictions1vsAll, i);

    }

    return true;
}

bool MLUtil::sendDetectedObjectToPort(int objectNum, tactileControl::ClassifierType classifierType){

    std::string objectName;

    try {

        objectName = objRecDataMap[classifierType]->objectsMap[objectNum];
        portUtil->sendStringToSpeaker("I think this is the " + objRecDataMap[classifierType]->objectsMap[objectNum]);

    }
    catch (const std::out_of_range& oor){

        portUtil->sendStringToSpeaker("I do not know this object");
    }

    return true;
}

bool MLUtil::resizeMLMatrix(gurls::gMat2D<double> &mat, int rows, int cols){

    std::vector<std::vector<double> > tempStorage;

    tempStorage.resize(mat.rows());
    for (int i = 0; i < tempStorage.size(); i++){
        tempStorage[i].resize(mat.cols());
        for (int j = 0; j < tempStorage[i].size(); j++){
            tempStorage[i][j] = mat(i, j);
        }
    }
    mat.resize(rows, cols);
    for (int i = 0; i < tempStorage.size(); i++){
        for (int j = 0; j < tempStorage[i].size(); j++){
            mat(i, j) = tempStorage[i][j];
        }
    }

    return true;
}

bool MLUtil::reset(){

    MLData* mlData = objRecDataMap[getClassifierType()];

    mlData->reset();

    clearCollectedFeatures();
    objectIDs.clear();

    return true;
}

bool MLUtil::release(){

    for (std::map<ClassifierType, MLData*>::iterator it = objRecDataMap.begin(); it != objRecDataMap.end(); ++it){
        objRecDataMap[it->first]->release();
    }

    return true;
}


bool MLUtil::clearCollectedFeatures(){

    collectedFeatures.clear();

    return true;
}

bool MLUtil::addCollectedFeatures(const std::vector<double> &features, int objectID){

    collectedFeatures.push_back(features);
    objectIDs.push_back(objectID);

    return true;
}

bool MLUtil::discardLastCollectedFeatures(){

    if (collectedFeatures.size() > 0){
        collectedFeatures.pop_back();
        objectIDs.pop_back();
        return true;
    }
    else {
        return false;
    }

}

bool MLUtil::processCollectedData(){

    MLData* mlData = objRecDataMap[getClassifierType()];

    if (collectedFeatures.size() > 0){

        /// add the new features to xTr and yTr 

        int numPrevObjects = mlData->yTr.cols();
        int numTotalObjects = mlData->objectsMap.size();
        int numPrevSamples = mlData->xTr.rows();
        int numNewSamples = collectedFeatures.size();

        std::vector<std::vector<double> > tempStorage;

        // resize xTr
        resizeMLMatrix(mlData->xTr, numPrevSamples + numNewSamples, collectedFeatures[0].size());

        // resize yTr
        resizeMLMatrix(mlData->yTr, numPrevSamples + numNewSamples, numTotalObjects);


        for (int i = 0; i < collectedFeatures.size(); i++){

            // update xTr
            for (int j = 0; j < collectedFeatures[i].size(); j++){
                mlData->xTr(numPrevSamples + i, j) = collectedFeatures[i][j];
            }

            // update yTr
            for (int j = 0; j < numTotalObjects; j++){
                if (j + 1 == objectIDs[i]){
                    mlData->yTr(numPrevSamples + i, j) = 1;
                }
                else {
                    mlData->yTr(numPrevSamples + i, j) = -1;
                }
            }
        }

        // extend the new yTr columns with -1 (only in the old rows)
        if (numTotalObjects > numPrevObjects){
            for (int i = 0; i < numPrevSamples; i++){
                for (int j = numPrevObjects; j < numTotalObjects; j++){
                    mlData->yTr(i, j) = -1;
                }
            }
        }

        mlData->trainingSetLoaded = true;

        /// re-train the model

        trainClassifier();

        return true;

    }
    else {

        return false;

    }

}

bool MLUtil::addNewObject(std::string objectName){

    MLData* mlData = objRecDataMap[getClassifierType()];

    int newKey;
    if (!mlData->objectsMap.empty()){
        newKey = mlData->objectsMap.rbegin()->first + 1;
    }
    else {
        newKey = 1;
    }
    mlData->objectsMap.insert(std::pair<int, std::string>(newKey, objectName));

    return true;
}

tactileControl::ClassifierType MLUtil::getClassifierType(){

    return static_cast<tactileControl::ClassifierType>(taskData->getInt(PAR_ML_CLASSIFIER_TYPE));
}