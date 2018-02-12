#ifndef TACTILECONTROL_OBJECTRECOGNITIONMANAGER_H
#define TACTILECONTROL_OBJECTRECOGNITIONMANAGER_H

#include "TactileControl/HandController.h"
#include "TactileControl/util/MLUtil.h"
#include "TactileControl/util/PortUtil.h"
#include "TactileControl/data/TaskData.h"

#include <string>

namespace tactileControl {


    class ObjectRecognitionManager {

    private:

        tactileControl::MLUtil *mlUtil;
        tactileControl::PortUtil *portUtil;
        tactileControl::TaskData *taskData;

        bool managerInitialized;

        /* ****** Debug attributes                              ****** */
        std::string dbgTag;

    public:
        /**
        * Constructor.
        */
        ObjectRecognitionManager();

        /**
        * Initializes the object recognition manager. Returns true in case of success.
        */
        bool open(const tactileControl::HandController &handController);

        /**
        * Loads training set from file. Returns true in case of success.
        */
        bool loadTrainingSet(std::string fileSuffix);

        /**
        * Saves training set to file. Returns true in case of success.
        */
        bool saveTrainingSet(std::string fileSuffix);

        /**
        * Loads object names from file. Returns true in case of success.
        */
        bool loadObjects(std::string fileSuffix);

        /**
        * Saves object names to file. Returns true in case of success.
        */
        bool saveObjects(std::string fileSuffix);

        /**
        * Loads trained classifier from file. Returns true in case of success.
        */
        bool loadModel(std::string fileSuffix);

        /**
        * Saves trained classifier to file. Returns true in case of success.
        */
        bool saveModel(std::string fileSuffix);

        /**
        * View machine learning data, like training set, object names etc. (used for debugging), Returns true in case of success.
        */
        bool viewData();

        /**
        * Trains a new classifier using the training data. Returns true in case of success.
        */
        bool train();

        /**
        * Discards last istance of collected features. Returns true in case of success.
        */
        bool discardLastFeatures();

        /**
        * Clears all collected features not yet included in the model. Returns true in case of success.
        */
        bool clearCollectedFeatures();

        /**
        * Adds the last set of instances of collected features in the training set and re-train the classifier. Returns true in case of success.
        */
        bool processCollectedData();

        /**
        *  Adds a new object in the object names list. Returns true in case of success.
        */
        bool addNewObject(std::string objectName);

        /**
        * Loads object names from file, loads training data from file, trains the classifier. Returns true in case of success.
        */
        bool getReady(std::string fileSuffix);

        /**
        * Reads the output scores of the visual classifier and stores them.
        */
        bool readVisualClassifierOutputScores();

        /**
        * Checks if the object recognition task is complete.
        */
        bool isObjectRecognitionTaskComplete();

        /**
        * Resets everything so that data collection can be restared. Returns true in case of success.
        */
        bool reset();

        /**
        * Closes the object recognition manager and releases all the resources. Returns true in case of success.
        */
        bool close();

    };

} // namespace tactileControl

#endif // TACTILECONTROL_OBJECTRECOGNITIONMANAGER_H
