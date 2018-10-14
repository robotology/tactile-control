# Modules description

The library `tactile-control-lib` allows to carry out a two/three-finger stable grasp. First, the fingers move towards the object and stop when a contact is detected, then the hand configuration changes to improve stability, exploiting a Gaussian mixture model trained using learning by demonstration. In addiction, it provides a useful interface to carry out tactile object recognition (see below for details).  
The module `tactile-control-wrapper` is a simple example of module that implements such library

## Dependencies

* YARP
* icub-main
* icub-contrib (only `tactile-control-wrapper`)
* GURLS (only `tactile-control-lib`)

## Compiling the code

 An example of compilation for Linux system is the following:

```
 git clone https://github.com/robotology/tactile-control.git
 
 cd tactile-control-lib
 mkdir build
 cd build
 ccmake ..
 make install
 
 cd ../..
 cd tactile-control-wrapper
 mkdir build
 cd build
 ccmake ..
 make install
 
```

## How to run 

### Prerequisities

* `iCubStartup` (including [`iKinGazeCtrl`](http://eris.liralab.it/iCub/main/dox/html/group__iKinGazeCtrl.html) )
* `SkinGuiAll`

### Running the `tactileControlWrapper` module

Three configuration files must be prepared before running the `tactileControlWrapper` module: 

* **module configuration file**: as usual, it initializes the variables of the module.
* **tactile library configuration file**: it is used by the module to initialize generic variables (not related to any particular iCub robot) of the tactile library. Its name is specified in the property `libConfigFileName` of the _module configuration file_.  
* **tactile library robot-specific configuration file**: it is used by the module to initialize robot-specific variables (related to a specific iCub, e.g. iCubGenova01) of the tactile library. Its name is obtained by concatening the _tactile library configuration file_ name to `_<icub>`, where `<icub>` is specified in the property `icub` of the _tactile library configuration file_.

To run the module, launch `tactileControlWrapper --from <module_configuration_file>` (a basic _module configuration file_ for tactile object recognition is `confTactileControlWrapper_objRec.ini`)

## How to carry out tactile object recognition

The `tactileControlWrapper` module provides a user-friendly rpc interface that can be used to collect data by grasping the object, train a model on such data and exploit the learnt model to run inference. Further objects can be added online at any time.
First, connect to the rpc port: `/<port_prefix>/cmd:i`, where `<port_prefix>` is specified in the property `portPrefix` of the _tactile library configuration file_. It provides many commands, here a list of the main ones:

* `help`: shows a summary of the possible commands
* `set <parameter_name> <parameter_value>`: sets the parameter `<parameter_name>` to `<parameter_value>`
* `get <parameter_name>`: gets the value of the parameter `<parameter_name>`
* `show set`: shows the current settings, that is the whole list of parameters of the module
* `open`: opens the active hand and stops any running task
* `arm`: sets the arm in home position
* `set_grip_strength <grip_strength_value>`: sets the desired grip strength to `<grip_strength_value>` (equivalently you can run `set control.high.gripStrength <grip_strength_value>`, the shortcut is provided because frequently used)
* `obj_rec <command_type> <command_parameter>`: introduces to several commanands related to the object recognition task (see below for details)
* `quit`: stops the module

### Common pipeline

Here is an example of the steps to be followed to carry out tactile object recognition (it assumes you are using the _module configuration file_ `confTactileControlWrapper_objRec.ini`):

* Connect to the rpc port `/tactileControlWrapper/cmd:i`
* Type `arm up` to raise the robot arm (you can specify which arm/hand in the property `hand`of the _module configuration file_)
* Type `open` to open the robot hand
* Add the objects you want to use (further objects can be added at any time). For each object type `obj_rec add_new_object <object_name>`, where `<object_name>` is a descriptive name of the object, such as "red_ball". The first object added will have ID 1, the second ID 2 and so on. The ID must be provided when collecting features, so write it down.
* Before starting collecting features of a given object (included in the set above), its ID must be provided, so type `set ml.objectID <object_ID>`
* Put the object between the thumb and the middle finger and run `grasp`. The data collection process begins:
  1) the thumb and the middle finger slowly get in touch with the object,
  2) grasp is stabilized,
  3) encoder and tactile data at the thumb and middle finger are stored,
  4) the object is squeezed,
  5) encoder and tactile data at the thumb and middle finger are stored,
  6) all other fingers wrap around the object,
  7) encoder data of the whole hand are stored,
  8) the hand opens.
  
  The resultant feature vector is stored along with the object ID, but no model is trained yet. The described grasp is performed (in place of a standard grasp focused just on stabilization) because the `ml.objRecTaskEnabled` is set to `true`, while the data collection is performed because the `ml.dataCollectionEnabled` is set to `true`. If anything goes wrong during the data collection grasp, have a look at the _Tips and tricks_ section below.
* Repeat the data collection as many times as needed for all the objects in the current set (remember to change the object ID when switching object). If you want to discard the last collected feature vector, type `obj_rec discard_last`. If you want to discard all the collected data type `obj_rec clear`.
* Type `obj_rec process_data` to add the collected feature vectors to the current training set and train on the data. Notice that once the last collected features are transferred into the training set, the `discard_last` and `clear` commands cannot be applied anymore (the set of collected features is empty now).
* Once you train on the data, a learnt model is available for inference. To classify an object, set `ml.objectClassificationEnabled` to `true` (and `ml.dataCollectionEnabled` to `false` if you want to interrupt the data collection), then put the object in the robot hand and run `grasp`. At the end of the data collection process, the feature vector will be used for inference (there is no need to specify `ml.objectID` of course, unless `ml.dataCollectionEnabled` is still `true` to keep collecting data while testing). As result of the inference, the string "_I think this is the <object_name>_" is sent to the port `/<port_prefix>/<hand>_hand/speaker:o`, where `<port_prefix>` is specified in the property `portPrefix` of the _tactile library configuration file_, while `hand` (_left_ or _right_) is specified in the property `hand` of the _module configuration file_.
* At this point, you can keep collecting features and even adding new objects. Then, when you run `obj_rec process_data`, the new features and object names will be added to the previous training set, and training will performed on the whole data.
* Notice that you can easily store on/load from disk the current training set, object list and trained model using the following commands:
  * `load_train_set / save_train_set <file_suffix>`: the training set is loaded from / saved to the files `<path>/trainingSetX_<file_suffix>.dat` (feature vectors) and `<path>/trainingSetY_<file_suffix>.dat` (1vsAll label), where `<path>` is specified in the property `ml.trainingDataPath` of the _tactile library configuration file_ 
  * `load_objects / save_objects <file_suffix>`: the object names list is loaded from / saved to the file `<path>/objectNames_<file_suffix>.dat`
  * `load_model / save_model <file_suffix>`: the trained model is loaded from / saved to the file `<path>/model_<file_suffix>.dat` 
  
** Tips and Tricks

When the module properties are not properly tuned for the current robot, during the grasping process many things can go wrong (when not specified, the module parameters mentioned below are assumed to be in the _tactile library robot-specific configuration file_):
* during the approach phase, one or more fingers involved do not move at all: check that `approach.maxPwm` or `approach.timeout` are not too low.
* when the fingers get in touch with the object, the grasp stabilization does not start: check that `approach.threshold` is not too high.
* For kinematic reasons, the thumb and the middle finger do not properly get in touch while closing: you need to adjust the thumb adduction/abduction joint and the thumb/middel finger distal joints
* grasp is very unstable (fingers oscillate): the low level PID gains (`control.pidKp` and `control.pidKi`) and the high level PID gains (`control.high.pidKp` and `control.high.pidKi`) probably need to be tuned. If different fingers have different sensitivity when touched, play also with the property `fingertipSensitivity` which provides, for each finger, a scale factor used to adjust the skin tactile output.
* either the initial grip strength or the grip strength used for squeezing are not satisfying (the object falls or it is squeezed too much): change the parameters `control.high.gripStrength` and `ml.gripStrengthForSqueezing`
* when wrapping all the fingers around the object (final phase of the object recognition data collection), one or more fingers do not move (or they apply too much force): check that ml.handEnclosureIndexProxJointPwm, ml.handEnclosureRingLittleJointPwm and ml.handEnclosureIndexDistJointPwm are not too low/high


  





