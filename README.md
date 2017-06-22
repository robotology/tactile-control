# Modules description

The library `tactile-control-lib` allows to carry out a three-finger stable grasp. First, the fingers move towards the object and stop when a contact is detected, then the hand configuration changes to improve stability, exploiting a Gaussian mixture model trained using learning by demonstration.
The module `tactile-control-wrapper` is a simple example of module that makes use of such library

## Dependencies

* YARP
* icub-main
* icub-contrib (only `tactile-control-wrapper`)
