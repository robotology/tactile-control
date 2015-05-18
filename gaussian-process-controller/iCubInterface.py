import numpy as np
import sys
import os
import yarp

class ICubInterface():

    def __init__(self, configFileName, path=''):
        self.isControllerLoaded = False
        if(path == ''):
            path = os.path.dirname(os.path.abspath(__file__))
        self.configFileFullName = path +'/r'+ configFileName + '.ini'

    def readFloat(self,fileDescriptor,nameString):
        return float(readString(fileDescriptor,nameString))
      
    def readInt(self,fileDescriptor,nameString):
        return int(round(readFloat(fileDescriptor,nameString)))
      
    def readString(self,fileDescriptor,nameString):
        splitline = fileDescriptor.readline().split()
        assert (nameString == splitline[0]),"unexpected entry"
        return splitline[1]
      
    def loadInterfaces(self):
        # load parameters from config file
		self.params = dict()
		fileDescriptor = open(self.configFileFullName,'r')
		self.params['robot'] = readString(fileDescriptor,'robot')
		self.params['whichHand'] = readString(fileDescriptor,'whichHand')
		
        # create driver and options
        self.driver = yarp.PolyDriver()
        options = yarp.Property()

        # set poly driver options
        options.put("robot",self.params['robot'])
        options.put("device","remote_controlboard")
        options.put("local","/example_enc/client")
        options.put("remote","/icub/" + self.params['whichHand'] + "_arm")

        # open and connect ports
        self.tactDataPort = yarp.BufferedPortBottle()
        self.tactDataPort.open("/local/icub/skin/" + self.params['whichHand'] + "_hand:i")
        yarp.Network.connect("/icub/skin/" + self.params['whichHand'] + "_hand","/local/icub/skin/" + self.params['whichHand'] + "_hand:i")

        # open driver
        print 'Opening motor driver'
        self.driver.open(options)
        if not self.driver.isValid():
            print 'Cannot open driver!'
            sys.exit()

        # create interfaces
        print 'Viewing interfaces...'
        self.iPos = self.driver.viewIPositionControl()
        if self.iPos is None:
            print 'Cannot view position interface!'
            sys.exit()
        self.iEnc = self.driver.viewIEncoders()
        if self.iEnc is None:
            print 'Cannot view encoders interface!'
            sys.exit()
        self.iVel = self.driver.viewIVelocityControl()
        if self.iVel is None:
            print 'Cannot view velocity interface!'
            sys.exit()
        self.iCtrl = self.driver.viewIControlMode()
        if self.iCtrl is None:
            print 'Cannot view control mode interface!'
            sys.exit()
		solf.iOlc = driver.viewIOpenLoopControl()
        if self.iOlc is None:
            print 'Cannot view open loop control mode interface!'
            sys.exit()

        self.numJoints = self.iPos.getAxes()

        # wait a bit for the interfaces to be ready
        yarp.Time_delay(1.0)

        # read encoders for the first time
		print 'reading encoders data'
        self.previousEncodersData = yarp.Vector(self.numJoints)
        ret = self.iEnc.getEncoders(self.previousEncodersData.data())
        while ret is False:
            ret = self.iEnc.getEncoders(self.previousEncodersData.data())

        # read tactile data for the first time
		print 'reading tactile data'
        self.previousTactileData = self.tactDataPort.read(True)
        while self.previousTactileData is None:
            self.previousTactileData = self.tactDataPort.read(True)
		
#        # store ref accelerations
#        self.refAccelerations = yarp.Vector(self.numJoints)
#        self.iVel.getRefAcceleration(self.refAccelerations.data())

#        # store control modes
#        self.controlModes = yarp.Vector(self.numJoints)
#        self.iCtrl.getControlModes(self.controlModes.data())

        self.isControllerLoaded = True

    def readTactileData(self):
        tactBtl = self.tactDataPort.read(False)
        if tactBtl is None:
            tactBtl = self.previousTactileData
        else:
            self.previousTactileData = tactBtl
        return tactBtl
    
    def readEncodersData(self):
        encodersData = yarp.Vector(self.numJoints)
        ret = self.iEnc.getEncoders(encodersData.data())
        if ret is False:
            encodersData = self.previousEncodersData
        else:
            self.previousEncodersData = encodersData
        return encodersData

    def velocityCommand(self,jointToMove,vel):
#       self.iCtrl.setVelocityControlMode(jointToMove)
#       self.iVel.setRefAcceleration(jointToMove,refAcc)
        ret = self.iVel.velocityMove(jointToMove,vel)
        if ret is False:
            print 'velocity command failed'

    def openLoopCommand(self,jointToMove,pwm):
#       self.iCtrl.setVelocityControlMode(jointToMove)
#       self.iVel.setRefAcceleration(jointToMove,refAcc)
        ret = iOlc.setRefOutput(jointToMove,pwm)
        if ret is False:
            print 'open loop command failed'

    def stopMoving(self,jointList):
		for i in range(len(jointList)):
			self.iVel.stop(jointList[i])

    def setVelocityMode(self,jointList):
		for i in range(len(jointList)):
			self.iCtrl.setVelocityMode(jointList[i])

    def setPositionMode(self,jointList):
		for i in range(len(jointList)):
			self.iCtrl.setPositionMode(jointList[i])

    def setOpenLoopMode(self,jointList):
		for i in range(len(jointList)):
			self.iCtrl.setOpenLoopMode(jointList[i])


    def closeInterface(self):

        # closing the driver
        self.driver.close()

