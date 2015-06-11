import iCubInterface
import numpy as np
import yarp
import time
import sys

def reachPosition(jointToMove,actionSteps,angleStep,kpGain,maxVoltage,targetPos,maxIterations,logEnabled,iCubI,fd):
   
    fullEncodersData = iCubI.readEncodersData()
    startingEncoderValue = fullEncodersData[jointToMove]

    if targetPos > startingEncoderValue:
        scale = 1
    else:
        scale = -1

    encoderValue = startingEncoderValue

    iterCounter = 0
    while iterCounter < maxIterations and scale*encoderValue < scale*targetPos:

        currentTarget = encoderValue + angleStep;
        intIterCounter = 0
        while intIterCounter < actionSteps:

            fullEncodersData = iCubI.readEncodersData()
            encoderValue = fullEncodersData[jointToMove]

            pwmToUse = kpGain*scale*(currentTarget - encoderValue)
            if pwmToUse > maxVoltage:
                pwmToUse = maxVoltage
            elif pwmToUse < -maxVoltage:
                pwmToUse = -maxVoltage
            iCubI.openLoopCommand(jointToMove,pwmToUse)

            if logEnabled:
                fd.write(str(iterCounter*0.10))
                fd.write(" ")
                fd.write(str(encoderValue-startingEncoderValue))
                fd.write(" ")
                fd.write(str(currentTarget-startingEncoderValue))
                fd.write(" ")
                fd.write(str((iterCounter+1)*angleStep))
                fd.write(" ")
                fd.write(str(pwmToUse))
                fd.write("\n")

#            if logEnabled:
#                print iterCounter*0.10,encoderValue-startingEncoderValue,currentTarget,(iterCounter+1)*angleStep

            intIterCounter = intIterCounter + 1

            time.sleep(0.01)

        iterCounter = iterCounter + 1

    iCubI.openLoopCommand(jointToMove,0.0)

    if iterCounter == maxIterations:
        print 'reaching ',targetPos,' failed'
        sys.exit()


def changeDirectionNTimes(jointToMove,actionSteps,angleStep,kpGain,maxVoltage,maxIterations,logEnabled,iCubI,fd):
   
    fullEncodersData = iCubI.readEncodersData()
    startingEncoderValue = fullEncodersData[jointToMove]

    encoderValue = startingEncoderValue

    iterCounter = 0
    while iterCounter < maxIterations:

        if int(iterCounter/5)%2 == 0:
            scale = 1
        else:
            scale = -1

        currentTarget = encoderValue + scale*angleStep;
        intIterCounter = 0
        while intIterCounter < actionSteps:

            fullEncodersData = iCubI.readEncodersData()
            encoderValue = fullEncodersData[jointToMove]

            pwmToUse = kpGain*(currentTarget - encoderValue)
            if pwmToUse > maxVoltage:
                pwmToUse = maxVoltage
            elif pwmToUse < -maxVoltage:
                pwmToUse = -maxVoltage
            iCubI.openLoopCommand(jointToMove,pwmToUse)

            if logEnabled:
                fd.write(str(iterCounter*0.10))
                fd.write(" ")
                fd.write(str(encoderValue-startingEncoderValue))
                fd.write(" ")
                fd.write(str(currentTarget-startingEncoderValue))
                fd.write(" ")
                if scale == 1:
                    fd.write(str((iterCounter%5+1)*angleStep))
                else:
                    fd.write(str(angleStep*5-(iterCounter%5+1)*angleStep))

                fd.write(" ")
                fd.write(str(pwmToUse))
                fd.write("\n")

#            if logEnabled:
#                print iterCounter*0.10,encoderValue-startingEncoderValue,currentTarget,(iterCounter+1)*angleStep

            intIterCounter = intIterCounter + 1

            time.sleep(0.01)

        iterCounter = iterCounter + 1

    iCubI.openLoopCommand(jointToMove,0.0)

    if iterCounter == maxIterations:
        print 'reaching ',targetPos,' failed'
        sys.exit()



def main():

    # module parameters
    dataDumperPortName = "/gpc/log:i"
    iCubIconfigFileName = "iCubInterface"
    jointToMove = 13
    # startinPosition1 < targetPosition < startingPosition2
    startingPosition1 = 10
    startingPosition2 = 50
    targetPosition = 30
    maxIterations = 50
    actionSteps = 20
    angleStep = 2
    kpGain = 100
    maxVoltage = 200
    
    fileName = "positionCD_A" + str(actionSteps) + "_E" + str(angleStep) + "_K" + str(kpGain) + "_M" + str(maxVoltage) + ".txt"; 
    fd = open(fileName,"w")

    # load iCub interface
    iCubI = iCubInterface.ICubInterface(dataDumperPortName,iCubIconfigFileName)
    iCubI.loadInterfaces()

    # set position mode
    iCubI.setPositionMode([jointToMove])

    # put finger in startingPosition1
    iCubI.setJointPosition(jointToMove,startingPosition1)

    # wait for the user
    raw_input("- press enter to move the finger -")

    # initialize open loop mode
    iCubI.setOpenLoopMode([jointToMove])

    # move finger from startingPosition1 to targetPosition
#    reachPosition(jointToMove,actionSteps,angleStep,kpGain,maxVoltage,targetPosition,maxIterations,True,iCubI,fd)
    changeDirectionNTimes(jointToMove,actionSteps,angleStep,kpGain,maxVoltage,15,True,iCubI,fd)

#    time.sleep(0.5)

    # move finger from targetPosition to startingPosition2
#    reachPosition(jointToMove,pwm,startingPosition2,maxIterations,True,iCubI,fd)

    # set position mode
##    iCubI.setPositionMode([jointToMove])

    # put finger in startingPosition2
##    iCubI.setJointPosition(jointToMove,startingPosition2)

#    time.sleep(0.5)

    # initialize open loop mode
##    iCubI.setOpenLoopMode([jointToMove])

    # move finger from startingPosition2 to targetPosition
#    reachPosition(jointToMove,pwm,targetPosition,maxIterations,False,iCubI,fd)

#    time.sleep(0.5)

    # move finger from targetPosition to startingPosition2
#    reachPosition(jointToMove,pwm,startingPosition2,maxIterations,True,iCubI,fd)

    fd.close()

    # restore position mode and close iCubInterface
    iCubI.setPositionMode([jointToMove])
    iCubI.closeInterface()
 
		
if __name__ == "__main__":
    main()
