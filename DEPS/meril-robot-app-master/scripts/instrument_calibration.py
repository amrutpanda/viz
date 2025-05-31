#!/usr/bin/python3

#
#   Developer : Philippe Piatkiewitz (philippe.piatkiewitz@vectioneer.com)
#   All rights reserved. Copyright (c) 2019-2022 VECTIONEER.
#

import os
import matplotlib

matplotlib.use('agg')
import matplotlib.pyplot as plt
import motorcortex
import time
from motorcortex_tools import *
from math import pi

# Tests
from instrument_calibration_procedures import set_PVA_mode
from instrument_calibration_procedures import set_generators
from instrument_calibration_procedures import set_reference
from instrument_calibration_procedures import calibrate_sterile_adapter_homing

try:
    plt.style.use('templates/styles/plotstyle.mplstyle')
    print("Loaded plotsyle")
except:
    print("Could not load plotstyle")

URL = "wss://192.168.2.101:5568:5567"

TEMPLATESFOLDER = "templates"
OUTPUTFOLDER = "results/"
PLOTFOLDER = "plots/"
DATAFOLDER = "data/"
if not os.path.exists(OUTPUTFOLDER + PLOTFOLDER):
    os.makedirs(OUTPUTFOLDER + PLOTFOLDER)

if not os.path.exists(OUTPUTFOLDER + DATAFOLDER):
    os.makedirs(OUTPUTFOLDER + DATAFOLDER)


class MCXConnection:
    def __init__(self, url=URL,
                 outputfolder="results/", datafolder="data/",
                 plotfolder="plots/", certificate="mcx.cert.crt"):
        self.url = url
        self.certificate = certificate
        self.outputfolder = outputfolder
        self.plotfolder = plotfolder
        self.datafolder = datafolder
        self.connect(self.url, certificate)

    def connect(self, url=URL, certificate="mcx.cert.crt"):
        motorcortex_types = motorcortex.MessageTypes()
        parameter_tree = motorcortex.ParameterTree()
        # Open request connection
        req, sub = motorcortex.connect(url, motorcortex_types, parameter_tree,
                                       certificate=certificate, timeout_ms=1000, login="", password="")
        tree = req.getParameterTree().get()
        self.req = req
        self.sub = sub

    def close(self):
        self.sub.close()
        self.req.close()


class mcxApp():
    def __init__(self):
        # references to common signals
        self.pathToModeCommand = "root/Logic/modeCommand"
        self.pathToStateCommand = "root/Logic/stateCommand"
        self.pathToState = "root/Logic/state"
        self.pathToActuator = "root/AxesControl/actuatorControlLoops/actuatorControlLoop%02d"
        self.pathToSignalGenerator = "root/ManipulatorControl/jointReferenceGenerator/signalGenerator%02d"
        self.pathToSetpointGenerator = "root/SetpointGenerators/SetpointGenerator%02d"
        self.pathToJogVelocity = "root/ManipulatorControl/hostInJointVelocity"
        self.pathToAxesPosition = "root/AxesControl/axesPositionsActual"
        self.pathToAxesForce = "root/AxesControl/sensorTorquesActual"
        self.pathToEthercat = "root/Ethercat"
        self.pathToIsEngaged = "root/Logic/isAtEngaged"
        self.pathToIsReset = "root/Logic/isAtReset"
        self.pathToHoming = "root/Logic/homing"
        self.pathToHomingEnable = "root/Logic/homing/homingEnabled"
        self.pathToHomingLogicTrigger = "root/Logic/homing/stateToHoming/:actuator%02d/setHardwareSnapshot"
        self.pathToHomingTrigger = "root/AxesControl/actuatorControlLoops/actuatorControlLoop%02d/positionTransformation/transducer/referencing/:fromState/setHardwareSnapshot"
        self.pathToHomingSnapshot = "root/Logic/homing/homingToState/:actuator%02d/hasHardwareSnapshot"
        self.pathToHomingCompleted = "root/Logic/homing/homingToState/:actuator%02d/isHardwareReferenced"
        self.pathToAxesLimiterMode = "root/AxesControl/axesLimiters/Limits%02d/mode"
        self.pathToActuatorJog = "root/AxesControl/actuatorControlLoops/actuatorControlLoop%02d/gotoJogging"
        self.pathToAxes = "root/AxesControl"
        self.pathToJogMode = "root/AxesControl/gotoJogMode"
        self.pathToAxesForceDetectorTooHigh = "root/AxesControl/axesTorqueDetectors/windowDetector%02d/isTooHigh"
        self.pathToAxesForceDetectorTooLow = "root/AxesControl/axesTorqueDetectors/windowDetector%02d/isTooLow"
        self.pathToActuatorForceDetectorTooHigh = "root/AxesControl/actuatorControlLoops/actuatorControlLoop%02d/isTooHigh"
        self.pathToActuatorForceDetectorTooLow = "root/AxesControl/actuatorControlLoops/actuatorControlLoop%02d/isTooLow"
        # States & Modes
        self.states = {"Off": 0, "Idle": 1, "Engaged": 4, "Homing": 7}
        self.modeCommands = {"Off": 0, "Pause": 1, "Manual Joint": 3, "Manual Cart": 4}
        self.stateCommands = {"Off": 0, "Engage": 2, "Homing": 7}
        self.PVAModes = {"PVA_Prelimit": 0, "PVA": 1, "VA": 2, "P": 3, "Off": 4}

        self.gearRatio = 50
        self.ticksPerRevolution = 4096
        # self.axesRange = 2 * pi * 698 / 360 # deg to rad
        self.axesRange = 22  # rad

    def gotoEngage(self, environment):
        print("Engaging Actuator")
        environment.req.setParameter(self.pathToStateCommand, self.stateCommands["Engage"]).get()
        waitFor(environment.req, self.pathToState, self.states["Engaged"], timeout=10)
        environment.req.setParameter(self.pathToModeCommand, self.modeCommands["Manual Joint"]).get()

    def gotoHoming(self, environment):
        print("Go to Homing actuators")
        environment.req.setParameter(self.pathToStateCommand, self.stateCommands["Homing"]).get()

    def gotoOff(self, environment):
        print("Disengaging actuator")
        environment.req.setParameter(self.pathToStateCommand, self.stateCommands["Off"]).get()


def main():
    # initialize the communication with the controller
    mcx = MCXConnection(url=URL,
                        outputfolder=OUTPUTFOLDER,
                        plotfolder=PLOTFOLDER,
                        datafolder=DATAFOLDER)
    if (not mcx.req):
        print("Exiting")
        exit(0)

    system = mcxApp()

    try:
        ### Disable PVA limiter for the specific axis
        set_PVA_mode(mcx, system, Axes=[7, 8, 9, 10], Mode=system.PVAModes["Off"])

        ### Set generators
        set_generators(mcx, system, Axes=[7, 8, 9, 10])
        system.gotoEngage(mcx)

        ### Sterile Adapter
        calibrate_sterile_adapter_homing(mcx, system, Actuators=[7, 8, 9, 10], travelRange=360)

        # ### Roll
        # calibrate_instrument_roll(mcx, system, Axis=9,Actuators=10)
        #
        # ### Set reference for Roll
        # system.gotoHoming(mcx)
        # set_reference(mcx, system, Axis=9,Actuators=[10])
        #
        # ### Disable PVA limiter for the specific axis
        set_PVA_mode(mcx, system, Axes=[7, 8, 9, 10], Mode=system.PVAModes["P"])

        ### Pitch
        # system.gotoEngage(mcx)
        # calibrate_instrument_pitch(mcx, system, Axis=6, Actuators=7)
        #
        # system.gotoHoming(mcx)
        # set_reference(mcx, system, Axis=6, Actuators=7)

    except KeyboardInterrupt:
        print("Interrupted! Exiting...")

    # close communication
    mcx.close()

    # Generate a PDF
    print("Generating PDF")
    timestring = time.strftime("%Y-%m-%d_%H-%M-%S")
    # pdfname = OUTPUTFOLDER + "/%s-output.pdf"%timestring
    # HTML(OUTPUTFOLDER + "/output.html").write_pdf(pdfname)

    # (OPTIONAL) Open the PDF in the browser
    print("Opening Document")
    # webbrowser.open(os.getcwd()+"/"+pdfname)

    exit(0)


if __name__ == '__main__':
    main()
