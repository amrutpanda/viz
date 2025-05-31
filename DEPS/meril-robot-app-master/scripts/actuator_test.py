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

# Tests
from actuator_measure_position_disturbance import measureActuatorPositionDisturbance
from actuator_measure_friction import measureActuatorFriction
from FitPositionDisturbanceData import fitActuatorPositionDisturbance

# from weasyprint import HTML, CSS

try:
    plt.style.use('templates/styles/plotstyle.mplstyle')
    print("Loaded plotsyle")
except:
    print("Could not load plotstyle")

URL = "wss://192.168.2.100:5568:5567"
# URL = "wss://192.168.2.101:5568:5567"

TEMPLATESFOLDER = "templates"
OUTPUTFOLDER = "results/"
PLOTFOLDER = "plots/"
DATAFOLDER = "data/"
if not os.path.exists(OUTPUTFOLDER + PLOTFOLDER):
    os.makedirs(OUTPUTFOLDER + PLOTFOLDER)

if not os.path.exists(OUTPUTFOLDER + DATAFOLDER):
    os.makedirs(OUTPUTFOLDER + DATAFOLDER)


class TestEnvironment:
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


class SystemDataManipulatorApp():
    def __init__(self):
        # references to common signals
        self.pathToModeCommand = "root/Logic/modeCommand"
        self.pathToStateCommand = "root/Logic/stateCommand"
        self.pathToState = "root/Logic/state"
        self.pathToActuator = "root/AxesControl/actuatorControlLoops/actuatorControlLoop%02d"
        self.pathToSignalGenerator = "root/ManipulatorControl/jointReferenceGenerator/signalGenerator%02d"
        self.pathToSetpointGenerator = "root/SetpointGenerators/SetpointGenerator%02d"
        self.pathToEthercat = "root/Ethercat"
        self.pathToIsEngaged = "root/Logic/isAtEngaged"
        self.pathToIsReset = "root/Logic/isAtReset"
        self.pathToJogMode = "root/MachineControl/gotoJogMode"

    # requirements
    self.maxFriction = 2.0  # kN
    self.states = {"Off": 0, "Engaged": 4}
    self.modeCommands = {"Off": 0, "Pause": 1, "Manual Joint": 3, "Manual Cart": 4}
    self.stateCommands = {"Off": 0, "Engage": 2}
    # Velocity Accelration Jerk test
    self.vaj_criteria = {"velocity": 0.1, "acceleration": 1, "jerk": 10}


def gotoEngage(self, environment):
    print("Engaging Actuator")
    environment.req.setParameter(self.pathToStateCommand, self.stateCommands["Engage"]).get()
    waitFor(environment.req, self.pathToState, self.states["Engaged"], timeout=10)
    environment.req.setParameter(self.pathToModeCommand, self.modeCommands["Manual Joint"]).get()


def gotoOff(self, environment):
    print("Disengaging actuator")
    environment.req.setParameter(self.pathToStateCommand, self.stateCommands["Off"]).get()


def main():
    # initialize the communication with the controller
    testEnv = TestEnvironment(url=URL,
                              outputfolder=OUTPUTFOLDER,
                              plotfolder=PLOTFOLDER,
                              datafolder=DATAFOLDER)
    if (not testEnv.req):
        print("Exiting")
        exit(0)

    system = SystemDataManipulatorApp()

    try:
        # send the system to engaged before starting the tests
        system.gotoEngage(testEnv)

        results = []
        ## Append you tests here:
        # #### Friction Tests:
        # results.append(measureActuatorFriction(testEnv, system, ID=1,  amplitude=0.2, offset = 0.0, frequencyHz=0.1,
        #                          plotForceRange=10.0, centerPlotAtForce=None, title=None))
        # # #### Position Disturbance Tests:
        # results.append(measureActuatorPositionDisturbance(testEnv, system, ID=1,  amplitude = 1.57,  offset = 0,
        #                                                   velocity = 0.1, plotForceRange = 60, dwellTime = 0))

        fitActuatorPositionDisturbance(testEnv, system)

    except KeyboardInterrupt:
        print("Interrupted! Exiting...")

    # Stop and reset everything
    system.gotoOff(testEnv)
    # close communication
    testEnv.close()

    # Render all results
    # file_loader = FileSystemLoader(TEMPLATESFOLDER)
    # env = Environment(loader=file_loader)
    # template = env.get_template("base.html")
    # output = template.render(tests = results)
    # fd = open(OUTPUTFOLDER + "/output.html","w")
    # fd.write(output)
    # fd.close()

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
