#!/usr/bin/python3

#
#   Developer : Philippe Piatkiewitz (philippe.piatkiewitz@vectioneer.com)
#   All rights reserved. Copyright (c) 2019 VECTIONEER.
#

import numpy as np
import time
from jinja2 import Template
from motorcortex_tools import *


def measureActuatorFriction(env, systemData, ID=1,
                            amplitude=0.1, offset=0.0, frequencyHz=0.025,
                            plotForceRange=10.0, centerPlotAtForce=None, title=None):
    template = Template("""
    <h2>{{title}}</h2>
    <p>Actuator friction is measured over the total stroke and is measured at different
     speeds to see the effects of sliding mode (slow speed) and viscous friction (high speed). 
     It is important to keep jerk and accelerations low to prevent system dynamics to dominate. </p>
    <table>
        <tr><th>Test Conditions</th></tr>
        <tr><td>Actuator (ID)</td><td>{{ID}}</td></tr>
        <tr><td>Date & Time</td><td>{{datetime}}</td></tr>
        <tr><td>amplitude</td><td class="numeric">{{amplitude}} m</td></tr>
        <tr><td>offset</td><td class="numeric">{{offset}} m</td></tr>
        <tr><td>frequency</td><td class="numeric">{{frequencyHz}} Hz</td></tr>
        <tr><th>Measurement</th></tr>
        <tr><td>Static Torque at midstroke</td><td class="numeric">{{'%0.3f' % fstat_at_midstroke}} Nm</td></tr>
        <tr><td>Friction at midstroke</td><td class="numeric">{{'%0.3f' % friction_at_midstroke}} Nm</td></tr>
    </table>
    <p>
    <img src="{{plot}}">
    </p>
    <p>
    <img src="{{plotV}}">
    </p>
    """)

    pathToPosition = systemData.pathToActuator % ID + "/actuatorPositionActual"
    pathToVelocity = systemData.pathToActuator % ID + "/actuatorVelocityActualFiltered"
    pathToForce = systemData.pathToActuator % ID + "/actuatorTorqueActual"
    pathToFrictionForce = systemData.pathToActuator % ID + "/feedforwardControl/velFeedForwardForce"

    req = env.req
    print("Measure Static Torque")
    NumSamples = 20
    sum = 0
    for cnt in range(0, NumSamples):
        sum = sum + req.getParameter(pathToForce).get().value[0]
        time.sleep(0.05)
    staticForceInMidstroke = sum / NumSamples
    if centerPlotAtForce:
        centerPlotAt = centerPlotAtForce
    else:
        centerPlotAt = staticForceInMidstroke

    print("Start Motion")
    # Set the signal type
    # req.setParameter(systemData.pathToSignalGenerator % ID + "/signalType", 2).get()  # triangle
    req.setParameter(systemData.pathToSignalGenerator % ID + "/signalType", 4).get()  # jerk limited
    req.setParameter(systemData.pathToSignalGenerator % ID + "/amplitude", -amplitude).get()
    req.setParameter(systemData.pathToSignalGenerator % ID + "/offset", offset).get()
    req.setParameter(systemData.pathToSignalGenerator % ID + "/frequency", frequencyHz * 2 * np.pi).get()
    req.setParameter(systemData.pathToSignalGenerator % ID + "/dwellTime", 0).get()
    req.overwriteParameter(systemData.pathToSignalGenerator % ID + "/enable", True, force_activate=True,
                           type_name=None).get()
    time.sleep(req.getParameter(systemData.pathToSignalGenerator % ID + "/newSettingFadeTime").get().value[0])

    waitFor(req, systemData.pathToSignalGenerator % ID + "/enableIsOn", timeout=10)

    print("Starting Datalogger")
    logger = DataLogger(env.url, [pathToPosition, pathToForce, pathToVelocity, pathToFrictionForce],
                        certificate=env.certificate, divider=10)
    logger.openFileAndWriteHeader(env.outputfolder + env.datafolder + "friction%02d.csv" % ID, False)
    logger.start()

    # Wait
    print("Waiting for measurement to complete")
    time.sleep(1 / frequencyHz)

    print("Stopping Datalogger")
    logger.stop()
    logger.close()

    print("Stopping Signal Generator")
    req.releaseParameter(systemData.pathToSignalGenerator % ID + "/enable").get()
    waitFor(req, systemData.pathToSignalGenerator % ID + "/enableIsOff", timeout=10)
    req.setParameter(systemData.pathToSignalGenerator % ID + "/signalType", 0).get()

    print("Done")

    output = []
    return output
