#!/usr/bin/python3
import math
#
#   Developer : Philippe Piatkiewitz (philippe.piatkiewitz@vectioneer.com)
#   All rights reserved. Copyright (c) 2019 VECTIONEER.
#

import time

import numpy as np
from jinja2 import Template
from motorcortex_tools import *


def set_reference(env, systemData, Axis=9, Actuators=[10]):
    req = env.req
    req.setParameter(systemData.pathToHomingEnable,
                     [False, False, False, False, False, False, False, False, False, False]).get()

    print("Set reference position")
    for Actuator in Actuators:
        req.setParameter(systemData.pathToHomingEnable, True, offset=(Actuator - 1), length=1).get()
        req.overwriteParameter(systemData.pathToHomingTrigger % Actuator, True, force_activate=True).get()
        time.sleep(0.05)
        req.releaseParameter(systemData.pathToHomingTrigger % Actuator).get()

    # Get 0-position
    req.setParameter(systemData.pathToSetpointGenerator % (Axis + 1) + "/input", 0.0).get()
    waitFor(req, systemData.pathToSetpointGenerator % (Axis + 1) + "/done", timeout=5)

    req.setParameter(systemData.pathToHomingEnable,
                     [False, False, False, False, False, False, False, False, False, False]).get()

    print("Homing procedure done")
    return


def calibrate_sterile_adapter_SPG(env, systemData, Actuators=[10], travelRange=10):
    req = env.req
    req.setParameter(systemData.pathToModeCommand, systemData.modeCommands["Manual Joint"]).get()
    print("calibrate sterile adapter position")

    for Actuator in Actuators:
        print("calibrate actuator ", Actuator)
        # First a full rotation always finds the threshold
        req.setParameter(systemData.pathToSetpointGenerator % Actuator + "/input", travelRange * 0.0174593197849).get()
        # waitFor(req, systemData.pathToSetpointGenerator % Actuator + "/done")
        waitFor(req, systemData.pathToForceDetectorTooHigh % Actuator + "/done")
        # check torque of the actuator > threshold

        req.setParameter(systemData.pathToSetpointGenerator % Actuator + "/input",
                         -travelRange * 0.1 * 0.0174593197849).get()
        waitFor(req, systemData.pathToSetpointGenerator % Actuator + "/done")
        req.setParameter(systemData.pathToSetpointGenerator % Actuator + "/input", 0.0).get()
        waitFor(req, systemData.pathToSetpointGenerator % Actuator + "/done")

    req.setParameter(systemData.pathToModeCommand, systemData.modeCommands["Pause"]).get()
    print("calibrate sterile adapter position done")
    return


def calibrate_sterile_adapter_homing(env, systemData, Actuators=[10], travelRange=10):
    req = env.req
    print("Calibrate sterile adapter position")
    req.setParameter(systemData.pathToStateCommand, systemData.stateCommands["Homing"]).get()

    for Actuator in Actuators:
        print("calibrate actuator ", Actuator)
        req.setParameter(systemData.pathToActuator % Actuator + "/gotoJogging", True).get()

    # check torque of the actuator > threshold
    allActuatorsDone = False
    while (not allActuatorsDone):
        allActuatorsDone = True
        for Actuator in Actuators:
            actuatorDone = req.getParameter(systemData.pathToHomingSnapshot % Actuator).get().value[0]
            if (actuatorDone == 1):
                req.setParameter(systemData.pathToActuator % Actuator + "/inputJogVelocity", 0).get()
                req.setParameter(systemData.pathToActuator % Actuator + "/gotoJogging", False).get()
                req.setParameter(systemData.pathToHomingTrigger % Actuator, True).get()
            else:
                req.setParameter(systemData.pathToActuator % Actuator + "/inputJogVelocity", 50).get()
            allActuatorsDone = allActuatorsDone and (actuatorDone == 1)
        time.sleep(0.05)
    print("All references are found")

    print("Check notch opposite of actuator ", Actuator)
    # for Actuator in Actuators:
    # First a full rotation always finds the threshold

    print("calibrate sterile adapter position done")
    return


def set_generators(env, systemData, Axes=[10]):
    req = env.req

    for Axis in Axes:
        # Setup the setpoint generator for the roll axis
        req.setParameter(systemData.pathToSetpointGenerator % (Axis) + "/maxJerk", 100).get()  # jerk limited
        req.setParameter(systemData.pathToSetpointGenerator % (Axis) + "/maxAcc", 20).get()  # acc limited
        req.setParameter(systemData.pathToSetpointGenerator % (Axis) + "/maxVel", 5).get()  # vel limited
        req.setParameter(systemData.pathToSetpointGenerator % (Axis) + "/faderType", 0).get()  # Jerk Limited

    print("setpoint generators initialized")
    return


def set_PVA_mode(env, systemData, Axes=9, Mode=3):
    req = env.req

    for Axis in Axes:
        print("Adjust PVA axes-Limit for axis-%02d" % Axis)
        req.setParameter(systemData.pathToAxesLimiterMode % (Axis), Mode).get()

    return


def calibrate_instrument_roll(env, systemData, Axis=9, Actuators=10):
    # Conflict:
    # Actuators = a range of multiple actuators e.g. 7,8,9
    # Axis is a single axis number. going from 0..x (where actuators start @ 01)
    pathToHomingVelocity = systemData.pathToAxes + "/axesHomingJogVelocitiesInput"
    pathToForce = systemData.pathToAxes + "/sensorTorquesActual"
    pathToForceDetectorTooHigh = systemData.pathToAxes + "/axesTorqueDetectors/windowDetector%02d/isTooHigh"
    pathToForceDetectorTooLow = systemData.pathToAxes + "/axesTorqueDetectors/windowDetector%02d/isTooLow"
    pathToBacklash = systemData.pathToActuator + "/motorBacklashCompensation/positionCorrection"

    pathToActuatorPosition = systemData.pathToActuator % Actuators + "/actuatorPositionActual"
    pathToMotorPosition = systemData.pathToActuator % Actuators + "/motorPositionActual"
    pathToVelocity = systemData.pathToActuator % Actuators + "/actuatorVelocityActualFiltered"
    pathToFrictionForce = systemData.pathToActuator % Actuators + "/feedforwardControl/velFeedForwardForce"

    req = env.req

    # constants
    M_2xPI = 2 * np.pi
    M_Ticks = systemData.gearRatio * systemData.ticksPerRevolution / M_2xPI

    print("Measure Static Torque")
    NumSamples = 20
    sum = 0
    for cnt in range(0, NumSamples):
        sum = sum + req.getParameter(systemData.pathToAxesForce).get().value[0]
        time.sleep(0.05)
    staticForceInMidstroke = sum / NumSamples

    referenceAxesPosition = req.getParameter(systemData.pathToAxesPosition).get().value[Axis]
    print("current axis position : ", referenceAxesPosition)

    print("Move CCW till limit stop, then move to CW limit stop")
    CCWAxesPositionSearch = referenceAxesPosition - (M_2xPI * 4)
    CWAxesPositionSearch = referenceAxesPosition + (M_2xPI * 4)

    print("Starting Datalogger")
    logger = DataLogger(env.url, [systemData.pathToAxesPosition, systemData.pathToAxesForce,
                                  pathToForce, pathToFrictionForce,
                                  pathToForceDetectorTooHigh % (Axis + 1), pathToForceDetectorTooLow % (Axis + 1)],
                        certificate=env.certificate, divider=10)
    logger.openFileAndWriteHeader(env.outputfolder + env.datafolder + "homing_trail%02d.csv" % (Axis + 1), False)
    logger.start()

    # Reset backlash before homing
    req.setParameter(pathToBacklash % Actuators, 0.0).get()

    # Go to CCW limit
    req.setParameter(systemData.pathToSetpointGenerator % (Axis + 1) + "/input", CCWAxesPositionSearch).get()
    waitFor(req, pathToForceDetectorTooLow % (Axis + 1), timeout=10)
    homingLimitPositionCCW = req.getParameter(systemData.pathToAxesPosition).get().value[Axis]
    homingLimitPositionCCW_Ticks = homingLimitPositionCCW * M_Ticks
    print("homing Limit Position CCW: ", homingLimitPositionCCW)

    # Move to CW limit
    req.setParameter(systemData.pathToSetpointGenerator % (Axis + 1) + "/input", CWAxesPositionSearch).get()
    waitFor(req, pathToForceDetectorTooHigh % (Axis + 1), timeout=10)
    homingLimitPositionCW = req.getParameter(systemData.pathToAxesPosition).get().value[Axis]
    homingLimitPositionCW_Ticks = homingLimitPositionCW * M_Ticks
    print("homing Limit Position CW: ", homingLimitPositionCW)

    homingLimitPositionRange = homingLimitPositionCCW - homingLimitPositionCW  # Radians
    homingLimitPositionRange_Rotations = homingLimitPositionRange / M_2xPI  # Rotations
    homingLimitPositionRange_Ticks = np.abs(homingLimitPositionCCW_Ticks - homingLimitPositionCW_Ticks)  # Ticks
    homingLimitPositionMiddle = homingLimitPositionCW + (homingLimitPositionRange / 2)
    print("homing center at : ", homingLimitPositionMiddle)
    print("range : %3.3f [Rotations]" % homingLimitPositionRange_Rotations)

    print("Backlash compensation")
    print("range : %6.0d [Ticks] found, %6.0d [Ticks] mechanical expected" % (
        homingLimitPositionRange_Ticks, systemData.axesRange * M_Ticks))
    backlashCompensation = (homingLimitPositionRange_Ticks - systemData.axesRange * M_Ticks) / 2
    print("backlash : %6.0d [Ticks] found" % backlashCompensation)
    req.setParameter(pathToBacklash % Actuators, -backlashCompensation).get()

    print("Go to center position reference")
    req.setParameter(systemData.pathToSetpointGenerator % (Axis + 1) + "/input", homingLimitPositionMiddle).get()
    waitFor(req, systemData.pathToSetpointGenerator % (Axis + 1) + "/done", timeout=10)

    print("Stopping Datalogger & write to file")
    logger.stop()
    logger.close()

    return
