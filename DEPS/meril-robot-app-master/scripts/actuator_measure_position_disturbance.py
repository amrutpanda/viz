#!/usr/bin/python3

#
#   Developer : Philippe Piatkiewitz (philippe.piatkiewitz@vectioneer.com)
#   All rights reserved. Copyright (c) 2019 VECTIONEER.
#

import matplotlib.pyplot as plt
import numpy as np
import time
from jinja2 import Template
from math import ceil
from motorcortex_tools import *


def measureActuatorPositionDisturbance(env, systemData, ID=1,
                                       amplitude=1.0, offset=0.0, velocity=0.05,
                                       plotForceRange=60.0, dwellTime=0.0, centerPlotAtForce=None, title=None):
    template = Template("""
    <h2>{{title}}</h2>
    <p>Actuator position disturbance is measured over the total stroke and is measured at very
     slow speeds to minimize dynamic forces acting on the actuator. </p>
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
    pathToMotorPosition = systemData.pathToActuator % ID + "/motorPositionActual"
    pathToVelocity = systemData.pathToActuator % ID + "/actuatorVelocityActualFiltered"
    pathToForce = systemData.pathToActuator % ID + "/actuatorTorqueActualFiltered"
    pathToFrictionForce = systemData.pathToActuator % ID + "/feedforwardControl/velFeedForwardForce"
    pathToModelForce = systemData.pathToActuator % ID + "/inputTorqueFilter/output"

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
    # req.setParameter(systemData.pathToSignalGenerator%ID+"/signalType", 2).get()    # triangle
    # req.setParameter(systemData.pathToSignalGenerator % ID + "/signalType", 4).get()  # jerk limited
    # req.setParameter(systemData.pathToSignalGenerator % ID + "/amplitude", -amplitude).get()
    # req.setParameter(systemData.pathToSignalGenerator % ID + "/offset", offset).get()
    # req.setParameter(systemData.pathToSignalGenerator % ID + "/frequency", frequencyHz * 2 * np.pi).get()
    # req.setParameter(systemData.pathToSignalGenerator % ID + "/dwellTime", dwellTime).get()
    # req.overwriteParameter(systemData.pathToSignalGenerator % ID + "/enable", True, force_activate=True,
    #                        type_name=None).get()
    # time.sleep(req.getParameter(systemData.pathToSignalGenerator % ID + "/newSettingFadeTime").get().value[0])
    req.setParameter(systemData.pathToSetpointGenerator % ID + "/maxJerk", 50).get()  # jerk limited
    req.setParameter(systemData.pathToSetpointGenerator % ID + "/maxAcc", 5).get()  # acc limited
    req.setParameter(systemData.pathToSetpointGenerator % ID + "/maxVel", 0.5).get()  # vel limited
    req.setParameter(systemData.pathToSetpointGenerator % ID + "/faderType", 1).get()  # constant velocity

    print("Get 0- position reference")
    # Get 0-position
    req.setParameter(systemData.pathToSetpointGenerator % ID + "/input", 0.0).get()
    waitFor(req, systemData.pathToSetpointGenerator % ID + "/done", timeout=10)
    offsetPosition = req.getParameter(pathToMotorPosition).get()
    print("start position: ", offsetPosition)

    print("move to start position")
    # Pre-position
    req.setParameter(systemData.pathToSetpointGenerator % ID + "/input", -amplitude).get()
    waitFor(req, systemData.pathToSetpointGenerator % ID + "/done", timeout=10)

    # measurement
    print("Starting Datalogger")
    logger = DataLogger(env.url,
                        [pathToPosition, pathToMotorPosition, pathToForce, pathToFrictionForce, pathToModelForce,
                         pathToVelocity],
                        certificate=env.certificate, divider=1)
    logger.openFileAndWriteHeader(env.outputfolder + env.datafolder + "positionDisturbance%02d.csv" % ID, False)
    logger.start()

    # Move 1
    req.setParameter(systemData.pathToSetpointGenerator % ID + "/maxVel", velocity).get()  # vel limited
    req.setParameter(systemData.pathToSetpointGenerator % ID + "/input", 2.0 * amplitude).get()

    # Wait 
    print("Waiting for measurement to complete")
    waitFor(req, systemData.pathToSetpointGenerator % ID + "/done", timeout=(2 * amplitude / velocity) + 2.0)

    # # reverse direction
    # req.setParameter(systemData.pathToSetpointGenerator % ID + "/input", -2.0*amplitude).get()
    # waitFor(req, systemData.pathToSetpointGenerator % ID + "/done", timeout=(2 * amplitude / velocity) + 2.0)
    # time.sleep((1 / frequencyHz) + dwellTime * 2)

    print("Stopping Datalogger & write to file")
    logger.stop()
    logger.close()

    print("Stopping Signal Generator")
    req.setParameter(systemData.pathToSetpointGenerator % ID + "/maxVel", 0.5).get()  # vel limited
    req.setParameter(systemData.pathToSetpointGenerator % ID + "/input", 0).get()
    waitFor(req, systemData.pathToSetpointGenerator % ID + "/done", timeout=(amplitude / velocity) + 2.0)
    # req.releaseParameter(systemData.pathToSignalGenerator % ID + "/enable").get()
    # waitFor(req, systemData.pathToSignalGenerator % ID + "/enableIsOff", timeout=10)
    # req.setParameter(systemData.pathToSignalGenerator % ID + "/signalType", 0).get()

    print("Done")
    # generate the position-force plot
    x = np.array(logger.traces[pathToPosition]["y"][0]).transpose()
    x_motor = np.array(logger.traces[pathToMotorPosition]["y"][0]).transpose()
    y = np.array(logger.traces[pathToForce]["y"][0]).transpose()
    y_model = np.array(logger.traces[pathToModelForce]["y"][0]).transpose()
    y_friction = np.array(logger.traces[pathToFrictionForce]["y"][0]).transpose()

    meany = np.mean(y)
    minx = -2  # np.min(x)
    maxx = 2  # np.max(x)
    xrange = maxx - minx
    midx = minx + xrange * 0.5
    delta = 0.01
    idxaroundmin = np.where((x > midx - delta) & (x < midx + delta))
    yaroundmid = y[idxaroundmin]
    friction_at_midstroke = np.max(yaroundmid) - np.min(yaroundmid)

    fig = plt.figure()
    plt.plot(x, y)
    plt.plot(x, y_model)
    plt.plot(x, y_friction)
    plt.xlabel("position (m)"), plt.ylabel("torque (Nm)")
    # plt.legend(["y", "y_model", "y_friction"])
    ax = plt.gca()
    ax.set_ylim([centerPlotAt - 0.5 * plotForceRange, centerPlotAt + 0.5 * plotForceRange])
    plt.title("position force plot")
    plt.savefig(env.outputfolder + env.plotfolder + "positionForcePlot%02d.png" % ID)

    # get single direction:
    print("number of points", x.shape)
    # t = ceil(x.shape[0] / 4)
    t = ceil(x.shape[0] / 2)
    c = 50
    stepsize = 50

    # Wn x sample frequency is the filter frequency.
    b, a = signal.butter(6, 0.01)
    y_comp = signal.filtfilt(b, a, y - y_model - y_friction)

    # forward path
    x1 = x[c:t - c:stepsize]
    y1 = y_comp[c:t - c:stepsize] - np.mean(y_comp[c:t - c:stepsize])

    # reverse path
    x2 = x[t + c:2 * t - c:stepsize]
    y2 = y_comp[t + c:2 * t - c:stepsize] - np.mean(y_comp[t + c:2 * t - c:stepsize])

    x1_corr = x[c:t - c:stepsize] + (4 * velocity / 1000)

    fig = plt.figure()
    plt.plot(x1, y1, color='b', linewidth='0.1'), plt.xlabel("position (m)"), plt.ylabel("torque (Nm)")
    plt.plot(x2, y2, color='g', linewidth='0.1')
    plt.plot(x1_corr, y1, color='k', linewidth='0.1')
    ax = plt.gca()
    ax.set_ylim([-0.1 * plotForceRange, 0.1 * plotForceRange])
    plt.title("position disturbance")
    plt.savefig(env.outputfolder + env.plotfolder + "positionDisturbance%02d.png" % ID)
    ax.set_xlim([3, 3.5])
    plt.savefig(env.outputfolder + env.plotfolder + "positionDisturbance%03d_zoom.png" % ID)

    # print('write table values')
    # req.setParameter(systemData.pathToActuator % ID + "/feedforwardControl/positionLookup/x", x1_corr.tolist()).get()
    # req.setParameter(systemData.pathToActuator % ID + "/feedforwardControl/positionLookup/y", y1.tolist()).get()
    # req.setParameter(systemData.pathToActuator % ID + "/feedforwardControl/positionLookup/numPoints", y1.shape[0]).get()
    # req.setParameter(systemData.pathToActuator % ID + "/feedforwardControl/positionLookup/useSortedData", 1).get()
    # req.setParameter(systemData.pathToActuator % ID + "/feedforwardControl/positionLookup/gain", 0.9).get()

    titlestr = ", actuator #"
    if (title):
        titlestr = " - %s" % title
    output = template.render(title="Actuator Position Disturbance" + titlestr + str(ID),
                             ID=ID,
                             datetime=time.strftime("%Y-%m-%d %H:%M:%S"),
                             friction_at_midstroke=friction_at_midstroke,
                             fstat_at_midstroke=staticForceInMidstroke,
                             amplitude=amplitude,
                             offset=offset,
                             frequencyHz=velocity,
                             plot=env.plotfolder + "positionDisturbance%03d.png" % ID)
    #
    return output
