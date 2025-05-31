#!/usr/bin/python3

#
#   Developer : Roel van Mil (roel.vanmil@vectioneer.com)
#   All rights reserved. Copyright (c) 2019-2024 VECTIONEER.
#
#
# URL = "wss://192.168.2.101:5568:5567"
URL = "wss://192.168.2.100:5568:5567"

# import the motorcortex library
import motorcortex
import numpy as np
from scipy import linalg

# Create a parameter tree object
parameter_tree = motorcortex.ParameterTree()
# Open request and subscribe connection
try:
    req, sub = motorcortex.connect(URL, motorcortex.MessageTypes(), parameter_tree,
                                   certificate="mcx.cert.crt", timeout_ms=1000,
                                   login="root", password="secret")
    tree = parameter_tree.getParameterTree()
    # print(f"Parameters: {tree}")

except RuntimeError as err:
    print(err)


def readMatrix():
    # Append you tests here:
    numActuators = req.getParameter(pathToNumberOfActuator).get().value[0]
    numAxes = req.getParameter(pathToNumberOfAxes).get().value[0]

    print("number of actuators %01d" % numActuators)
    print("number of axes %01d" % numAxes)

    # generate transformation matrices
    actuatorsToAxesTransform = np.array(req.getParameter(pathToActuatorsToAxesTransform).get().value).reshape(
        numActuators, numAxes)

    axesToActuatorsTransform = np.array(req.getParameter(pathToAxesToActuatorsTransform).get().value).reshape(numAxes,
                                                                                                              numActuators)

    # axesToActuatorInverse = linalg.inv(axesToActuatorsTransform)

    # Print the results
    print("\nactuatorsToAxesTransform:\n", actuatorsToAxesTransform)

    print("\naxesToActuatorsTransform:\n", axesToActuatorsTransform)

    print("\naxesToActuatorsInverse:\n", axesToActuatorInverse)


def writeMatrix():
    # Make a matrix to modify and write to the parameter tree
    numActuators = req.getParameter(pathToNumberOfActuator).get().value[0]
    numAxes = req.getParameter(pathToNumberOfAxes).get().value[0]

    axesToActuatorsTransform = np.array(req.getParameter(pathToAxesToActuatorsTransform).get().value).reshape(
        numActuators, numAxes)
    newActuatorsToAxesTransform = linalg.inv(axesToActuatorsTransform);
    print(newActuatorsToAxesTransform)

    tmp = np.array(newActuatorsToAxesTransform).reshape(numAxes * numActuators).tolist()

    req.setParameter(pathToActuatorsToAxesTransform, tmp).get()  # this line requires a float?

    getNewActuatorsToAxesTransform = np.array(req.getParameter(pathToActuatorsToAxesTransform).get().value).reshape(
        numActuators, numAxes)
    print("\ngetNewActuatorsToAxesTransform:\n", getNewActuatorsToAxesTransform)


# Main Program
pathToActuatorsToAxesTransform = "root/AxesControl/actuatorsToAxesTransform"
pathToAxesToActuatorsTransform = "root/AxesControl/axesToActuatorsTransform"
pathToNumberOfActuator = "root/AppInfo/features/axes/numberOfActuators"
pathToNumberOfAxes = "root/AppInfo/features/axes/numberOfAxes"

readMatrix()
# writeMatrix()

sub.close()
req.close()
