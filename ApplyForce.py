import serial
import time
import numpy as np
from decimal import *
import matplotlib.pyplot as plt

# changes the decimal precision to 3
from simple_pid import PID

# getcontext().prec = 3

# global variables
serialPort = serial.Serial(port="COM6", baudrate=19200, bytesize=8, timeout=10, stopbits=serial.STOPBITS_ONE)


def readSerialPort():
    serialPort.flushInput()
    serialPort.write(b"D")
    STX = serialPort.read(1)
    # print(STX)
    if STX != b'\x02':
        serialPort.write(b"iSI")  # Stop the machine
        raise Exception('Expected x02 not received/acknowledged, instead received {}'.format(STX))

    line = serialPort.readline()
    return line.decode("utf-8").strip("\r\n")


def TX(command):
    # print("TX:" + command)
    serialPort.flushInput()  # clears the input buffer, response you get from "D"
    serialPort.write((command + "\r\n").encode())
    ACK = serialPort.read(1)
    # print(ACK)
    if ACK != b'\x06':
        serialPort.write(b"iSI")  # Stop the machine
        raise Exception('Expected x06 not received/acknowledged, instead received {}'.format(ACK))


def getMachineState(state):
    """
    getMachineState returns the force or relative position or absolute position of the device
    :param state: string of input 'force' or 'relative position' or 'absolute position'
    """

    position = 0.0
    resp = readSerialPort()
    print((resp.split(",")))
    if state == "force":
        position = float(resp.split(",")[0])
    if state == "relative position":
        position = float(resp.split(",")[1])
    if state == "absolute position":
        position = float(resp.split(",")[2])
    return position


# nextDifference refers to the halfway point between current and desired force
def newDifference(desiredForce):
    return float(desiredForce - getMachineState("force"))


# Using compression mode settings if speed is -ve go down and vice versa
lastDir = None  # True - UP, False - Down


def GO(speed):
    global lastDir
    speed = max(-500, min(500, speed))  # max speed either direction is 500 mm/min
    # TX("iSI")
    # print("Speed = %.3f mm/min" % speed)
    if abs(speed) < 0.001:
        TX("iSI")
    elif speed < 0:
        if lastDir:
            TX("iSI")
            lastDir = False
        if lastDir is None:
            lastDir = False
        TX("s" + str(abs(int(speed * 1000))).zfill(7) + "S")
        TX("iVI")
    else:
        if not lastDir:
            TX("iSI")
            lastDir = True
        if lastDir is None:
            lastDir = True
        TX("r" + str(abs(int(speed * 1000))).zfill(7) + "R")
        TX("iTI")


def Stop():
    TX("iSI")


def gotoPosition(startposition, tolerance=1):  # mm
    posDif = startposition - getMachineState("absolute position")
    # print("start pos = %.3f mm" % getMachineState("absolute position"))
    print("dif = %.3f mm" % posDif)
    prevDir = -np.sign(posDif)
    while abs(posDif) > tolerance:
        posDif = startposition - getMachineState("absolute position")
        if prevDir == 1 and np.sign(posDif) == -1:
            GO(-500)
            prevDir = np.sign(posDif)
        elif prevDir == -1 and np.sign(posDif) == 1:
            GO(500)
            prevDir = np.sign(posDif)
        time.sleep(0.1)
    Stop()
    print("final pos = %.3f mm" % getMachineState("absolute position"))


a1 = 5
b1 = 5
c1 = 0.00

a = 20
b = 40
c = 0.00


def gotoForce(desiredForce, tolerance=0.002, timeout=30, stableTime=3):
    try:
        timeoutTicks = time.time() + timeout  # seconds
        stableTicks = time.time() + stableTime  # seconds
        x = []
        y = []
        start = time.time()
        f1 = getMachineState("force")
        diff1 = desiredForce - f1
        lastDir2 = (diff1 > 0)
        while True:
            sams = 3
            avgForce = 0
            for i in range(sams):
                avgForce += getMachineState("force")
                # time.sleep(0.01)
            avgForce /= sams
            difference = desiredForce - avgForce
            x.append(time.time() - start)
            y.append(avgForce)
            absdifference = abs(difference)

            print("Diff  = %.3f N" % difference)

            # if the target has been reached
            if absdifference <= tolerance:
                # TX("iSI")  # stop
                if time.time() > stableTicks:  # if stability has occurred
                    print("Stable")
                    break
            else:  # reset the stable timer
                stableTicks = time.time() + stableTime

            speed = ((difference * absdifference) * a) + (difference * b) + np.sign(difference) * c
            if lastDir2 != (difference > 0):  # Up is True, if its changing directions
                if absdifference > tolerance * 0.5:  # if the difference is going out of tolerance
                    GO(speed)
                else:
                    Stop()
            else:
                GO(speed)

            # if the timeout has exceeded break from the loop
            if time.time() > timeoutTicks:
                print("Timeout occurred")
                break
        Stop()

        print("Final Force = %.3f N" % getMachineState("force"))
    except:
        Stop()
    # plotting the points
    plt.plot(x, y)

    # naming the x axis
    plt.xlabel('time (s)')
    # naming the y axis
    plt.ylabel('force (N)')

    # giving a title to my graph
    plt.title('Target %f'% desiredForce)

    # function to show the plot
    plt.show() # comment out or else it stops after every gotoForce


# Configure Machine to initial conditions- compression mode
TX("z01000000500000c001020Z")

gotoPosition(210)
# wait for stable
time.sleep(5)
# Zero the Force
TX("&")
# Reset the Change in distance to 0
TX("g0G")

time.sleep(1)
print("Force = %.3f N" % getMachineState("force"))

# print(getMachineState("absolute position"))
targetForce = -5.0
gotoForce(targetForce)
# time.sleep(60)
print("Force = %.3f N" % getMachineState("force"))
print("Position = %.3f mm" % getMachineState("absolute position"))
targetForce = -20
gotoForce(targetForce)
# time.sleep(60)
print("Force = %.3f N" % getMachineState("force"))
print("Position = %.3f mm" % getMachineState("absolute position"))
targetForce = -6
gotoForce(targetForce)
# time.sleep(60)
print("Force = %.3f N" % getMachineState("force"))
print("Position = %.3f mm" % getMachineState("absolute position"))
targetForce = -1.0
gotoForce(targetForce)
# time.sleep(60)
print("Force = %.3f N" % getMachineState("force"))
print("Position = %.3f mm" % getMachineState("absolute position"))
targetForce = -0.01
gotoForce(targetForce)
# time.sleep(60)
print("Force = %.3f N" % getMachineState("force"))
print("Position = %.3f mm" % getMachineState("absolute position"))

gotoPosition(210)

'''
timeoutTicks = time.time() + 60  # seconds
while True:
    time.sleep(0.1)

    print("Force = %.3f N" % getMachineState("force"))
    # if the timeout has exceeded break from the loop
    if time.time() > timeoutTicks:
        print("Timeout occurred")
        break
'''
