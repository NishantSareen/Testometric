import serial
import time

# global variables
serialPort = serial.Serial(port="COM6", baudrate=19200, bytesize=8, timeout=10, stopbits=serial.STOPBITS_ONE)


def readSerialPort():
    serialPort.flush()
    serialPort.write(b"D")
    print(serialPort.readline())


# Acknowledge that command is received
def TX(command):
    print("TX:" + command)
    serialPort.flushInput()  # clears the input buffer, response you get from "D"
    serialPort.write((command + "\r\n").encode())
    ACK = serialPort.read(1)
    print(ACK)
    if ACK != b'\x06':
        raise Exception('Expected x06 not received/acknowledged, instead received {}'.format(ACK))


# Sent a request and received a response from the machine
def TXRX(command):
    # print("TXRX:" + command)
    serialPort.flushInput()
    serialPort.write((command + "\r\n").encode())
    # STX = serialPort.read(1)
    #  print(b"RX1:" + STX)
    line = serialPort.readline()
    #  print(b"RX:" + line)
    return line.decode("utf-8").strip("\r\n\x02")


TX("z01000000500000c001020Z")  # Configure Machine to initial conditions- compression mode

resp = TXRX("D")
respSplit = resp.split(",")
print(respSplit)
force = float(respSplit[0])
relPos = float(respSplit[1])
absPos = float(respSplit[2])
print(force)
print(relPos)
print(absPos)


# Using compression mode settings if speed is -ve go down and vice versa
def GO(speed):
    TX("iSI")
    if speed < 0:
        TX("s" + str(abs(speed * 1000)).zfill(7) + "S")
        TX("iVI")
    else:
        TX("r" + str(abs(speed * 1000)).zfill(7) + "R")
        TX("iTI")


GO(500)

'''
while True:
    GO(500)
    time.sleep(2)
    # print(TXRX("D"))
    GO(-500)
    time.sleep(2)
    # print(TXRX("D"))
'''

