# 19 July 2014
# 08 Dec 2016 - updated for Python3

# in case any of this upsets Python purists it has been converted from an equivalent JRuby program

# this is designed to work with ... ArduinoPC2.ino ...

# the purpose of this program and the associated Arduino program is to demonstrate a system for sending
#   and receiving data between a PC and an Arduino.

# The key functions are:
#    sendToArduino(str) which sends the given string to the Arduino. The string may
#                       contain characters with any of the values 0 to 255
#
#    recvFromArduino()  which returns an array.
#                         The first element contains the number of bytes that the Arduino said it included in
#                             message. This can be used to check that the full message was received.
#                         The second element contains the message as a string


# the overall process followed by the demo program is as follows
#   open the serial connection to the Arduino - which causes the Arduino to reset
#   wait for a message from the Arduino to give it time to reset
#   loop through a series of test messages
#      send a message and display it on the PC screen
#      wait for a reply and display it on the PC

# to facilitate debugging the Arduino code this program interprets any message from the Arduino
#    with the message length set to 0 as a debug message which is displayed on the PC screen

# the message to be sent to the Arduino starts with < and ends with >
#    the message content comprises a string, an integer and a float
#    the numbers are sent as their ascii equivalents
#    for example <LED1,200,0.2>
#    this means set the flash interval for LED1 to 200 millisecs
#      and move the servo to 20% of its range

# receiving a message from the Arduino involves
#    waiting until the startMarker is detected
#    saving all subsequent bytes until the end marker is detected

# NOTES
#       this program does not include any timeouts to deal with delays in communication
#
#       for simplicity the program does NOT search for the comm port - the user must modify the
#         code to include the correct reference.
#         search for the lines
#               serPort = "/dev/ttyS80"
#               baudRate = 9600
#               ser = serial.Serial(serPort, baudRate)
#


#=====================================

#  Function Definitions

#=====================================

def sendToArduino(sendStr):
    arduino.write(sendStr.encode('utf-8'))  # change for Python3
    print(len(sendStr.encode('utf-8')))

#======================================


def recvFromArduino():
    global startMarker, endMarker

    ck = ""
    x = "z"  # any value that is not an end- or startMarker
    byteCount = -1  # to allow for the fact that the last increment will be one too many

    # wait for the start character
    while ord(x) != startMarker:
        x = arduino.read()
        if len(x) == 0:
            x = "z"

    # save data until the end marker is found
    while ord(x) != endMarker:
        if ord(x) != startMarker:
            ck = ck + x.decode("utf-8")  # change for Python3
            byteCount += 1
        x = arduino.read()

    return(ck)


#============================

def waitForArduino():

    # wait until the Arduino sends 'Arduino Ready' - allows time for Arduino reset
    # it also ensures that any bytes left over from a previous message are discarded

    global startMarker, endMarker

    msg = ""
    while msg.find("Arduino is ready") == -1:

        while arduino.inWaiting() == 0:
            pass

        msg = recvFromArduino()

        print (msg)  # python3 requires parenthesis
        print ()

#======================================


def runTest(td):
    numLoops = len(td)
    waitingForReply = False
    sendToArduino("[")
    if waitingForReply == False:
        n = 0
        while n < numLoops:
            teststr = td[n]

            sendToArduino(teststr)
            print ("Sent from PC -- LOOP NUM " + str(n) + " TEST STR " + teststr)
            n += 1
        sendToArduino("]")
        waitingForReply = True

    if waitingForReply == True:

        while arduino.inWaiting() == 0:
            pass

        dataRecvd = recvFromArduino()
        print ("Reply Received  " + dataRecvd)
        waitingForReply = False

        print ("===========")

    # time.sleep(5)


def waitForFinish():
    while arduino.inWaiting() == 0:
        pass

    dataRecvd = recvFromArduino()
    print ("Reply Received  " + dataRecvd)
    # waitingForFinishSign = False

    print ("===========")

#======================================

# THE DEMO PROGRAM STARTS HERE

#======================================


import serial
import time

print ()
print ()


# NOTE the user must ensure that the serial port and baudrate are correct
# serPort = "/dev/ttyS80"
serPort = "/dev/ttyACM0"
# serPort = "/dev/ttyACM1"
baudRate = 115200
try:
    arduino = serial.Serial(serPort, baudRate, timeout=1)
except:
    print("Please check the port!")
print ("Serial port " + serPort + " opened  Baudrate " + str(baudRate))


startMarker = 60
endMarker = 62


waitForArduino()


testData = []
testData.append("<100000,16383>")
testData.append("<200000,16383>")
testData.append("<300000,16383>")
testData.append("<400000,16383>")
testData.append("<100000,16383>")
testData.append("<200000,16383>")
testData.append("<300000,16383>")
testData.append("<400000,16383>")
# testData.append("<4,500000000,16383>")
# testData.append("<5,600000000,16383>")
# testData.append("<6,700000000,16383>")
# testData.append("<LED1,800,0.700>")
# testData.append("<LED2,800,0.500>")
# testData.append("<LED2,200,0.200>")
# testData.append("<LED1,200,0.700>")

runTest(testData)
waitForFinish()


arduino.close
