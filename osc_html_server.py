# -*- coding: utf-8 -*-

# kill the HTTP server process: sudo netstat -tulnp | grep :5000

from flask import Flask, request, jsonify
from flask_cors import CORS

from osc4py3.as_eventloop import *
from osc4py3 import oscmethod as osm
from osc4py3 import oscbuildparse

import dynamixel_sdk as dxl
import threading
import json,time,math

# Initialize Flask app
app = Flask(__name__)
CORS(app)

# set to True for testing on a Windows laptop
BYPASS_SERIAL = False

# Configure Dynamixel serial port
PORT_NAME = '/dev/ttyS0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

ADDR_TORQUE_ENABLE = 24
ADDR_LED = 25
ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POSITION = 37
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

#def initSerial():
if not BYPASS_SERIAL:
    # Initialize PortHandler instance
    port_handler = dxl.PortHandler(PORT_NAME)

    # Initialize PacketHandler instance
    packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)

    # Open port
    if not port_handler.openPort():
        raise IOError("Failed to open port")

    # Set port baudrate
    if not port_handler.setBaudRate(BAUDRATE):
        raise IOError("Failed to set baud rate")
    for dxl_id in range(1, 7):
        dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, 29, 32)
        dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, 28, 0)
        dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, 27, 0)
        dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(port_handler, dxl_id, 32, 0)
        dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_LED, 3)
        time.sleep(0.1)
    for dxl_id in range(1, 7):
        dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_LED, 0)

######################## Poppy getters & setters ########################

def set_position(dxl_id, position):
    global BYPASS_SERIAL
    if BYPASS_SERIAL:
        print("set position "+str(dxl_id) + " "+str(position))
    else:
        if position is not None:
            # Write goal position
            posNum = int(512+position*1023/300)
            dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(port_handler, dxl_id, ADDR_GOAL_POSITION, posNum)
            if dxl_comm_result != dxl.COMM_SUCCESS:
                return packet_handler.getTxRxResult(dxl_comm_result)
            elif dxl_error != 0:
                return packet_handler.getRxPacketError(dxl_error)
        else:
            # Disable torque for this motor
            dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != dxl.COMM_SUCCESS:
                return packet_handler.getTxRxResult(dxl_comm_result)
            elif dxl_error != 0:
                return packet_handler.getRxPacketError(dxl_error)
        return "OK"

def set_led(dxl_id, color):
    global BYPASS_SERIAL

    if (dxl_id < 1) or (dxl_id > 6):
        print("invalid index:", str(dxl_id))
        return

    if BYPASS_SERIAL:
        print("set led "+str(dxl_id) + " "+str(color))
    else:
        if color=="off":
            a = 0
        elif color=="red":
            a = 1
        elif color=="green":
            a = 2
        elif color=="yellow":
            a = 3
        elif color=="blue":
            a = 4
        elif color=="purple":
            a = 5
        elif color=="cyan":
            a = 6
        elif color=="white":
            a = 7
        else:
            a = 0
        dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_LED, a)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            return packet_handler.getTxRxResult(dxl_comm_result)
        elif dxl_error != 0:
            return packet_handler.getRxPacketError(dxl_error)

def get_position(dxl_id):
    global BYPASS_SERIAL
    if BYPASS_SERIAL:
        return 100, 0
    else:
        position, dxl_comm_result, dxl_error = packet_handler.read2ByteTxRx(port_handler, dxl_id, ADDR_PRESENT_POSITION)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print("ERROR (COMM) failed reading position "+str(dxl_id))
            return None, packet_handler.getTxRxResult(dxl_comm_result)
        elif dxl_error != 0:
            print("ERROR (DXL) failed reading position "+str(dxl_id))
            return None, packet_handler.getRxPacketError(dxl_error)
        return int(-150+position*300/1023),0

######################## Threaded callbacks ########################

def update():
    global targetPoses, lastPose, lastUpdateTime, motionStartTime
    global RUNNING, UPDATE_INTERVAL_SEC, DEBUG
    while RUNNING:
        osc_process()

        if targetPoses is None:
            print("Error: targetPoses in None")
            targetPoses = []

        t=time.time()
        if (t - lastUpdateTime) > UPDATE_INTERVAL_SEC:
        
            if len(targetPoses)>0: # if there is a target
                targetPose = targetPoses[0]

                if len(targetPose) == 0 or len(targetPose) == 3 or len(targetPose) == 7: # move 1 motor or 6 motors 
                    if len(targetPose) == 0:
                        currentMotionDuration = 0
                    else:
                        currentMotionDuration = targetPoses[0][len(targetPose)-1] # duration is the last value

                    # if motion duration expired
                    if (t-motionStartTime) > currentMotionDuration: 
                        print("target reached")
                        sendPositions()

                        reachedPose = targetPoses.pop(0) # remove reached targetPosition
                        # store reached pos in case we cannot read the position when starting next motion
                        if len(reachedPose) == 7: 
                            lastPose = reachedPose # TODO check if not None
                        if len(reachedPose) == 3:
                            lastPose[reachedPose[0]] = reachedPose[1] # TODO check if not None

                        motionStartTime = t # start new motion (if there are more targets)

                        if len(targetPoses)>0:
                            print("target next pose :" +str(targetPoses[0]))
                    else:
                        # lerp move to target position
                        a = (t-motionStartTime)/currentMotionDuration
                        
                        if len(targetPose) == 3: # one motor: index, angle, duration
                            motorIndex = targetPose[0]
                            targetAngle = targetPose[1]
                            startAngle = lastPose[motorIndex-1]
                            if startAngle is not None:
                                set_position(motorIndex, None if targetAngle is None else (1-a)*startAngle + (a)*targetAngle)
                            else:
                                print("not moving motor "+str(motorIndex))

                        elif len(targetPose) == 7: # 6 motors: angle1-angle6, duration
                            for motorIndex in range(1, 7):
                                targetAngle = targetPoses[0][motorIndex-1]
                                startAngle = lastPose[motorIndex-1]
                                if startAngle is not None:
                                    set_position(motorIndex, None if targetAngle is None else (1-a)*startAngle + (a)*targetAngle)
                                else:
                                    print("not moving motor "+str(motorIndex))
                                    print(lastPose)
                                        
                elif len(targetPose) == 0:
                    print("STOP")
                else:
                    print("ERROR invalid targetPose length: "+str(len(targetPose)))
            lastUpdateTime = t
    
def setTargetPoses(newPoses):
    global targetPoses, motionStartTime, lastPose, DEBUG

    if newPoses is None: # Stop
        newPoses = [[]]

    # check poses integrity
    for pose in newPoses:
        if not len(pose) in [0, 3, 7]:
            print("ERROR invalid length for pose: "+str(pose))
            return False
        
    # set current pose as lastPose, if we can read it
    for motorIndex in range(1,7):
        angle, errMsg = get_position(motorIndex)
        if angle is None:
            print("ERROR can't read position "+str(motorIndex))
            print(errMsg)
            print("keeping last position instead")
            # TODO have an option do disable the motor motion instead
        else:
            lastPose[motorIndex-1] = angle

    targetPoses = newPoses.copy()
    motionStartTime=time.time() # start the new motion now
    if DEBUG:
        print("set target: "+str(targetPoses))
    return True

def sendPositions():
    global CLIENT_NAME
    #print("send positions to "+CLIENT_NAME)
    #msg = oscbuildparse.OSCMessage("/positions", ",iiiiii", [int(get_position(i)[0]) for i in range(1, 7)])
    #osc_send(msg, "client")
    for i in range(1, 7):
        motorIndex = 7-i # 6 to 1
        angle, errormsg = get_position(motorIndex)
        if angle is None:
            print("ERROR sent angle "+str(motorIndex)+" is None")
            print(errormsg)
        msg = oscbuildparse.OSCMessage("/angle/"+str(i), ",i", [404 if angle is None else angle])
        osc_send(msg, CLIENT_NAME)

######################## HTTP routing ########################

@app.route('/set_positions', methods=['POST'])
def handle_set_positions():
    comment = "OK"
    if not setTargetPoses(request.get_json()):
        return jsonify({"result": str(pose)+" n'a pas 0, 3 ou 7 éléments"}), 400
    return jsonify({"result": comment}), 200

@app.route('/set_leds', methods=['POST'])
def handle_set_leds():
    content = request.get_json()
    for dxl_id in range(1, 7):
        result = set_led(dxl_id, content[dxl_id-1])
    return jsonify({"result": "OK"}), 200

@app.route('/get_positions', methods=['GET'])
def handle_get_position():
    positions = []
    for dxl_id in range(1, 7):  # Assuming motor IDs are 1 to 6
        position, errmsg = get_position(dxl_id)
        if position is not None:
            positions.append(position)
        else:
            positions.append(None)
    return jsonify(positions), 200

######################## OSC Methods ########################

def initOSCServer(listeningPort):
    print("Opening OSC server...")
    osc_startup()
    # listen for messages on listeningPort, from any IP
    osc_udp_server("0.0.0.0", listeningPort, "localhost")
    print("    Listening for OSC messages on :"+str(listeningPort))

    osc_method("/*", gotOSCMsg, argscheme=osm.OSCARG_ADDRESS + osm.OSCARG_TYPETAGS) # we need adress and typetags
    osc_method("/handshake", updateClient, argscheme=osm.OSCARG_DATAUNPACK)
    osc_method("/leds/set", setLeds, argscheme=osm.OSCARG_TYPETAGS  + osm.OSCARG_DATAUNPACK) # check typetags + unpack data for separate functions
    osc_method("/motors/get", getPositions, argscheme=osm.OSCARG_TYPETAGS  + osm.OSCARG_DATAUNPACK) # check typetags + unpack data for separate functions
    osc_method("/motors/set", setPositions, argscheme=osm.OSCARG_TYPETAGS  + osm.OSCARG_DATAUNPACK) # check typetags + unpack data for separate functions
    osc_method("/motors/add", addToPosition, argscheme=osm.OSCARG_TYPETAGS  + osm.OSCARG_DATAUNPACK) # check typetags + unpack data for separate functions
    
    
def updateClient(ip ,port, name):
    osc_terminate()
    initOSCServer(9000)
    initOSCClient(ip, port, name)
    getPositions("/", [])

def initOSCClient(targetIp, targetPort, name):
    global CLIENT_NAME
    osc_udp_client(targetIp, targetPort, name)
    print("    I will send OSC message to "+name+" at "+str(targetIp)+":"+str(targetPort))
    CLIENT_NAME = name
    
    msg = oscbuildparse.OSCMessage("/hello", None, [])
    osc_send(msg, CLIENT_NAME)
        
def gotOSCMsg(address, typetags):
    global DEBUG
    if DEBUG:
        print("got OSC: "+  address + " typetags: " + typetags)
    

def getPositions(typetags, *args):
    sendPositions()

def setPositions(typetags, *args):
    #print("SET POSITIONS OSC")
    # set 6 angles
    if (typetags == ",fiiiiii"):
        duration = args[0]

        # instant pose
        if duration <= MINIMUM_DURATION: 
           for motorIndex in range(1,7): # 1 to 6
                set_position(motorIndex, args[motorIndex])
        # go to pose
        else:
            pose = [args[6], args[5], args[4], args[3], args[2], args[1], args[0]] # reverse 6 positions then append duration
            setTargetPoses([pose])
            
    # set one angle
    # instant move
    elif (typetags == ",ii"): 
        set_position(7-args[0], args[1])
    # disable
    elif (typetags == ",iI"): 
        motorIndex = 7-args[0]
        setTargetPoses(None) # interrupt current motion
        set_position(motorIndex, None) # 6 to 1
    # go to angle
    elif (typetags == ",fii"): 
        duration = args[0]
        motorIndex = 7-args[1] # 6 to 1
        angle = args[2]

        # instantaneous move
        if duration <= MINIMUM_DURATION: 
            setTargetPoses(None) # interrupt current motion
            # TODO interrupt only this motor ?
            set_position(motorIndex, angle)
        # go to
        else:
            setTargetPoses([ [motorIndex, angle, duration] ])
    
    # set all            
    elif (typetags == ",i"):
        angle = args[0]
        setTargetPoses(None) # interrupt current motion
        for motorIndex in range(1,7): # 1 to 6
            set_position(motorIndex, angle)
    elif (typetags == ",fi"):
        duration = args[0]
        angle = args[1]

        # instantaneous move
        if duration <= MINIMUM_DURATION: 
            setTargetPoses(None) # interrupt current motion
            for motorIndex in range (1,7):
                set_position(motorIndex, angle)
        else:
            targetPose = [angle for i in range(6)]
            targetPose.append(duration)
            setTargetPoses([targetPose])
    elif (typetags == ",I"):
        setTargetPoses(None) # interrupt current motion
        for motorIndex in range(1,7):
            set_position(motorIndex, None)
            
    else:
        print("ERROR: typetags are not valid")

def addToPosition(typetags, *args):
    # index, value
    if (typetags == ",ii"):
        motorIndex = 7-args[0]
        angle = args[1]
        currentAngle = get_position(motorIndex)[0]
        if currentAngle is not None:
            value = min(90, max(-90, currentAngle + angle))
            set_position(motorIndex, value)
        
def setLeds(typetags, *args):
    # set 6 leds
    if (typetags == ",ssssss"):
        for index, val in enumerate(args):
            ledIndex = index + 1
            set_led(ledIndex, val)
            
    elif (typetags == ",iiiiii"):
        #args.reverse() # 6 to 1
        for index, val in enumerate(args):
            ledIndex = 6 - index # 6 to 1
            colors =["off", "red", "green", "yellow", "blue", "purple", "cyan", "white"]
            set_led(ledIndex, colors[val])
           
    # set 1 led
    elif (typetags == ",is"):
        ledIndex = 7-args[0] # 6 to 1
        color = args[i]
        set_led(ledIndex, color)

    elif (typetags == ",ii"):
        ledIndex = 7-args[0] # 6 to 1
        colorIndex = args[1] # 0 to 7
        colors =["off", "red", "green", "yellow", "blue", "purple", "cyan", "white"]
        set_led(ledIndex, colors[colorIndex])
            
    # set all
    elif (typetags == ",s"):
        for ledIndex in range(1,7):
            set_led(ledIndex, args[0])

    elif (typetags == ",i"):
        for ledIndex in range(1,7):
            set_led(ledIndex, colors[args[0]])
    
    else:
        print("ERROR: typetags are not valid")
      
#########################################################

# Run the app
if __name__ == '__main__': 
    UPDATE_INTERVAL_SEC = 0.05
    MINIMUM_DURATION = 0.11
    DEBUG = True
    CLIENT_NAME = "client"
    #STREAM_POS = True # TODO
    lastUpdateTime =  time.time()
    motionStartTime=time.time()
    targetPoses = [[]]
    lastPose = [0, 0, 0, 0, 0, 0, 2.0]


    try:
        # init OSC 
        initOSCServer(9000)
        initOSCClient("10.0.0.2", 12000, "local")
        #  update thread
        RUNNING = True
        thread = threading.Thread(target=update)
        thread.start()

        # run flask app
        app.run(host='0.0.0.0')
        

    except KeyboardInterrupt:
        pass
    finally:
        RUNNING = False
        osc_terminate()
        print("closing OSC")

