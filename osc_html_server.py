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
            return None, packet_handler.getTxRxResult(dxl_comm_result)
        elif dxl_error != 0:
            return None, packet_handler.getRxPacketError(dxl_error)
        return int(-150+position*300/1023),0

######################## Threaded callbacks ########################

def update():
    global startPose, targetPoses, lastUpdateTime, motionStartTime, RUNNING, UPDATE_INTERVAL_SEC, newTarget
    while RUNNING:
        osc_process()

        if targetPoses is None:
            print("Error: targetPoses in None")
            targetPoses = []

        t=time.time()
        if (t - lastUpdateTime) > UPDATE_INTERVAL_SEC:
        
            if len(targetPoses)>0 and len(targetPoses[0])>5:
                currentMotionDuration = targetPoses[0][6]

                # if motion duration expired
                if (t-motionStartTime) > currentMotionDuration: 
                    #print("target reached")
                    #print(t-motionStartTime)
                    #print(currentMotionDuration)
                    targetPoses.pop(0) # remove reached targetPosition
                    motionStartTime = t # start new motion
                    updateStartPose()

                    if len(targetPoses)>0:
                        print("target next pose :" +str(targetPoses[0]))
                else:
                    # lerp move to target position
                    for motorIndex in range(1, 7):
                        a = (t-motionStartTime)/currentMotionDuration
                        targetPosition = targetPoses[0][motorIndex-1]
                        startPosition = startPose[motorIndex-1]
                        if startPosition is not None:
                            set_position(motorIndex, None if targetPosition is None else (1-a)*startPosition + (a)*targetPosition)
            lastUpdateTime = t

def updateStartPose():
    global startPose
    startPose = [get_position(i)[0] if get_position(i)[0] is not None else 0 for i in range(1,7)]
    startPose.append(0) # duration
    if DEBUG:
        print("update start pose: " +str(startPose))
    sendPositions()
    return startPose
    
def setTargetPoses(newPoses):
    global targetPoses, motionStartTime

    if newPoses is None:
        #print("STOP")
        targetPoses = [[]]
        return True

    if DEBUG:
        print("set target: "+str(newPoses))
    # check integrity
    for pose in newPoses:
        if len(pose) != 7:
            print("ERROR pose not valid")
            return False
    
    motionStartTime=time.time()
    newTarget = newPoses.copy()
    newTarget.insert(0, updateStartPose()) # insert a dummy position
    targetPoses = newTarget
    if DEBUG:
        print("targetPoses: "+str(targetPoses))
    return True

def sendPositions():
    global CLIENT_NAME
    print("send positions to "+CLIENT_NAME)
    #msg = oscbuildparse.OSCMessage("/positions", ",iiiiii", [int(get_position(i)[0]) for i in range(1, 7)])
    #osc_send(msg, "client")
    for i in range(1, 7):
        motorIndex = 7-i # 6 to 1
        msg = oscbuildparse.OSCMessage("/angle/"+str(i), ",i", [int(get_position(motorIndex)[0])])
        osc_send(msg, CLIENT_NAME)

######################## HTTP routing ########################

@app.route('/set_positions', methods=['POST'])
def handle_set_positions():
    #print("SET POSITIONS")
    comment = "OK"
    if not setTargetPoses(request.get_json()):
        return jsonify({"result": str(pose)+" n'a pas 7 éléments"}), 400
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
    print("Listening for OSC messages on :")
    print(str(listeningPort))

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
    print("I will send OSC message to "+name+" at ")
    print(str(targetIp)+":"+str(targetPort))
    CLIENT_NAME = name
    
    msg = oscbuildparse.OSCMessage("/hello", None, [])
    osc_send(msg, CLIENT_NAME)
        
def gotOSCMsg(address, typetags):
    global DEBUG
    if (DEBUG):
        print("got OSC: "+  address + " typetags: " + typetags)
    

def getPositions(typetags, *args):
    sendPositions()

def setPositions(typetags, *args):
    print("SET POSITIONS OSC")
    #6 values
    if (typetags == ",fiiiiii"):
        duration = args[0]
        
        if duration <= MINIMUM_DURATION: 
           print("instant pose")
           for motorIndex in range(1,7): # 1 to 6
                set_position(motorIndex, args[motorIndex])
        else:
            print("go to pose")
            pose = [args[6], args[5], args[4], args[3], args[2], args[1], args[0]] # duration after 6 positions
            #pose = args.flip()
            setTargetPoses([pose])
            
    # index, value
    elif (typetags == ",ii"):
        set_position(7-args[0], args[1])
    elif (typetags == ",fii"):
        duration = args[0]
        motorIndex = 7-args[1] # 6 to 1
        pos = args[2]

        # instantaneous move
        if duration <= MINIMUM_DURATION: 
            setTargetPoses(None) # interrupt current motion
            set_position(motorIndex, pos)
        else:
            targetPose = [int(get_position(i)[0]) for i in range(1,7)]
            targetPose.append(duration)
            targetPose[motorIndex-1] = pos
            setTargetPoses([targetPose])
    elif (typetags == ",iI"):
        setTargetPoses(None) # interrupt current motion
        set_position(7-args[0], None) # 6 to 1
    
    # set all            
    elif (typetags == ",i"):
        setTargetPoses(None) # interrupt current motion
        for motorIndex in range(1,7): # 1 to 6
            set_position(motorIndex, args[0])
    elif (typetags == ",fi"):
        duration = args[0]
        pos = args[1]

        # instantaneous move
        if duration <= MINIMUM_DURATION: 
            setTargetPoses(None) # interrupt current motion
            for motorIndex in range (1,7):
                set_position(motorIndex, pos)
        else:
            targetPose = [pos for i in range(6)]
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
        index = 7-args[0]
        value = min(90, max(-90, int(get_position(index)[0]) + args[1]))
        set_position(index, value)
        
def setLeds(typetags, *args):
    print(args)

    # 6 values
    if (typetags == ",ssssss"):
        for index, val in enumerate(args):
            ledIndex = index + 1
            set_led(ledIndex, val)
            
    if (typetags == ",iiiiii"):
        for index, val in enumerate(args):
            ledIndex = index + 1
            colors =["off", "red", "green", "yellow", "blue", "purple", "cyan", "white"]
            set_led(ledIndex, colors[val])
           
    # index, value 
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
    updateStartPose()



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

