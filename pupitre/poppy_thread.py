from flask import Flask, request, jsonify
from flask_cors import CORS
import dynamixel_sdk as dxl
import threading
import json,time,math
global listPos,t0,posInit

# Initialize Flask app
app = Flask(__name__)
CORS(app)

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
listPos = []
t0=time.time()



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

# Function to set position
def set_position(dxl_id, position):
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

# Function to get current position
def get_position(dxl_id):
    position, dxl_comm_result, dxl_error = packet_handler.read2ByteTxRx(port_handler, dxl_id, ADDR_PRESENT_POSITION)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        return None, packet_handler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        return None, packet_handler.getRxPacketError(dxl_error)
    return int(-150+position*300/1023),0

posInit = []
for dxl_id in range(1, 7):  # Assuming motor IDs are 1 to 6
    position, errmsg = get_position(dxl_id)
    if position is not None:
        posInit.append(position)
    else:
        posInit.append(0)
print("posIint:",posInit)

def run_script():
    global listPos,t0,posInit
    while True:
        t=time.time()
        
        try:
            if len(listPos)>0:
                print(listPos,t-t0)
                if (t-t0)>listPos[0][6]:
                    listPos.pop(0)
                    t0=t
                    posInit = []
                    for dxl_id in range(1, 7):  # Assuming motor IDs are 1 to 6
                        position, errmsg = get_position(dxl_id)
                        
                        if position is not None:
                            posInit.append(position)
                        else:
                            posInit.append(0)
                    print("posIint:",posInit)
                else:
                    for dxl_id in range(1, 7):
                        a = (t-t0)/listPos[0][6]
                        if listPos[0][dxl_id-1] is not None:
                            set_position(dxl_id, (1-a)*posInit[dxl_id-1]+(a)*listPos[0][dxl_id-1])
                        else:
                            set_position(dxl_id, None)
        except Exception as error:
            # handle the exception
            print("An exception occurred:", error) # An exception occurred: division by zero
        
        while time.time()<t+0.04:
            time.sleep(0.001)

thread = threading.Thread(target=run_script)
thread.start()

@app.route('/set_positions', methods=['POST'])
def handle_set_positions():
    global listPos,t0,posInit
    comment = "OK"
    content = request.get_json()
    listPos = [content[0]]
    for c in content:
        if (len(c)==7):
            listPos.append(c)
        else:
            comment = str(c)+" n'a pas 7 éléments"
    t0=0
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

# Run the app
if __name__ == '__main__':
    app.run(host='0.0.0.0')