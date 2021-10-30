from flask import Flask, request, jsonify, send_file
import serial


def sendCommand(text):
    print("Sending command:" + text)
    hexapod = serial.Serial(port="COM3", baudrate=9600)
    hexapod.write((text + "\n").encode('ascii'))
    commandResponse = str(hexapod.readline()).rstrip()
    hexapod.close()
    return commandResponse


app = Flask(__name__)

@app.route('/', methods=['GET'])
def index():
    return send_file('index.html')

@app.route('/leg/<leg_index>', methods = ['PUT'])
def leg(leg_index):
    if request.method == 'PUT':
        data = request.get_json()
        if data is None:
            raise Exception("No json provided")

        x = data['position']['x']
        y = data['position']['y']
        z = data['position']['z']
        commandResponse = sendCommand("legik {} {} {} {}".format(leg_index, x, y, z))
        return jsonify({"result":commandResponse})

@app.route('/leg/<leg_index>/joint/<joint_index>', methods = ['PUT'])
def legJointAngle(leg_index, joint_index):
    data = request.get_json()
    if data is None:
        raise Exception("No json provided")

    degrees = int(data['degrees'])
    commandResponse = sendCommand("legjointangle {} {} {}".format(leg_index, joint_index, degrees))
    return jsonify({"result":commandResponse})

@app.route('/servo/<servo_index>', methods = ['POST'])
def servo(servo_index):
    if request.method == 'POST':
        data = request.get_json()
        if data is None:
            raise Exception("No json provided")

        us = data['us']
        commandResponse = sendCommand("jointmove {} {}".format(servo_index, us))
        return jsonify({"result":commandResponse})

@app.route('/release', methods = ['POST'])
def release():
    if request.method == 'POST':
        commandResponse = sendCommand("release")
        return jsonify({"result":commandResponse})


app.run(host="0.0.0.0", port=5000)
