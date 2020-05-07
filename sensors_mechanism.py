# sensors_mechanism module is an abstract layer to simulate physcial mechanism
# such as rudder motor, restrictor motor and various of sensors, such as speed
# sensor, camera for self-driving, steering angle sensor

#for frametimestamp saving
from datetime import datetime
#reading and writing files
import os
#high level file operations
import shutil
#real-time server
import socketio
#concurrent networking
import eventlet
#web server gateway interface
import eventlet.wsgi
#web framework
from flask import Flask
# socket transfer
from socket import *
# transfer dictionary using json
import json
# child process
import multiprocessing

#initialize our server
sio = socketio.Server()
#our flask (web) app
app = Flask(__name__)

#registering event handler for the server
@sio.on('telemetry')
def telemetry(sid, data):
    print('telemetry')

    # send image data to self driving ECU through socket
    global tcpCliSock
    tcpCliSock.send(data['image'].encode())

    dict = {"speed":float(data['speed']), "rotate_speed":float(data['throttle'])}
    send_data = json.dumps(dict)
    tcpCliSock2.send(send_data.encode())

    send_control(m_dict['steering_angle'],m_dict['throttle'])

    # send speed data to speed ECU through CANBUS



@sio.on('connect')
def connect(sid, environ):
    print('connect success', sid)
    send_control(0,0)

def send_control(steering_angle, throttle):
    sio.emit(
        "steer",
        data={
            'steering_angle': steering_angle.__str__(),
            'throttle': throttle.__str__()
        },
        skip_sid=True)

# child process function to communicate with body_control module
def body_control_connection():
    HOST = ''
    PORT = 12345
    BUFSIZ = 40960
    ADDR = (HOST,PORT)

    tcpSerSock = socket(AF_INET,SOCK_STREAM)
    tcpSerSock.bind(ADDR)
    tcpSerSock.listen(5)

    # initial connection to body_control module
    while True:
        print('waiting for body control module connection...')
        tcpCliSock, addr = tcpSerSock.accept()
        print('...connnecting from :', addr)
        print('BODY CONTROL CONNECTION SUCCESS!')

        while True:
            # get throttle and steering angle signal from body control
            data = tcpCliSock.recv(BUFSIZ)
            # handle the signal
            data = data.decode()
            dict = json.loads(data)
            throttle = dict['throttle']
            steering_angle = dict['steering_angle']
            # motor and pump invoke mechanism action
            # send_control(steering_angle,throttle)  can't send data
            # using share value to update
            print('throttle: '+str(throttle)+'  steering_angle: '+str(steering_angle))
            m_dict['throttle'] = throttle
            m_dict['steering_angle'] = steering_angle


        tcpCliSock.close()
    tcpSerSock.close()


if __name__ == '__main__':
    manager = multiprocessing.Manager()
    m_dict = manager.dict()
    m_dict['throttle'] = 0
    m_dict['steering_angle'] = 0
    # create a child process to listen message from boby control module
    p = multiprocessing.Process(target=body_control_connection)
    p.start()

    # initial connection between camera to self driving module to simulate the image
    # transfer cable connecting camera, laser radar to self driving ECU
    HOST = '127.0.0.1'
    PORT = 21568
    BUFSIZ =40960
    ADDR = (HOST,PORT)
    tcpCliSock = socket(AF_INET,SOCK_STREAM)
    tcpCliSock.connect(ADDR)

    # initial connection between speed and rotate speed to car status ECU module
    # to simulate the signal cable connecting sensors to car status ECU
    HOST2 = '127.0.0.1'
    PORT2 = 23333
    BUFSIZ2 =40960
    ADDR2 = (HOST2,PORT2)
    tcpCliSock2 = socket(AF_INET,SOCK_STREAM)
    tcpCliSock2.connect(ADDR2)

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
