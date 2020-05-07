# self driving module is simulating self driving ECU, which get image from camera
# through image cable and speed from speed ECU from CAN BUS

#parsing command line arguments
import argparse
#decoding camera images
import base64
#for frametimestamp saving
from datetime import datetime
#reading and writing files
import os
#high level file operations
import shutil
#matrix math
import numpy as np
#real-time server
import socketio
#concurrent networking
import eventlet
#web server gateway interface
import eventlet.wsgi
#image manipulation
from PIL import Image
#web framework
from flask import Flask
#input output
from io import BytesIO

#load our saved model
from keras.models import load_model

#helper class
import utils

# socket transfer
from socket import *
# transfer dictionary using json
import json
# child process
import multiprocessing

import time

import can

#init our model and image array as empty
model = None
prev_image_array = None

#set min/max speed for our autonomous car
MAX_SPEED = 25
MIN_SPEED = 10

#and a speed limit
speed_limit = MAX_SPEED

# using trained CNN to predict
def predict(image, speed,conn):
    try:
        image = Image.open(BytesIO(base64.b64decode(image)))
        image = np.asarray(image)       # from PIL image to numpy array
        image = utils.preprocess(image) # apply the preprocessing
        image = np.array([image])       # the model expects 4D array

        # predict the steering angle for the image
        steering_angle = float(model.predict(image, batch_size=1))
        # lower the throttle as the speed increases
        # if the speed is above the current speed limit, we are on a downhill.
        # make sure we slow down first and then go back to the original max speed.
        global speed_limit
        if speed > speed_limit:
            speed_limit = MIN_SPEED  # slow down
        else:
            speed_limit = MAX_SPEED
        throttle = 1.0 - steering_angle**2 - (speed/speed_limit)**2

        print('{} {} {}'.format(steering_angle, throttle, speed))
        # inform parent process through pipe
        m_dict['throttle'] = throttle
        m_dict['steering_angle'] = steering_angle
        conn.send(1)


    except Exception as e:
        print(e)
    except OSError:
        print("Error: Image read error")

    # save frame
    if args.image_folder != '':
        timestamp = datetime.utcnow().strftime('%Y_%m_%d_%H_%M_%S_%f')[:-3]
        image_filename = os.path.join(args.image_folder, timestamp)
        image.save('{}.jpg'.format(image_filename))

# child process functions to listen and send message on CANBUS, once it get
# speed message from CANBUS, send speed data to parent process using pipe. Once
# it get throttle and steering angle instruction from parent process through pipe,
# it send CANBUS message
def CANBUS_send(conn):
    channel = 'vcan0'
    bustype = 'socketcan'
    bus = can.interface.Bus(channel=channel, bustype=bustype)
    print("CANBUS CONNECTION SUCCESS")

    while True:
        data = conn.recv()
        # treat throttle and steering_angle instruction
        throttle = m_dict['throttle']
        steering_angle = m_dict['steering_angle']
        # print(throttle)
        # send throttle data to CANBUS
        throttle_data = [0]*8
        # change the value range from [-1, 1] to [0, 2]
        throttle = float(throttle)+1
        for i in range(8):
            throttle_data[i] = int(throttle*10**i)%10
        msg = can.Message(arbitration_id=0x6ce, data=throttle_data, is_extended_id=False)
        bus.send(msg)
        # print(steering_angle)
        # send steering angle data to CANBUS
        steering_angle_data = [0]*8
        # change the value range from [-1, 1] to [0, 2]
        steering_angle = float(steering_angle)+1
        for i in range(8):
            steering_angle_data[i] = int(steering_angle*10**i)%10
        msg = can.Message(arbitration_id=0x6ca, data=steering_angle_data, is_extended_id=False)
        bus.send(msg)

def CANBUS_listen():
    channel = 'vcan0'
    bustype = 'socketcan'
    bus = can.interface.Bus(channel=channel, bustype=bustype)
    print("CANBUS CONNECTION SUCCESS")

    while True:
        message = bus.recv()
        # throttle data from CANBUS
        if message.arbitration_id==0x6dd:
            print("get speed data from speed ECU")
            speed = 0
            for i in range(8):
                speed += message.data[i]*10**(-i+2)
            m_dict['speed'] = speed
            print('speed: ', speed)
            continue
        if message.arbitration_id==0x6df:
            print("get automonous status data from dashboard")
            automonous_status = message.data[0]
            m_dict['automonous_status'] = automonous_status
            print('automonous_status: ',automonous_status)
            continue 



if __name__ == '__main__':
    manager = multiprocessing.Manager()
    m_dict = manager.dict()
    m_dict['speed'] = 0
    m_dict['throttle'] = 0
    m_dict['steering_angle'] = 0
    m_dict['automonous_status'] = 0
    # create a pipe between parent process and chile process when parent prediction
    # function output an instruction send to child process through pipe
    parent_conn, child_conn = multiprocessing.Pipe()
    # create a child process to listen and send message from CANBUS
    p1 = multiprocessing.Process(target=CANBUS_send, args=(child_conn, ))
    p1.start()
    p2 = multiprocessing.Process(target=CANBUS_listen)
    p2.start()

    parser = argparse.ArgumentParser(description='Remote Driving')
    parser.add_argument(
        'model',
        type=str,
        help='Path to model h5 file. Model should be on the same path.'
    )
    parser.add_argument(
        'image_folder',
        type=str,
        nargs='?',
        default='',
        help='Path to image folder. This is where the images from the run will be saved.'
    )
    args = parser.parse_args()

    #load model
    model = load_model(args.model)

    if args.image_folder != '':
        print("Creating image folder at {}".format(args.image_folder))
        if not os.path.exists(args.image_folder):
            os.makedirs(args.image_folder)
        else:
            shutil.rmtree(args.image_folder)
            os.makedirs(args.image_folder)
        print("RECORDING THIS RUN ...")
    else:
        print("NOT RECORDING THIS RUN ...")

    HOST = ''
    PORT = 21568
    BUFSIZ = 40960
    ADDR = (HOST,PORT)

    tcpSerSock = socket(AF_INET,SOCK_STREAM)
    tcpSerSock.bind(ADDR)
    tcpSerSock.listen(5)

    # initial connection to camera
    while True:
        print('waiting for camera connection...')
        tcpCliSock, addr = tcpSerSock.accept()
        print('...connnecting from :', addr)
        print('CAMERA CONNECTION SUCCESS!')

        while True:
            # get image data from image cable
            data = tcpCliSock.recv(BUFSIZ)
            if not data:
                break
            # get speed data from CANBUS
            speed = m_dict['speed']
            # input datas to trained model
            if m_dict['automonous_status'] == 1:
                predict(data.decode(), speed, parent_conn)

        tcpCliSock.close()
    tcpSerSock.close()
