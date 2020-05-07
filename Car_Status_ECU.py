# socket transfer
from socket import *
# transfer dictionary using json
import json

import can

if __name__ == '__main__':
    channel = 'vcan0'
    bustype = 'socketcan'
    bus = can.interface.Bus(channel=channel, bustype=bustype)
    print("CANBUS CONNECTION SUCCESS")
    
    HOST = ''
    PORT = 23333
    BUFSIZ = 40960
    ADDR = (HOST,PORT)

    tcpSerSock = socket(AF_INET,SOCK_STREAM)
    tcpSerSock.bind(ADDR)
    tcpSerSock.listen(5)

    # initial connection to camera
    while True:
        print('waiting for speed and rotate sensors connection...')
        tcpCliSock, addr = tcpSerSock.accept()
        print('...connnecting from :', addr)
        print('SPEED SENSORS CONNECTION SUCCESS!')
        print('ROTATE SENSORS CONNECTION SUCCESS!')

        while True:
            # get speed and rotate data from image cable
            data = tcpCliSock.recv(BUFSIZ)
            if not data:
                break
            # get speed data from sensors
            data = data.decode()
            dict = json.loads(data)
            speed = dict['speed']
            rotate_speed = dict['rotate_speed']
            # print(speed)
            # send speed data to dashboard and self driving module
            speed_data = [0]*8
            speed = float(speed)
            for i in range(8):
                speed_data[i] = int(speed*10**(i-2))%10
            msg = can.Message(arbitration_id=0x6bd, data=speed_data, is_extended_id=False)
            bus.send(msg)
            msg = can.Message(arbitration_id=0x6dd, data=speed_data, is_extended_id=False)
            bus.send(msg)
            # print(rotate_speed)
            # send rotate speed data to CANBUS
            rotate_speed_data = [0]*8
            # change the value range from [-1, 1] to [0, 2]
            rotate_speed = float(rotate_speed)+1
            for i in range(8):
                rotate_speed_data[i] = int(rotate_speed*10**i)%10
            msg = can.Message(arbitration_id=0x6b0, data=rotate_speed_data, is_extended_id=False)
            bus.send(msg)

        tcpCliSock.close()
    tcpSerSock.close()
