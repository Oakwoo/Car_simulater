# the body control module is a intergration module of DCU（drive control unit)
# and steering control unit, which get the throttle and steering angle instruction
# from CAN BUS and transfer these instruction of electrical signal to control
# throttle percentage and pressure of steering pump

from socket import *
# child process
import multiprocessing
# transfer dictionary using json
import json
import can

# child process function to listen message on CANBUS, once it get message, send
# data to parent process through pipe
def CANBUS_connection(conn):
    channel = 'vcan0'
    bustype = 'socketcan'
    bus = can.interface.Bus(channel=channel, bustype=bustype)
    print("CANBUS CONNECTION SUCCESS")

    while True:
        message = bus.recv()
        # throttle data from CANBUS
        if message.arbitration_id==0x6ce:
            print("get throttle data from CANBUS")
            throttle = 0
            for i in range(8):
                throttle += message.data[i]*10**(-i)
            m_dict['throttle'] = throttle-1
            print(throttle)
            conn.send(1)
            continue
        # steering angle data from CANBUS
        if message.arbitration_id==0x6ca:
            print("get steering angle data from CANBUS")
            steering_angle = 0
            for i in range(8):
                steering_angle += message.data[i]*10**(-i)
            m_dict['steering_angle'] = steering_angle-1
            print(steering_angle)
            conn.send(1)
            continue



if __name__ == '__main__':
    manager = multiprocessing.Manager()
    m_dict = manager.dict()
    m_dict['throttle'] = 0
    m_dict['steering_angle'] = 0
    parent_conn, child_conn = multiprocessing.Pipe()
    # create a child process to listen message from CANBUS
    p = multiprocessing.Process(target=CANBUS_connection, args=(child_conn, ))
    p.start()

    # initial connection between body control module to mechanism module to simulate
    # the hard instruction wire connecting body control to throttle motor, steering pump
    HOST = '10.183.27.73'
    PORT = 12345
    BUFSIZ =40960
    ADDR = (HOST,PORT)

    tcpCliSock = socket(AF_INET,SOCK_STREAM)
    tcpCliSock.connect(ADDR)
    print("MECHANISM CONNECTION SUCCESS")

    while True:
        # asyn
        data = parent_conn.recv()
        # treat throttle and steering_angle instruction
        throttle = m_dict['throttle']
        steering_angle = m_dict['steering_angle']
        dict = {"steering_angle":steering_angle, "throttle":throttle}
        data = json.dumps(dict)
        print(data)
        tcpCliSock.send(data.encode())
