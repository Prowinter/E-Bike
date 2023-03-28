#!/usr/bin/env python

from __future__ import print_function

from can_communication.srv import *
from can_communication.msg import *
import rospy
import can
import time
import atexit

from CAN_Servo import *
from VESC_Motor import *

CAN_ID = 0x0E

def on_exit():
    notifier.stop()
    bus.shutdown()

def Process_Message(msg):
    # rospy.logdebug("Received message:", msg.data)
    # 12 bits ADC -> Angle
    if (msg.arbitration_id == 0x0E):
        servo.Process_Message(msg)
    elif  ((msg.arbitration_id & 0xFF) == motor_1.ID):
        motor_1.Process_Message(msg)
    elif  ((msg.arbitration_id & 0xFF) == motor_2.ID):
        motor_2.Process_Message(msg)
    else:
        print("error")

def handle_can_servo(req):
    result = servo.RotationAngle(req.Servo_Direction)
    # print("Angle={}".format(result))
    response = CANResponse()
    response.success = True
    response.message = "OK"
    return response

def handle_motor_1(req):
    result = motor_1.SetRPM(req.Motor_rpm)
    # print("RPM={}".format(result))
    response = VESCResponse()
    response.success = True
    response.message = "OK"
    return response

def handle_motor_2(req):
    result = motor_2.SetDuty(req.Motor_duty)
    # print("Duty={}".format(result))
    response = VESCResponse()
    response.success = True
    response.message = "OK"
    return response

rospy.init_node('can_server')

can_servo_pub = rospy.Publisher('Bicycle/CAN_Servo_info', Bicycle, queue_size=10)
motor_1_pub = rospy.Publisher('Bicycle/VESC_Motor_1_info', Bicycle, queue_size=10)
motor_2_pub = rospy.Publisher('Bicycle/VESC_Motor_2_info', Bicycle, queue_size=10)

logger = can.CSVWriter("/home/prowinter/Desktop/Bicycle_ws/log/log.csv") 

filters = [
    {"can_id": 0x0E, "can_mask": 0x7FF, "extended": False},     # CAN-Servo
    {"can_id": VESC_ID_1 | CAN_PACKET_STATUS_1 << 8, "can_mask": 0x1FFFFFFF, "extended": True},     # VESC-STATUS_1
    {"can_id": VESC_ID_2 | CAN_PACKET_STATUS_1 << 8, "can_mask": 0x1FFFFFFF, "extended": True},     # VESC-STATUS_1
    {"can_id": VESC_ID_1 | CAN_PACKET_STATUS_5 << 8, "can_mask": 0x1FFFFFFF, "extended": True},     # VESC-STATUS_5
]

bus = can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=1000000, can_filters=filters)

servo = CAN_Servo(CAN_ID,bus,can_servo_pub)
motor_1 = VESC_Motor(VESC_ID_1,bus,motor_1_pub)
motor_2 = VESC_Motor(VESC_ID_2,bus,motor_2_pub)

listeners = [
    Process_Message,    # Callback function, print the received messages
    logger,             # save received messages to log file
]

notifier = can.Notifier(bus, listeners)
atexit.register(on_exit)


a = rospy.Service('CAN_Servo', CAN, handle_can_servo)
b = rospy.Service('VESC_Motor_1', VESC, handle_motor_1)
c = rospy.Service('VESC_Motor_2', VESC, handle_motor_2)
# motor_1.SetRPM(1000)
rospy.spin()