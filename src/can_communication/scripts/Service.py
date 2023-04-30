#!/usr/bin/env python

from __future__ import print_function

from can_communication.srv import *
from can_communication.msg import *
import rospy
import can
import time
import atexit

from Odrive import *
from CAN_Servo import *
from VESC_Motor import *

CAN_ID = 0x0E

def on_exit():
    notifier.stop()
    bus.shutdown()

def timer_callback(event):
    odrive_motor.Get_Pos_Vel()
    odrive_motor.Get_Bus_Cur()
    

def Process_Message(msg):
    # rospy.logdebug("Received message:", msg.data)
    # 12 bits ADC -> Angle
    # if (msg.arbitration_id == 0x0E):
    #     servo.Process_Message(msg)
    # elif  ((msg.arbitration_id & 0xFF) == motor_1.ID):
    #     motor_1.Process_Message(msg)
    # elif  ((msg.arbitration_id & 0xFF) == motor_2.ID):
    #     motor_2.Process_Message(msg)
    if((msg.arbitration_id >> 5 & 0xFF) == Odrive_Node_ID):
        odrive_motor.Process_Message(msg)
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

def handle_odrive_motor(req):
    odrive_motor.Set_RPM(req.Motor_vel)
    print("RPM={}".format(req.Motor_vel))
    response = Odrive_srvResponse()
    response.success = True
    response.message = "OK"
    return response

rospy.init_node('can_server')

# can_servo_pub = rospy.Publisher('Bicycle/CAN_Servo_info', Bicycle, queue_size=10)
# motor_1_pub = rospy.Publisher('Bicycle/VESC_Motor_1_info', Bicycle, queue_size=10)
# motor_2_pub = rospy.Publisher('Bicycle/VESC_Motor_2_info', Bicycle, queue_size=10)
odrive_motor_pub = rospy.Publisher('Bicycle/Odrive_Motor_info', Odrive_msg, queue_size=100)

logger = can.CSVWriter("/home/prowinter/Desktop/Bicycle_ws/log/log.csv") 

filters = [
    {"can_id": 0x0E, "can_mask": 0x7FF, "extended": False},     # CAN-Servo
    {"can_id": VESC_ID_1 | CAN_PACKET_STATUS_1 << 8, "can_mask": 0x1FFFFFFF, "extended": True},     # VESC-STATUS_1
    {"can_id": VESC_ID_2 | CAN_PACKET_STATUS_1 << 8, "can_mask": 0x1FFFFFFF, "extended": True},     # VESC-STATUS_1
    {"can_id": VESC_ID_1 | CAN_PACKET_STATUS_5 << 8, "can_mask": 0x1FFFFFFF, "extended": True},     # VESC-STATUS_5
    {"can_id": Odrive_Node_ID << 5 | Axis0_Get_Encoder_Estimates, "can_mask": 0x7FF, "extended": False}, 
    {"can_id": Odrive_Node_ID << 5 | Axis0_Get_Bus_Voltage_Current, "can_mask": 0x7FF, "extended": False}, 
]

bus = can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=500000, can_filters=filters)

# servo = CAN_Servo(CAN_ID,bus,can_servo_pub)
# motor_1 = VESC_Motor(VESC_ID_1,bus,motor_1_pub)
# motor_2 = VESC_Motor(VESC_ID_2,bus,motor_2_pub)
odrive_motor = Odrive_Motor(Odrive_Node_ID,bus,odrive_motor_pub)

listeners = [
    Process_Message,    # Callback function, print the received messages
    logger,             # save received messages to log file
]

notifier = can.Notifier(bus, listeners)
atexit.register(on_exit)



# a = rospy.Service('CAN_Servo', CAN, handle_can_servo)
# b = rospy.Service('VESC_Motor_1', VESC, handle_motor_1)
# c = rospy.Service('VESC_Motor_2', VESC, handle_motor_2)
d = rospy.Service('Odrive_Motor', Odrive_srv, handle_odrive_motor)
timer = rospy.Timer(rospy.Duration(1.0/100.0), timer_callback)
rospy.spin()