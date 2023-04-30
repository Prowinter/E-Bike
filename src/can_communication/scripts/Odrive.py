import can
import rospy
import struct
from can_communication.msg import Odrive_msg
import ctypes

Odrive_Node_ID = 0x05

Axis0_Get_Encoder_Estimates = 0x09
Axis0_Get_Bus_Voltage_Current = 0x17

Axis0_Set_Input_Vel = 0x0D

class FloatUnion(ctypes.Union):
    _fields_ = [("bytes", ctypes.c_uint32),("value", ctypes.c_float)]
    
class UintUnion(ctypes.Union):
    _fields_ = [("float_val", ctypes.c_float),("byte_vals", ctypes.c_uint8 * 4)]

class Odrive_Motor:
    def __init__(self ,id ,bus ,Publisher):
        self.ID = id
        self.bus = bus
        self.motor_rpm = 0.0
        self.motor_pos = 0.0
        self.motor_voltage = 0.0
        self.motor_current = 0.0
        self.MaxRPM = 30
        self.Publisher = Publisher
        self.Get_Pos_TASK = None
        self.Get_Bus_TASK = None
        
    def Get_Pos_Vel(self):
        msg = can.Message(arbitration_id= (self.ID << 5 | Axis0_Get_Encoder_Estimates),is_extended_id=False,is_remote_frame=True)
        try:
            #self.Get_Pos_TASK = self.bus.send_periodic(msg, 0.01, store_task=True)
            self.bus.send(msg)
        except can.CanError:
            rospy.logwarn("Odrive Get_Bus_Cur wrong ...")
            
    def Get_Bus_Cur(self):
        msg = can.Message(arbitration_id= (self.ID << 5 | Axis0_Get_Bus_Voltage_Current),is_extended_id=False,is_remote_frame=True)
        try:
            #self.Get_Bus_TASK = self.bus.send_periodic(msg, 0.01, store_task=True)
            self.bus.send(msg)
        except can.CanError:
            rospy.logwarn("Odrive Get_Bus_Cur wrong ...")

    def Set_RPM(self,rpm):
        if(rpm > self.MaxRPM):
            rpm = self.MaxRPM
        elif(rpm < -self.MaxRPM):
            rpm = -self.MaxRPM
        float_union = UintUnion()
        float_union.float_val = rpm
        msgg = can.Message(arbitration_id= (self.ID << 5 | Axis0_Set_Input_Vel), is_extended_id=False,
                        data=[float_union.byte_vals[0], float_union.byte_vals[1], float_union.byte_vals[2], float_union.byte_vals[3], 0, 0, 0, 0])
        try:
            self.bus.send(msgg)
        except can.CanError:
            rospy.logwarn("Odrive Set_RPM wrong ...")

    def Process_Message(self,msg):
        if(msg.arbitration_id & 0x1F == Axis0_Get_Encoder_Estimates):
            if(msg.dlc==8):
                float_union = FloatUnion()
                float_union.bytes = (msg.data[3] << 24) | (msg.data[2] << 16) | (msg.data[1] << 8) | msg.data[0]
                self.motor_pos = float_union.value
                float_union.bytes = (msg.data[7] << 24) | (msg.data[6] << 16) | (msg.data[5] << 8) | msg.data[4]
                self.motor_rpm = float_union.value
                #print("POS:{},RPM:{}".format(self.motor_pos,self.motor_rpm))
                msg = Odrive_msg()
                msg.Device_ID = self.ID
                msg.Motor_rpm = self.motor_rpm
                msg.Motor_pos = self.motor_pos
                msg.Motor_voltage = self.motor_voltage
                msg.Motor_current = self.motor_current
                self.Publisher.publish(msg)
        elif(msg.arbitration_id & 0x1F == Axis0_Get_Bus_Voltage_Current):
            if(msg.dlc==8):
                float_union = FloatUnion()
                float_union.bytes = (msg.data[3] << 24) | (msg.data[2] << 16) | (msg.data[1] << 8) | msg.data[0]
                self.motor_voltage = float_union.value
                float_union.bytes = (msg.data[7] << 24) | (msg.data[6] << 16) | (msg.data[5] << 8) | msg.data[4]
                self.motor_current = float_union.value
                #print("V:{},C:{}".format(self.motor_voltage,self.motor_current))
