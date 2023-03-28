import can
import rospy
import struct
from can_communication.msg import *

CAN_PACKET_SET_DUTY   = 0x00
CAN_PACKET_SET_RPM    = 0x03
CAN_PACKET_STATUS_1   = 0x09
CAN_PACKET_STATUS_5   = 0x1B

VESC_ID_1 = 0x68
VESC_ID_2 = 0x69

class VESC_Motor:
    def __init__(self ,id ,bus ,Publisher):
        self.ID = id
        self.bus = bus
        self.motor_rpm = 0
        self.motor_duty = 0.0
        self.motor_voltage = 0.0
        self.motor_current = 0.0
        self.SetRPMTASK = None
        self.SetDutyTASK = None
        self.MaxRPM = 12000
        self.MaxDuty = 0.2
        self.Publisher = Publisher


    def SetRPM(self,rpm):
        if(rpm > self.MaxRPM):
            rpm = self.MaxRPM
        elif(rpm < -self.MaxRPM):
            rpm = -self.MaxRPM
        msg = can.Message(arbitration_id= (CAN_PACKET_SET_RPM << 8 | self.ID),
                        data=[rpm >> 24 & 0xFF, rpm >> 16 & 0xFF, rpm >> 8 & 0xFF, rpm & 0xFF])
        # rospy.loginfo("Send msg:{}".format(msg.data))
        try:
            #self.SendTASK = self.bus.send_periodic(msg, 0.01, store_task=True)
            self.bus.send(msg)
        except can.CanError:
            rospy.logwarn("VESC_Motor ID:{} SetRPM wrong ...".format(self.ID))
        return rpm

    def SetDuty(self,duty):
        if(duty > self.MaxDuty):
            duty = self.MaxDuty
        elif(duty < -self.MaxDuty):
            duty = -self.MaxDuty
        tem_data = int(duty * 100000);
        msg = can.Message(arbitration_id= (CAN_PACKET_SET_DUTY << 8 | self.ID),
                        data=[tem_data >> 24 & 0xFF, tem_data >> 16 & 0xFF, tem_data >> 8 & 0xFF, tem_data & 0xFF])
        # rospy.loginfo("Send msg:{}".format(msg.data))
        try:
            self.SendTASK = self.bus.send_periodic(msg, 0.01, store_task=True)
        except can.CanError:
            rospy.logwarn("VESC_Motor ID:{} Setduty wrong ...".format(self.ID))
        return duty

    def Process_Message(self,msg):
        if   (msg.arbitration_id >> 8 & 0xFF == CAN_PACKET_STATUS_1):
            if(msg.dlc == 8):
                # Convert to C int32
                rpm_uint32 = msg.data[0] << 24 | msg.data[1] << 16 | msg.data[2] << 8 | msg.data[3]
                rpm_int32 = struct.pack('I', rpm_uint32)
                self.motor_rpm     = struct.unpack('i', rpm_int32)[0]
                
                # Convert to C int16
                rpm_uint16 = msg.data[4] << 8 | msg.data[5]
                rpm_int16 = struct.pack('H', rpm_uint16)
                self.motor_current = float(struct.unpack('h', rpm_int16)[0])/10.0

                # Convert to C int16
                rpm_uint16 = msg.data[6] << 8 | msg.data[7]
                rpm_int16 = struct.pack('H', rpm_uint16)
                self.motor_duty = float(struct.unpack('h', rpm_int16)[0])/1000.0

                # rospy.loginfo("VESC_Motor ID:{} motor_rpm:{} motor_current:{} motor_duty:{}".format(self.ID,self.motor_rpm,self.motor_current,self.motor_duty))
                if(self.ID == VESC_ID_1):
                    msg = Bicycle()
                    msg.Device_ID = self.ID
                    msg.Motor_rpm = self.motor_rpm
                    msg.Motor_duty = self.motor_duty
                    msg.Bicycle_voltage = self.motor_voltage
                    self.Publisher.publish(msg)
                elif(self.ID == VESC_ID_2):
                    msg = Bicycle()
                    msg.Device_ID = self.ID
                    msg.Motor_rpm = self.motor_rpm
                    msg.Motor_duty = self.motor_duty
                    self.Publisher.publish(msg)
            else:
                rospy.logwarn("VESC_Motor ID:{} CAN_STATUS_1 DLC wrong ...".format(self.ID))
        elif (msg.arbitration_id >> 8 & 0xFF == CAN_PACKET_STATUS_5):
            if(msg.dlc == 8):
                self.motor_voltage = float(msg.data[6] << 4 | msg.data[5])/1e1
                #rospy.loginfo("Bicycle_voltage:{}".format(self.motor_voltage))
            else:
                rospy.logwarn("VESC_Motor ID:{} CAN_STATUS_5 DLC wrong ...".format(self.ID))
