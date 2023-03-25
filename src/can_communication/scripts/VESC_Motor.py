import can

CAN_PACKET_SET_DUTY   = 0x00
CAN_PACKET_SET_RPM    = 0x03
CAN_PACKET_STATUS_1   = 0x09
CAN_PACKET_STATUS_5   = 0x1B

class VESC_Motor:
    def __init__(self,id,bus):
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


    def SetRPM(self,rpm):
        if(rpm > self.MaxRPM):
            rpm = self.MaxRPM
        elif(rpm < -self.MaxRPM):
            rpm = -self.MaxRPM
        msg = can.Message(arbitration_id= (CAN_PACKET_SET_RPM << 8 | self.ID),
                        data=[rpm >> 24 & 0xFF, rpm >> 16 & 0xFF, rpm >> 8 & 0xFF, rpm & 0xFF])
        print("Send msg:{}".format(msg.data))
        try:
            self.SendTASK = self.bus.send_periodic(msg, 1, duration=5, store_task=True)
        except can.CanError:
            print("VESC_Motor ID:{} SetRPM wrong ...".format(self.ID))
        return rpm

    def SetDuty(self,duty):
        if(duty > self.MaxDuty):
            duty = self.MaxDuty
        elif(duty < -self.MaxDuty):
            duty = -self.MaxDuty
        tem_data = int(duty * 100000);
        msg = can.Message(arbitration_id= (CAN_PACKET_SET_DUTY << 8 | self.ID),
                        data=[tem_data >> 24 & 0xFF, tem_data >> 16 & 0xFF, tem_data >> 8 & 0xFF, tem_data & 0xFF])
        print("Send msg:{}".format(msg.data))
        try:
            self.SendTASK = self.bus.send_periodic(msg, 1, duration=5, store_task=True)
        except can.CanError:
            print("VESC_Motor ID:{} Setduty wrong ...".format(self.ID))
        return duty

    def Process_Message(self,msg):
        if   (msg.arbitration_id >> 8 & 0xFF == CAN_PACKET_STATUS_1):
            if(msg.dlc == 8):
                self.motor_rpm     = msg.data[0] << 24 | msg.data[1] << 16 | msg.data[2] << 8 | msg.data[3]
                self.motor_current = float(msg.data[4] << 8 | msg.data[5])/10.0
                self.motor_duty    = float(msg.data[6] << 8 | msg.data[7])/1000.0
                print("VESC_Motor ID:{} motor_rpm:{} motor_current:{} motor_duty:{}".format(self.ID,self.motor_rpm,self.motor_current,self.motor_duty))
            else:
                print("VESC_Motor ID:{} CAN_STATUS_1 DLC wrong ...".format(self.ID))
        elif (msg.arbitration_id >> 8 & 0xFF == CAN_PACKET_STATUS_5):
            if(msg.dlc == 8):
                self.motor_voltage = float(msg.data[6] << 8 | msg.data[7])/1.0
            else:
                print("VESC_Motor ID:{} CAN_STATUS_5 DLC wrong ...".format(self.ID))