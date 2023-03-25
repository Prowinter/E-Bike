import can

class CAN_Servo:
    def __init__(self, bus):
        self.bus = bus
        self.angle = 0
        self.Min_Angle = 80
        self.MAX_Angle = 80
        self.GetAngle_periodic()
        self.SendStatus = 0
        self.SendTASK = None
        self.MaxAngle = 3148
        self.MinAngle = 948

    def __del__(self):
        self.rotation_speed = 1

    def GetAngle_periodic(self):
        msg = can.Message(arbitration_id=0x0E,
                data=[0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                is_extended_id=False)
        self.bus.send_periodic(msg, 1, duration=None, store_task=True)

    def Process_Message(self,msg):
        if msg.data[0] == 0x02:
            low_byte  = msg.data[1] 
            high_byte = msg.data[2] & 0x0F
            self.angle = ((high_byte << 8 )  | low_byte)
            print("Angle:", self.angle)
        elif msg.data[0] == 0x04:
            if msg.data[2] == 0xAA:
                self.SendStatus = 1
                self.SendTASK.stop()
                print("Send OK")
            else:
                print("Send ERROR")

    def GetCurrentAngle(self):
        msg = can.Message(arbitration_id=0x0E,
                        data=[0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                        is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanError:
            print("Something went wrong ...")

    def RotationAngle(self,angle,speed=5):
        if(angle > MaxAngle):
            angle = MaxAngle
        elif(angle < -MinAngle):
            angle = -MinAngle
        angle_low_byte  = angle & 0xFF
        angle_high_byte = (angle >> 8) & 0x0F
        speed_low_byte  = speed & 0x1F
        speed_high_byte = (speed >> 8) & 0x00
        msg = can.Message(arbitration_id=0x0E,
                        data=[0x03, angle_low_byte, angle_high_byte, speed_low_byte, speed_high_byte, 0x00, 0x00, 0x00],
                        is_extended_id=False)
        print("Send msg:{}".format(msg.data))
        try:
            self.SendTASK = self.bus.send_periodic(msg, 1, duration=5, store_task=True)
        except can.CanError:
            print("Something went wrong ...")
        return angle

    def GetServoID(self):
        msg = can.Message(arbitration_id=0x0E,
                        data=[0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                        is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanError:
            print("Something went wrong ...")

