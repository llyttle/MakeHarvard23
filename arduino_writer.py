import serial
import serial.tools.list_ports as list_ports
import time,rclpy
import math


class Serial_cmd:
    Arduino_IDs = ((0x2341, 0x0043), (0x2341, 0x0001), 
                   (0x2A03, 0x0043), (0x2341, 0x0243), 
                   (0x0403, 0x6001), (0x1A86, 0x7523),
                   (0x9025, 0x0067))
    
    def __init__(self, port=''):
        print("test abc")
        if port == '':
            self.dev = None
            self.connected = False
            devices = list_ports.comports()
            for device in devices:
                print(device)
                print(type(device))
                print(device.vid)
                print(type(device.vid))
                if device.vid == 9025:
                    print("before try")
                    try:
                        self.dev = serial.Serial(device.device, 115200)
                        self.connected = True
                        print('Connected to {!s}...'.format(device.device))
                    except:
                        print("an exception occurred :(")
                if self.connected:
                    break
        else:
            try:
                self.dev = serial.Serial(port, 9600)
                self.connected = True
            except:
                self.edev = None
                self.connected = False
    def write_data_to_arduino(self, string_to_write):
        if self.connected:
            self.dev.write(string_to_write.encode())
        else:
            raise Exception("self not connected!")
    def read_data(self):
        if self.connected:
            try:
                print(self.dev.readline().decode().rstrip())
            except:
                pass

def main(args=None):
    writer = Serial_cmd()
    print_debugging = True
    print("initializing serial connection")
    if not writer.connected:
        raise Exception("error connecting")

    #constants:
    theta_threshold = math.pi / 8


    #common vars
    num_spins = 0


    # to come from Loren
    dir = "left"
    rel_theta = math.pi/4

    while True:
        #REL_THETA POS TURN LEFT
        #REL THETA NEG TURN RIGHT
        if rel_theta < theta_threshold:
            dir = "straight"
        elif rel_theta > 0:
            dir = "left"
        else:
            dir = "right"
        if print_debugging:
            print(f"Direction: {dir}")

        match dir:
            case "left":
                left_motor_speed = 0
                right_motor_speed 130
            case "right":
                left_motor_speed = 120
                right_motor_speed 0
            case "straight":
                left_motor_speed = 150
                right_motor_speed 120
            case _:
                raise Exception("Invalid direction.")
        to_print = ""
        string_to_write = f"<{left_motor_speed},{right_motor_speed}\n"
        writer.write_data_to_arduino(string_to_write)
        to_print += string_to_write
        if print_debugging:
            print(to_print)
        while writer.dev.in_waiting:
            writer.read_data()
        time.sleep(0.08)
        
        
    

if __name__ == '__main__':
    main()
