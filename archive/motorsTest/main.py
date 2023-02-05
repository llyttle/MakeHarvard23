import serial

def setup():
    # list(serial.tools.list_ports.comports())
    ser = serial.Serial('COM11',write_timeout=2)
    print(ser.name)
    ser.baudrate = 9600  # set Baud rate to 9600
    ser.bytesize = 8     # Number of data bits = 8
    ser.parity   ='N'    # No parity
    ser.stopbits = 1     # Number of Stop bits = 1
    ser.flushInput()    
    return ser

def txTeensy(ser):
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(b'1')

# def rxTeensy(ser):
#     ser.open()
#     if ser:
#         try:
#             print(ser.read())
#             print(ser.dev.readline().decode().rstrip())
#         except:
#             pass
#     ser.close()

if __name__ == "__main__":
    ser = setup()
    txTeensy(ser)
    # rxTeensy(ser)

