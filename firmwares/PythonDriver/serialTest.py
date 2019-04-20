import serial
import struct

ser = serial.Serial('/dev/cu.SLAB_USBtoUART', 115200)

while(True):
    # data = ser.readline()
    # print(data.decode('utf-8'))
    try:
        value = float(input('Value to send:'))
        byte_array = bytearray(struct.pack('f', value))
        written = ser.write(byte_array)
        print(written)

    except ValueError:
        print('Not a number!')