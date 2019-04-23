import serial
import struct

ser = serial.Serial('/dev/cu.SLAB_USBtoUART', 115200)

while(True):
    try:
        value = float(input('Value to send:'))

        # Ping test
        print('###')
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print('Pinging ESP32')
        ser.write(b'\x00')
        length = ser.read(1)[0]
        print('Response is ' + str(length) + ' bytes long')
        data = ser.read(length-1)
        if data == b'\x42':
            print('Ping OK')
        else:
            print('Ping Failed')
        print('###')

        # Store float test
        print('###')
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print('Storing a float')
        bytes = struct.pack('f', value)
        ser.write(b'\x01' + bytes)
        length = ser.read(1)[0]
        print('Response is ' + str(length) + ' bytes long')
        data = ser.read(length-1)
        if data[0] == 0x12 and data[1] == 0x00:
            print('Storing OK')
        else:
            print('Storing Failed')
        print('###')

        # Get float test
        print('###')
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print('Getting a float')
        ser.write(b'\x02')
        length = ser.read(1)[0]
        print('Response is ' + str(length) + ' bytes long')
        data = ser.read(length-1)
        if data[0] == 0x12 and data[1] == 0x00:
            print('Getting OK')
        else:
            print('Getting Failed')
        print('###')

        # Bad command test
        print('###')
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print('Sending garbage')
        ser.write(b'\x693875')
        length = ser.read(1)[0]
        print('Response is ' + str(length) + ' bytes long')
        data = ser.read(length-1)
        if data[0] == 0x12 and data[1] == 0x01:
            print('Garbage OK')
        else:
            print('Garbage Failed')
        print('###')

    except ValueError:
        print('Not a number!')