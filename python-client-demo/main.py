#!/usr/bin/env python3

import serial
from cobs import cobs
from crc import CrcCalculator, Crc32

import api_pb2 as api
import error_pb2 as api_error
import v1.v1_pb2 as api_v1

def hex_str(input):
    return " ".join(hex(n) for n in input)

api_request = api.Message()
api_request.v1_request.getInfo.SetInParent()
print("compiling message:   ", api_request)

data_to_send = api_request.SerializeToString()

print("raw data to send:   ", hex_str(data_to_send))

crc_calculator = CrcCalculator(Crc32.CRC32)
checksum = crc_calculator.calculate_checksum(data_to_send)
checksum = checksum.to_bytes(4, 'big')
data_to_send += checksum
print("crc'd data to send: ", hex_str(data_to_send))

data_to_send = cobs.encode(data_to_send)
print("cobs data to send:  ", hex_str(data_to_send + b'\0'))
print("---")

data_received = b''

# ttyACM0 is probably your programming interface
ser = serial.Serial('/dev/ttyACM1', 19200, timeout=1)

writing_done = False
while True:
    iw = ser.in_waiting
    if iw:
        ib = ser.read(iw)
        data_received += ib
        if not ib.find(b'\x00') == -1:
            break

    if not writing_done:
        b = data_to_send[0:10]
        data_to_send = data_to_send[10:]
        if b:
            ser.write(b)
        else:
            l = ser.write(b'\x00')
            writing_done = True

print("raw data received:  ", hex_str(data_received))
data_received = cobs.decode(data_received[:-1])
print("un-cobs received:   ", hex_str(data_received))
message_crc = data_received[-4:]
data_received = data_received[:-4]
print("CRC received        ", hex_str(message_crc))
crc_calculator = CrcCalculator(Crc32.CRC32)
checksum = crc_calculator.calculate_checksum(data_received)
checksum = checksum.to_bytes(4, 'big')
print("CRC expected        ", hex_str(checksum))
print("CRC match           ", message_crc == checksum)
print("un-crcd received:   ", hex_str(data_received))
# print("received            ", data_received.decode())
response = api.Message().FromString(data_received)
print("received:           ", response)