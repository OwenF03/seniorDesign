from testclass import Test 
import serial
import matplotlib as plt
import numpy as np
import struct 
import sys

def build_dt_msg(numElements):
    msg_bytes = b"dat"
    data_bytes = struct.pack("!I", numElements) 
    msg_bytes += data_bytes 
    return msg_bytes + b"0"


port = ""
#open serial port
if(len(sys.argv) == 2):
    port = sys.argv[1]
else: #default
    port = '/dev/ttyACM0'

dut = Test(port) 

msg = build_dt_msg(64)

#dut.send_msg("nullmsgx".encode("utf-8"))
dut.send_msg(msg) 
#dut.send_msg("calxxxxx".encode("utf-8"))
dut.recieve_data(64)
