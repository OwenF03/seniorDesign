#usage : {packet length} 
from testclass import Test
import serial
import matplotlib as plt
import numpy as np
import sys

port = ""
#open serial port
if(len(sys.argv) == 2):
    port = sys.argv[1]
else: #default
    port = '/dev/ttyACM0'

dut = Test(port) 
num = 512
    
dut.recieve_data(num)
dut.plotAllSensors()
#dut.print_voltages()


