import serial
import struct
import matplotlib.pyplot as plt
import numpy as np

# Class wrapper for testing MUSIC algorithm implementation on STM 32 F411CEU6 MCU
# Communication is achieved over UART
# TODO : Actual error checking would probably be worthwhile
class Test: 
    SIZEFLOAT = 4 # size of float in bytes
    NUMSENSORS = 4
    m_conn = serial.Serial() 
    m_voltage = np.zeros((NUMSENSORS, 0))
    def __init__(self, port, baud = 115200, NUMSENSORS = 4):
        self.NUMSENSORS = NUMSENSORS
        self.m_conn.port = port
        self.m_conn.baudrate = baud 
        self.m_conn.open()
        self.m_conn.timeout = 1

    #Send a start command (binary 1, tbd if final value) 
    def start_transfer(self):
        startCmd = bytearray() 
        startCmd.append(0x01)
        with self.m_conn as sh:
            sh.write(startCmd)

    # Populate a numpy array containing num items recieved
    # from open serial port
    def recieve_data(self, num): 
        ret = np.zeros(num * self.NUMSENSORS)
        readSize = (num * self.NUMSENSORS * self.SIZEFLOAT) 
        #Each sensor reading is four floats (4 sensors, 4 bytes per float)
        readVals = []
        self.start_transfer(); 
        with self.m_conn as sh:
            readVals = sh.read(readSize)  
        #iterate through byte array by four 
        for i in range(0, num * self.NUMSENSORS * self.SIZEFLOAT, 4):
            ret[i // 4] = struct.unpack('f', readVals[i : i + 4])[0] #Construct float from read values (bytearray)
        
        self.m_voltage.resize((self.NUMSENSORS, num))
        # reorder array so that the sensor voltages are in order 0, 1, 2 ...
        for i in range(0, self.NUMSENSORS):
            for j in range(i, ret.size, self.NUMSENSORS):
                self.m_voltage[i][j // 4] = ret[j]


    def print_voltages(self):
        for i in range(0, self.NUMSENSORS):
            print(f"Sensor {i}")
            for j in range(0, self.m_voltage[i].size):
                print(f"{self.m_voltage[i][j]}") 

    
    #Create voltage plot for a single sensor value
    # inputs 
    # sensorNum : number of sensor to plot
    # numSamples : optional number of samples to plot, DON'T  CALL WITH > THAN ARRAY SIZE
    def m_plotSingleSensor(self, sensorNum, numSamples = None, ax = None):
        sensorName = f"Sensor {sensorNum} Values"
        if(numSamples is None):
            numSamples = self.m_voltage[sensorNum].size
        if(ax is None):
            ax = plt.axes()
        
        xAx = np.arange(self.m_voltage[sensorNum].size) + 1
        ax.plot(xAx, self.m_voltage[sensorNum][:numSamples])
        ax.set_title(sensorName)
        ax.set_xlabel("Sample")
        ax.set_ylabel("Voltage (V)")
        return ax

    def plotSignleSensor(self, sensorNum, numSamples = None):
        ax = self.m_plotSingleSensor(sensorNum, numSamples)
        fig = ax.figure
        fig.show()

    #Plot all sensors as a grid, return figure
    def plotAllSensors(self, numSamples = None):
        if(numSamples is None):
            numSamples = self.m_voltage[0].size

        cols = int(np.ceil(np.sqrt(self.NUMSENSORS)))
        rows = int(np.ceil(self.NUMSENSORS / cols))

        fig, axes = plt.subplots(rows, cols, figsize=(4 * cols, 3 * rows))
        axes = axes.flatten()
        for i,v in enumerate(axes):
            if(i >= self.NUMSENSORS):
                v.visible = False
            else:
                self.m_plotSingleSensor(i, numSamples, v)
        fig.suptitle("All Sensor Values")
        plt.show()
