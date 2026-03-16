from testclass import Test 
import serial
import matplotlib as plt
import numpy as np
import struct 
import sys
import time

NUMSAMPLES = 1024
NUM_SENSORS = 4
def build_dt_msg(numElements):
    msg_bytes = b"dat"
    data_bytes = struct.pack("<I", numElements) 
    msg_bytes += data_bytes 
    return msg_bytes + b"0"

# Expects value and channel to be 8 bit values 
def build_spi_msg(value, channel): 
    msg_bytes = b"spi"
    data_bytes = struct.pack("<BB", value, channel)
    msg_bytes += data_bytes + b"000"
    return msg_bytes

def build_ech_msg():
    msg_bytes = b"ech"
    data_bytes = struct.pack("<I", 0)
    msg_bytes += data_bytes
    return msg_bytes + b"0"

def build_txt_msg(msg):
    msg_bytes = bytes(msg, "utf-8")
    if(len(msg_bytes) < 8):
        for i in range(len(msg_bytes), 8):
            msg_bytes = msg_bytes + b"0"
    return msg_bytes 

port = ""
#open serial port
if(len(sys.argv) == 2):
    port = sys.argv[1]
else: #default
    port = '/dev/ttyACM0'

dut = Test(port, NUMSENSORS=NUM_SENSORS) 

#msg = build_ech_msg()
#msg = build_txt_msg()

#dut.send_msg("nullmsgx".encode("utf-8"))
#dut.send_msg(msg) 
#dut.send_msg("calxxxxx".encode("utf-8"))
while True:
    print("[L] Enter Command : ", end="")
    cmd = input() 
    cmd = cmd.lower()
    if (cmd == "sta"):
        msg = build_txt_msg("sta")  
        dut.send_msg(msg)
        resp = dut.recieve_ack()
        print("\t[M] " + str(struct.unpack("8s", resp[0])[0])) 
        # Wait for calculated value 
        resp = dut.recieve_ack(5) 
        # Extract 2 shorts
        for i in resp:
            print(i)
        unpacked = struct.unpack("<3bh", resp[0])
        print("\t[L] Result received : " + str(np.frombuffer(resp[0], '<i2', offset=3)))

        pass;
    elif (cmd == "stac"):
        msg = build_txt_msg("sta")  
        dut.send_msg(msg)
        resp = dut.recieve_ack()
        print("\t[M] " + str(struct.unpack("8s", resp[0])[0])) 
        # Wait for calculated value 

        test_duration = 5.0  # Time to measure in seconds
        byte_count = 0
        start_time = time.time()
        
        print(f"Starting measurement for {test_duration} seconds...")

        # --- MEASUREMENT LOOP ---
        while (time.time() - start_time) < test_duration:
            # Request/Read data (assuming this gets 7 bytes based on your unpack logic)
            resp = dut.recieve_ack(7) 
            
            # Check if resp has data
            if resp and len(resp) > 0:
                raw_data = resp[0]
                # 1. Accumulate Byte Count
                byte_count += len(raw_data)

        elapsed = time.time() - start_time
        print("-" * 30)
        print(f"Time Elapsed: {elapsed:.4f} s")
        print(f"Total Bytes:  {byte_count}")
        print(f"Throughput:   {byte_count / elapsed:.2f} Bytes/sec")
        print("-" * 30)
    elif (cmd == "stp"):
        msg = build_txt_msg("stp")
        dut.send_msg(msg)
        print("[L] Sent Stop Command")
    elif (cmd == "spi"):
        print("\t[L] Enter (value, channel) : ", end="")
        vals = input()
        vals = vals.replace("(", '')
        vals = vals.replace(")", '')
        vals = vals.split(",")
        if(len(vals) != 2) : 
            print("\t[L] Invalid format, expecting (value, channel)")
            continue
        msg = build_spi_msg(np.uint8(int(vals[0])), np.uint8(int(vals[1]))) 
        dut.send_msg(msg)
        resp = dut.recieve_ack()
        print("\t[M] " + str(struct.unpack("8s", resp[0])[0])) 
    elif (cmd == "dat"):
        print(f"\t[L] Enter Data Transfer Size (Max {NUMSAMPLES}) : ", end="")
        val = input()
        print(f"\t[L] Enter File Name For Log : ", end="")
        fn = input()
        msg = build_dt_msg(int(val))
        dut.send_msg(msg)
        resp = dut.recieve_ack()
        print(f"\t[M] " + str(struct.unpack("8s", resp[0])[0]))
        data = dut.recieve_data(int(val))
        if(fn != ""):
            for i in data:
                print(i)
            np.save(fn, np.array(data)) 
        else:
            dut.plotAllSensors() 
    elif (cmd == "sfr"):
        pass;
        print("[L] Unrecognized Command : ")
    elif (cmd == "tspi"):
        print("\t[L] Enter value,channel : ", end="")
        vals = input()
        vals = vals.replace("(", '')
        vals = vals.replace(")", '')
        vals = vals.split(",")
        if(len(vals) != 2) : 
            print("\t[L] Invalid format, expecting (value, channel)")
            continue
        msg = build_spi_msg(np.uint8(int(vals[0])), np.uint8(int(vals[1]))) 
        dut.send_msg(msg)
        resp = dut.recieve_ack()
        msg = build_dt_msg(NUMSAMPLES)
        time.sleep(0.0001)
        dut.send_msg(msg)
        resp = dut.recieve_ack()
        data = dut.recieve_data(NUMSAMPLES)
        for i in data:
            print(i)
        np.save("spi" + str(vals[0]), np.array(data)) 


    elif (cmd == "exit"):
        print("Exiting Program")
        exit()

    #resp = dut.recieve_ack()
    #print(resp)
