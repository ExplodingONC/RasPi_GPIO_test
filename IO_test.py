# import system modules
import errno
import os
import sys
import time
import datetime
# import project modules
import ctypes
import numpy
# import I/O modules
import RPi.GPIO as GPIO
import smbus2

# parameters
use_i2c = False
extCDLL = ctypes.CDLL("/home/ExplodingONC/Projects_Python/direct_GPIO.so")

# timestamp
date = datetime.datetime.now().astimezone()
print(date.strftime("%Y-%m-%d %H:%M:%S.%f %Z %z"))

# GPIO register hook
if extCDLL.GPIO_reg_setup():
    print("GPIO hook failed!")
    sys.exit()
else:
    print("GPIO register hooked.")

# setup IIC bus
try:
    i2c_channel = 1
    i2c_address_lidar = 0x2A
    lidar_reg_enable = 0x24
    i2c1 = smbus2.SMBus(i2c_channel)
except FileNotFoundError as err:
    print("FileNotFoundError", err)
    if err.errno == 2:
        print("I2C not enabled. Check raspi-config.")
    sys.exit()
except Exception as err:
    print("Error:", err)
    print("I2C initialization failed!")
    sys.exit()
else:
    print("I2C initialized.")

# setup GPIOs
try:
    # GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(17, GPIO.OUT)
except Exception as err:
    print("Error:", err)
    print("GPIO initialization failed!")
    sys.exit()
else:
    print("GPIO initialized.")

# just put it here
time.sleep(0.01)

try:
    
    # GPIO manipulation
    for x in range(1, 11):
        # write GPIO
        GPIO.output(17, x % 2)
        print("test %3d:" % (x), end=' ')
        # read GPIO
        current_val = GPIO.input(17)
        if current_val:
            print("Pin 17 is HIGH")
        else:
            print("Pin 17 is LOW")
        # relay results
        if use_i2c:
            try:
                i2c1.write_byte_data(i2c_address_lidar, lidar_reg_enable, current_val)
            except OSError as err:
                print(" -", "OSError", err)
                if err.errno == 121:
                    print(" - I2C: No response from device! Check wiring on GPIO2/3.")
            except Exception as err:
                print("Error:", err)
                print(" - I2C unknown error!")
            else:
                print(" - I2C data sent.")
        # loop delay
        # time.sleep(0.01)

    GPIO.output(17, 1)
    # GPIO frame read
    width = int(960)
    height = int(720)
    time_start = time.perf_counter_ns()
    extCDLL.read_area_frame.restype = numpy.ctypeslib.ndpointer(dtype=ctypes.c_uint32, shape=(4, height, 2 * (width + 1)))
    data_stream = extCDLL.read_area_frame(ctypes.c_int(width + 1), ctypes.c_int(height))
    time_end = time.perf_counter_ns()
    print("GPIO burst read result:\r\n{0:b}".format(data_stream[0, 0, 0], data_stream[0, 0, 1]))
    print("Total time of %dx%d pixels and x8 reads: %fs, aka %5.3fMHz."
        % (width, height, 1e-9 * (time_end - time_start), 1e3 * (8 * (width + 1) * height) / (time_end - time_start)))
    del data_stream

    # GPIO frame read (through FIFO)
    width = int(960)
    height = int(720)
    time_start = time.perf_counter_ns()
    extCDLL.read_area_frame_fifo.restype = numpy.ctypeslib.ndpointer(dtype=ctypes.c_uint32, shape=(4, height, 2 * (width + 1)))
    data_stream = extCDLL.read_area_frame_fifo(ctypes.c_int(width + 1), ctypes.c_int(height))
    time_end = time.perf_counter_ns()
    print("GPIO fifo burst result:\r\n{0:b}".format(data_stream[0, 0, 0], data_stream[0, 0, 1]))
    print("Total time of %dx%d pixels and x8 reads: %fs, aka %5.3fMHz."
        % (width, height, 1e-9 * (time_end - time_start), 1e3 * (8 * (width + 1) * height) / (time_end - time_start)))
    del data_stream

    # GPIO burst read
    repeat_num = int(1e6)
    time_start = time.perf_counter_ns()
    extCDLL.burst_read.restype = numpy.ctypeslib.ndpointer(dtype=ctypes.c_uint32, shape=(repeat_num,))
    data_stream = extCDLL.burst_read(ctypes.c_int(repeat_num))
    time_end = time.perf_counter_ns()
    print("GPIO burst read result:\r\n{0:b}".format(data_stream[0], data_stream[1]))
    print("Total time of %d reads: %fs, aka %5.3fMHz."
        % (repeat_num, 1e-9 * (time_end - time_start), 1e3 * repeat_num / (time_end - time_start)))
    del data_stream

    # GPIO single read
    repeat_num = int(1e5)
    data_stream = numpy.zeros(repeat_num, dtype=int, order='C')
    time_start = time.perf_counter_ns()
    for x in range(0, repeat_num):
        data_stream[x] = extCDLL.single_read()
        data_stream[x] = extCDLL.single_read()
    time_end = time.perf_counter_ns()
    print("GPIO query read result:\r\n{0:b}".format(data_stream[0], data_stream[1]))
    print("Total time of %d reads: %fs, aka %5.3fMHz."
        % (repeat_num, 1e-9 * (time_end - time_start), 1e3 * repeat_num / (time_end - time_start)))
    del data_stream

    # GPIO single read
    repeat_num = int(1e4)
    data_stream = numpy.zeros(repeat_num, dtype=int, order='C')
    time_start = time.perf_counter_ns()
    for x in range(0, repeat_num):
        for y in range(0, 12):
            data_stream[x] |= GPIO.input(17) << y
            data_stream[x] |= GPIO.input(17) << y
    time_end = time.perf_counter_ns()
    print("RPi.GPIO read result:\r\n{0:b}".format(data_stream[0], data_stream[1]))
    print("Total time of %d reads: %fs, aka %5.3fkHz."
        % (repeat_num, 1e-9 * (time_end - time_start), 1e6 * repeat_num / (time_end - time_start)))
    del data_stream

finally:

    # GC
    GPIO.cleanup()
    sys.exit(0)
