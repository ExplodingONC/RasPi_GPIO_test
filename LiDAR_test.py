# import system modules
import errno
import os
import sys
import time
import datetime
# import project modules
import ctypes
import numpy
import scipy.constants
# import I/O modules
import RPi.GPIO as GPIO
import smbus2

# parameters
extCDLL = ctypes.CDLL("/home/ExplodingONC/Projects_Python/direct_GPIO.so")
width = int(96)  # not include header pixel
height = int(72)
pixel_count = (width + 1) * height

# timestamp
date = datetime.datetime.now().astimezone()
print(date.strftime("%Y-%m-%d %H:%M:%S.%f %Z %z"))

# GPIO register hook
if extCDLL.GPIO_reg_setup():
    print("GPIO hook failed!")
    sys.exit()
else:
    print("GPIO register hooked.")

# setup GPIOs
try:
    # GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17, GPIO.OUT)  # reset lock (N)
    GPIO.setup(27, GPIO.OUT)  # partial reset (N)
    GPIO.setup(22, GPIO.OUT)  # master reset (N)
except Exception as err:
    print("Error:", err)
    print("GPIO initialization failed!")
    sys.exit()
else:
    print("GPIO initialized.")

# setup IIC bus
try:
    i2c_channel = 1
    i2c_address_lidar = 0x2A
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

# reset FIFO
GPIO.output(22, 0)
time.sleep(0.01)
GPIO.output(22, 1)
time.sleep(0.01)

# LiDAR setup through IIC
try:
    i2c1.write_byte_data(i2c_address_lidar, 0x00, 0b11100011)  # stop operation
    i2c1.write_byte_data(i2c_address_lidar, 0x08, 0x00)  # ext_reset
    i2c1.write_byte_data(i2c_address_lidar, 0x09, 0x40)
    i2c1.write_byte_data(i2c_address_lidar, 0x0A, 0x00)  # H_pixel_num
    i2c1.write_byte_data(i2c_address_lidar, 0x0B, 0x60)
    i2c1.write_byte_data(i2c_address_lidar, 0x0C, 0x00)  # V_pixel_num
    i2c1.write_byte_data(i2c_address_lidar, 0x0D, 0x48)
    i2c1.write_byte_data(i2c_address_lidar, 0x0E, 0x25)  # HST_offset
    i2c1.write_byte_data(i2c_address_lidar, 0x0F, 0b00110000)  # light_pattern
    i2c1.write_byte_data(i2c_address_lidar, 0x10, 0xFF)  # frame blanking
    i2c1.write_byte_data(i2c_address_lidar, 0x11, 0x00)
    i2c1.write_byte_data(i2c_address_lidar, 0x12, 0x08)  # ADC_delay_cfg
    i2c1.write_byte_data(i2c_address_lidar, 0x13, 0b01000000)  # LV_delay, Nlight
    i2c1.write_byte_data(i2c_address_lidar, 0x14, 0x80)
    i2c1.write_byte_data(i2c_address_lidar, 0x15, 0x00)
    i2c1.write_byte_data(i2c_address_lidar, 0x16, 0x01)  # Ndata
    i2c1.write_byte_data(i2c_address_lidar, 0x17, 0xFF)  # VTX1
    i2c1.write_byte_data(i2c_address_lidar, 0x18, 0xFF)  # VTX2
    i2c1.write_byte_data(i2c_address_lidar, 0x19, 0x00)  # VTX3
    i2c1.write_byte_data(i2c_address_lidar, 0x1A, 0x04)
    i2c1.write_byte_data(i2c_address_lidar, 0x1B, 0xFF)  # light_pulse_width
    i2c1.write_byte_data(i2c_address_lidar, 0x1D, 0x01)  # light_pulse_offset
    i2c1.write_byte_data(i2c_address_lidar, 0x1F, 0x7F)  # P4_delay
    i2c1.write_byte_data(i2c_address_lidar, 0x20, 0b00001001)  # L/A, Light_pulse_half_delay, H_pixel_blanking
    i2c1.write_byte_data(i2c_address_lidar, 0x21, 0x00)  # T1 (linear only)
    i2c1.write_byte_data(i2c_address_lidar, 0x22, 0x00)  # PHIS (linear only)
    i2c1.write_byte_data(i2c_address_lidar, 0x23, 0x00)  # T2 (linear only)
    i2c1.write_byte_data(i2c_address_lidar, 0x24, 0b00001111)  # timing signal enable
    i2c1.write_byte_data(i2c_address_lidar, 0x00, 0b11000011)  # start clock divider
    i2c1.write_byte_data(i2c_address_lidar, 0x00, 0b10000011)  # start clock
    i2c1.write_byte_data(i2c_address_lidar, 0x00, 0b00000011)  # start operation
    time.sleep(0.01)
except OSError as err:
    print(" -", "OSError", err)
    if err.errno == 121:
        print(" - I2C: No response from device! Check wiring on GPIO2/3.")
    sys.exit()
except Exception as err:
    print("Error:", err)
    print(" - I2C unknown error!")
    sys.exit()
else:
    print(" - I2C data sent.")


# GPIO frame read (through FIFO)
time.sleep(0.25)
GPIO.output(17, 0)  # disable FIFO reset signal
time_start = time.perf_counter_ns()
extCDLL.read_area_frame_fifo.restype = numpy.ctypeslib.ndpointer(dtype=ctypes.c_uint32, shape=(4, height, 2 * (width + 1)))
data_stream = extCDLL.read_area_frame_fifo(ctypes.c_int(width + 1), ctypes.c_int(height))
time_end = time.perf_counter_ns()
print("Total time of %dx%d pixels: %fs." % (width + 1, height, 1e-9 * (time_end - time_start)))
GPIO.output(17, 1)  # enable FIFO reset

# LiDAR data processing
depth_map = numpy.zeros((height, width), dtype=int, order='C')
# slicing
data_F1_Ch1 = data_stream[0, :, 0::2]
data_F1_Ch2 = data_stream[0, :, 1::2]
data_F2_Ch1 = data_stream[1, :, 0::2]
data_F2_Ch2 = data_stream[1, :, 1::2]
data_F3_Ch1 = data_stream[2, :, 0::2]
data_F3_Ch2 = data_stream[2, :, 1::2]
data_F4_Ch1 = data_stream[3, :, 0::2]
data_F4_Ch2 = data_stream[3, :, 1::2]
# calculating
for y in range(0, height):
    for x in range(0, width):
        dif_F1 = data_F1_Ch1[y, x + 1] - data_F1_Ch2[y, x + 1]
        dif_F2 = data_F2_Ch1[y, x + 1] - data_F2_Ch2[y, x + 1]
        if dif_F1 >= 0:
            depth_map[y, x] = (dif_F2 / (abs(dif_F1) + abs(dif_F2)) + 1) / 4 * scipy.constants.c * 4.25e-6
        else:
            depth_map[y, x] = (-dif_F2 / (abs(dif_F1) + abs(dif_F2)) + 3) / 4 * scipy.constants.c * 4.25e-6
        # WIP, lacks calibration process now (F3-F4)
print(depth_map)

# GC
GPIO.cleanup()
sys.exit(0)
