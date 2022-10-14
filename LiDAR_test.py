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
import spidev

# parameters
width = int(96)  # not include header pixel
height = int(72)
pixel_count = (width + 1) * height

# timestamp
date = datetime.datetime.now().astimezone()
print(date.strftime("%Y-%m-%d %H:%M:%S.%f %Z %z"))

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

# setup SPI bus
try:
    spi_channel = 0
    spi_device_MCU = 1
    spi = spidev.SpiDev()
    spi.open(spi_channel, spi_device_MCU)
except Exception as err:
    print("Error:", err)
    print("SPI initialization failed!")
    sys.exit()
else:
    spi.max_speed_hz = 10000000
    spi.mode = 0
    print("SPI initialized.")

# setup GPIOs
try:
    # GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    pin_sensor_rst = 4
    pin_mcu_rst = 23
    GPIO.setup(pin_sensor_rst, GPIO.OUT)  # sensor reset (P)
    GPIO.setup(pin_mcu_rst, GPIO.OUT)  # MCU reset (N)
except Exception as err:
    print("Error:", err)
    print("GPIO initialization failed!")
    sys.exit()
else:
    print("GPIO initialized.")

# reset devices
GPIO.output(pin_sensor_rst, 1)
GPIO.output(pin_mcu_rst, 0)
time.sleep(0.01)
GPIO.output(pin_sensor_rst, 0)
GPIO.output(pin_mcu_rst, 1)
time.sleep(0.1)


# main program
print()
try:

    # LiDAR setup through IIC
    try:
        i2c1.write_byte_data(i2c_address_lidar, 0x00, 0b11100011)  # stop operation
        i2c1.write_byte_data(i2c_address_lidar, 0x07, 0b11000000)  # unknown?
        i2c1.write_byte_data(i2c_address_lidar, 0x08, 0x00)  # ext_reset
        i2c1.write_byte_data(i2c_address_lidar, 0x09, 0x40)
        i2c1.write_byte_data(i2c_address_lidar, 0x0A, 0x00)  # H_pixel_num
        i2c1.write_byte_data(i2c_address_lidar, 0x0B, 0x60)
        i2c1.write_byte_data(i2c_address_lidar, 0x0C, 0x00)  # V_pixel_num
        i2c1.write_byte_data(i2c_address_lidar, 0x0D, 0x48)
        i2c1.write_byte_data(i2c_address_lidar, 0x0E, 0x25)  # HST_offset
        i2c1.write_byte_data(i2c_address_lidar, 0x0F, 0b10110111)  # light_pattern
        i2c1.write_byte_data(i2c_address_lidar, 0x10, 0xFF)  # frame blanking
        i2c1.write_byte_data(i2c_address_lidar, 0x11, 0x00)
        i2c1.write_byte_data(i2c_address_lidar, 0x12, 0x08)  # ADC_delay_cfg
        i2c1.write_byte_data(i2c_address_lidar, 0x13, 0b01000000)  # LV_delay, Nlight
        i2c1.write_byte_data(i2c_address_lidar, 0x14, 0x80)
        i2c1.write_byte_data(i2c_address_lidar, 0x15, 0x00)
        i2c1.write_byte_data(i2c_address_lidar, 0x16, 0x01)  # Ndata
        i2c1.write_byte_data(i2c_address_lidar, 0x17, 0x7F)  # VTX1
        i2c1.write_byte_data(i2c_address_lidar, 0x18, 0x7F)  # VTX2
        i2c1.write_byte_data(i2c_address_lidar, 0x19, 0x00)  # VTX3
        i2c1.write_byte_data(i2c_address_lidar, 0x1A, 0x04)
        i2c1.write_byte_data(i2c_address_lidar, 0x1B, 0x7F)  # light_pulse_width
        i2c1.write_byte_data(i2c_address_lidar, 0x1D, 0x01)  # light_pulse_offset
        i2c1.write_byte_data(i2c_address_lidar, 0x1F, 0x3F)  # P4_delay
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
    time.sleep(0.25)

    while 1:
        pass

    """
    # GPIO frame read (through FIFO)
    data_stream = 0

    # LiDAR data processing
    depth_map = numpy.zeros((height, width), dtype=int, order='C')
    intensity_map = numpy.zeros((height, width), dtype=int, order='C')
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
            dif_F3 = data_F3_Ch1[y, x + 1] - data_F3_Ch2[y, x + 1]
            dif_F4 = data_F4_Ch1[y, x + 1] - data_F4_Ch2[y, x + 1]
            dif_1 = (dif_F1 - dif_F3) // 2
            dif_2 = (dif_F2 - dif_F4) // 2
            sum_F1 = data_F1_Ch1[y, x + 1] + data_F1_Ch2[y, x + 1]
            sum_F2 = data_F2_Ch1[y, x + 1] + data_F2_Ch2[y, x + 1]
            sum_F3 = data_F3_Ch1[y, x + 1] + data_F3_Ch2[y, x + 1]
            sum_F4 = data_F4_Ch1[y, x + 1] + data_F4_Ch2[y, x + 1]
            sum_1 = sum_F1 + sum_F3
            sum_2 = sum_F2 + sum_F4
            if dif_1 >= 0:
                depth_map[y, x] = (dif_2 / (abs(dif_1) + abs(dif_2)) + 1) / 4 * scipy.constants.c * 4.25e-6
            else:
                depth_map[y, x] = (-dif_2 / (abs(dif_1) + abs(dif_2)) + 3) / 4 * scipy.constants.c * 4.25e-6
            intensity_map[y, x] = (sum_1 + sum_2) / 8
            # WIP, lacks calibration process now
    print(depth_map)
    print(intensity_map)
    """

finally:
    # GC
    GPIO.cleanup()
    spi.close()
    print("\n - GPIO clean up.")
    sys.exit(0)
