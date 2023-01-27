# spitest.py
# A brief demonstration of the Raspberry Pi SPI interface, using the Sparkfun
# Pi Wedge breakout board and a SparkFun Serial 7 Segment display:
# https://www.sparkfun.com/products/11629

import sys
import time
import spidev

# setup SPI bus
try:
    spi_channel = 0
    spi_device_MCU = 0
    spi = spidev.SpiDev()
    spi.open(spi_channel, spi_device_MCU)
except Exception as err:
    print("Error:", err)
    print("SPI initialization failed!")
    sys.exit()
else:
    spi.max_speed_hz = 1000000
    spi.mode = 0b11
    spi.bits_per_word = 8
    spi.lsbfirst = False
    print(f"SPI initialized at {spi.max_speed_hz}Hz.")

try:
    time.sleep(0.5)
    while 1:
        ret = spi.xfer([0x01])
        print(ret)
        time.sleep(0.1)

except Exception as err:
    print("Error:", err)

finally:
    # GC
    spi.close()
    print("\n - Clean up.")
    sys.exit(0)
