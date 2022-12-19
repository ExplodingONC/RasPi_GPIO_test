# RasPi_IO_test
A test of RasPi 4B's I/O capability, nothing special... Bunch of random things tested and piled up here.

The originall goal is reading a 12bit parallel signal. Direct access, FIFO buffer and home-made serializer tested.

Also tested OpenCV video output capability (with multi-monitor and multi-threading).

The corresponding circuit is https://github.com/ExplodingONC/S15454-01WT_on_RasPi (outdated)

Python and C shared object are used. Direct register access from C program is the fastest method, but also the least stable one. Currently settled on home-made serializer plan achieved with PIO on RP2040.

Some of the if-else conditions have place holders since this is just a test.
