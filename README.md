# serial_connect

## Description
C++ uart library

## Installation

As soon as you clone this repository, type these commands bellow. 

```
mkdir build && cd build
cmake .. && make
sudo make install 
```

Then you can use serial_connect library by just including  serial_connect/serial_connect.hpp 

Make sure you write 

```
find_package(serial_connect REQUIRED) 
```

in your CMakeLists.txt file.

## How to use
1. Create SerialConnect class
2. Set device information
3. Open device
4. Read and Write the device
5. Close device
6. (Optional) Set receive interrupt callback function
7. (Optional) Set error and information display function
