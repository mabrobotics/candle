### MD80 - USB-CAN driver
This driver will make it simple to implement MD80 into existing projects. The driver will let the user change MD80's 
configuration as well as control the MD80 with most/all of its functionality.

## Dependencies
The library does not require any additional software to be functional, It can work as-is. 
However to make full use of it, two additional packages can be used - setserial (for increasing maximal access frequency
to the serial port used for communication with CANdle) and doxygen (for building the documentation). To get them:
```
sudo apt install setserial
sudo apt install doxygen
```

## USB Access
To enable access to CANdle from userspace, the user should be added to dialout group by calling:
```
sudo usermod -a -G dialout <user>     # where <user> is current username
```
If this is not possible, devices access level can be granted by:
```
sudo chmod /dev/ttyACMx     # where x is CANdle port number, usually 0
```
If this is also not possible, programs that use CANdle (including examples), can be launched with ```sudo```


## Latency and bandwith
Communication frequency is currently fixed at 100Hz. This is true for both transmitting (PC->CANdle) and receiving 
(CANdle->PC). The CANdle will haandle low level FDCAN communications with the MD80s, as rate sufficient to update 
all states (and send all commands), faster that the update rate with PC, regardless of selected FDCAN baudrate.

## Documentation
Documentation is based on Doxygen. To build it use:
```
doxygen Doxyfile
```
A new Documentation directory will be created. Easly accesible HTML version will then be available at:
```
Documentation/html/index.html
```
## Building
Make sure you are in md80_driver main directory. 
```
mkdir build
cd build 
cmake ..
make
```
This will trigger the build of the library and examples. The library will be placed in libs/ directory, 
include files in include/ directory, and examples in examples/directory

By default the library is build as STATIC, but can also be built as SHARED. If SHARED library is desired, 
the user shall uncomment line
```
set(LIB_TYPE SHARED)
```
in ```Candle/CMakeLists.txt```. Thiss will not only change the build type to SHAREd, but also generate a project
template at: ```build/Project/```. The project features a custom CMakeLists.txt file that links all necessary libs and 
header files to main.cpp. 
Feel free to copy the project and start your develepoment from there.
## Examples
When building a set of examples will be built. The example executables are placed in 
```
build/Candle/examples/
```
while the source files are in
```
Candle/examples/
```
Examples feature a simple use cases for most of the methods supplied with the library as well as comments guiding user 
around the code and implementation.