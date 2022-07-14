# Menteebot fork
### MD80 - USB-CAN driver
This driver will make it simple to implement MD80 into existing projects. The driver will let the user change MD80's 
configuration as well as control the MD80 with most/all of its functionality.

## Md80 x CANdle quick start guide
The following video demonstrates the basics of working with MD80 drives using CANdle and the library from this repo.
## Quick start guide
For a quick guide on how to get started with MD80 drives and CANdle, check out the video below:
[![MD80xCANdle Quick start guide](https://img.youtube.com/vi/bIZuhFpFtus/0.jpg)](https://www.youtube.com/watch?v=bIZuhFpFtus)

## Dependencies
The library does not require any additional software to be functional, It can work as-is. 
However, to make full use of it, two additional packages can be used - setserial (for increasing maximal access frequency
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
If this is not possible, device access level can be granted by:
```
sudo chmod 777 /dev/ttyACMx     # where x is CANdle port number, usually 0
```
If this is also not possible, programs that use CANdle (including examples), can be launched with ```sudo```


## Latency and bandwidth
The communication frequency is currently fixed at 100Hz. This is true for both transmitting (PC->CANdle) and receiving 
(CANdle->PC). The CANdle will handle low-level FDCAN communications with the MD80s, at a rate sufficient to update 
all states (and send all commands), faster than the update rate with PC, regardless of the selected FDCAN baudrate.

## Documentation
Documentation is based on Doxygen. To build it use:
```
doxygen Doxyfile
```
A new Documentation directory will be created. Easily accesible HTML version will then be available at:
```
Documentation/html/index.html
```
## Building
Make sure you are in `md80_usbcan` main directory. 
```
mkdir build
cd build 
cmake ..
make
```
This will trigger the build of the library and examples. The library will be placed in `libs/` directory, 
include files in `include/` directory, and examples in examples/directory

By default, the library is built as SHARED library, and generates a project template in the selected build directory
The project template features a custom CMakeLists.txt file that links all necessary libs and 
header files to main.cpp. Feel free to copy the project and start your development from there.

The library can be built as static library with CMake variable CANDLE_BUILD_STATIC:
```
cmake .. -DCANDLE_BUILD_STATIC=TRUE
```

To build python library version, use CANDLE_BUILD_PYTHON CMake variable:
```
cmake .. -DCANDLE_BUILD_PYTHON=TRUE
```

## Examples
When building a set of examples will be built. The example executables are placed in 
```
build/Candle/examples/
```
while the source files are in
```
Candle/examples/
```
Examples feature simple use cases for most of the methods supplied with the library as well as comments guiding the user 
around the code and implementation.

# Example1
This is a most basic project showcasing a Candle::ping() method. It will ping the FDCAN bus for active drives and print 
all FDCAN IDs (drive IDs) that have responded. It will also blink LEDs on all drives that were detected.

# Example 2
This program will ping the FDCAN bus. If any md80 is found it will print out it's rotor position for a few seconds.

# Example 3
This is a demonstration of how to properly configure a single md80. It will ping the drives and set the new current limit 
to the first drive found. It will also change the FDCAN Id of this drive to ```newFDCanId```, which is randomly generated.
There is also a commented settings save method. If the ```configMd80Save``` stays commented out, the current and FDCAN parameters will be volatile, 
meaning they will be there until the drive is powered down. Using ```configMd80Save``` will make the change permanent 
(or until you change it next time).

# Example 4
This example will move the motor in Impedance Control mode, using default controller parameters. The drive will slowly  
perform a sinusoidal movement from the position of -3 [rad] to +[3] rad from the position at the start.

# Example 5
Example 5 is similar to ```Example 4``` with the difference being that the controller parameters are custom this time. Enter the desired control parameters as 'kp' and 'kd' arguments of the command:
```
./example5 kp kd

# for example
./example5 0.1 0.01
```
Be careful when selecting the kp/kd - with high gains the servos can get quite violent and perform rapid movements!

# Example 6 
This example shows how to control more than just one drive - it will ping the FDCAN bus and move all the drives it finds
(up to 12), in a synchronized manner. 

# Example 7
Demonstrates Velocity PID mode. The drive will follow a step velocity command - starting at 20 rad/s and going slightly 
faster every ~2 seconds.

# Example 8
Shows how Position PID mode works. An example will move the drive in a similar trajectory as Example 4 and 5, but using
a Position PID controller. You can see how it is different from Impedance mode and how much more parameters can be
customized.

# Example 9
Showcases usage of multiple candles on the same computer.
