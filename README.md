# CANdle library 
CANdle C++ library is the heart of the MD80 x CANdle ecosystem on the master controller side. 
It takes care of all actions connected to communication and provides API for higher-level software. For a detailed manual please see [MD80 x CANdle Manual](https://www.mabrobotics.pl/servos/manual).
 
## C++ library
To run the examples clone the repo and make sure you're in the `candle` main directory. Execute:
```
mkdir build
cd build 
cmake ..
make
```
After a successful build you should be able to see the compiled examples in `candle/build/` directory. 
To run an example simply call `./exampleX` where `X` is the example number. 
 
## Python library
CANdle Python library was created using [pybind11](https://github.com/pybind/pybind11). 
You can install the package using the command:
`pip install pyCandleMAB`. Python examples are located in `examples_python` directory. 
 
## Examples
Examples feature simple use cases for most of the methods supplied with the library as well as comments guiding the user 
around the code and implementation. The C++ and Python examples are equivalent. 
 
### Example1
This is a most basic project showcasing a Candle::ping() method. It will ping the FDCAN bus for active drives and print 
all FDCAN IDs (drive IDs) that have responded. It will also blink LEDs on all drives that were detected.
 
### Example 2
This program will ping the FDCAN bus. If any MD80 is found it will print out its rotor position for a few seconds.
 
### Example 3
This is a demonstration of how to properly configure a single MD80. It will ping the drives and set the new current limit 
to the first drive found. It will also change the FDCAN Id of this drive to ```newFDCanId```, which is randomly generated.
There is also a commented save method. If the ```configMd80Save``` stays commented out, the current and FDCAN parameters will be volatile, 
meaning they will be there until the drive is powered down. Using ```configMd80Save``` will make the change permanent 
(or until you change it next time).
 
### Example 4
This example will move the motor in Impedance Control mode, using default controller parameters. The drive will slowly  
perform a sinusoidal movement from the position of -3 [rad] to +3 [rad] from the position at the start.
 
### Example 5
Example 5 is similar to `example 4` with the difference being that the controller parameters are custom this time. Enter the desired control parameters as 'kp' and 'kd' arguments of the command:
```
./example5 kp kd
 
### for example ###
./example5 0.1 0.01
```
Be careful when selecting the kp/kd - with high gains the servos can get quite violent and perform rapid movements!
 
### Example 6 
This example shows how to control more than just one drive - it will ping the FDCAN bus and move all the drives it finds
(up to 12), in a synchronized manner. 
 
### Example 7
Demonstrates Velocity PID mode. The drive will follow a step velocity command - starting at 20 rad/s and going slightly 
faster every ~2 seconds.
 
### Example 8
Shows how Position PID mode works. An example will move the drive in a similar trajectory as Example 4 and 5, but using
a Position PID controller. You can see how it is different from Impedance mode and how much more parameters can be
customized.
 
### Example 9
Showcases usage of multiple CANdles on the same computer using USB bus. 
 
### Example 10
Similar to `example 6`, but based on the SPI bus available on single-board computers. Before running be sure all actuators are switched to 8M speed. 
 
### Example 11
Similar to `example 6`, but based on the UART bus available on single-board computers. Before running be sure all actuators are switched to 8M speed. 
 
### Example 12
Register read-write demo. Please remember that register operations are only valid before candle.begin() is called. 

### Example 13
Profile velocity demo.

### Example 14
Profile position demo.

### Example 15
Raw torque demo.

 
