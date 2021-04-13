### MD80 - USB-CAN driver
This basic driver will make it simple to implement MD80 into existing projects. The driver will let the user change MD80's configuration as well as control the MD80 with most/all of its functionality.

## Latency
With just basic configuration, the USB-CAN driver will only perform at ~62 Hz. 
To lower the latency, following actions have to be performed.
```
sudo apt install setserial
setserial /dev/ttyUSB0 low_latency
```
This will boost communication frequency to about 400Hz. The final frequency is dependent on host system configuration and load.

## Building
Drivers is organized as two shared libraries. To use it right away, copy Template directory anywhere and use it
as a template for new project.
+ Note: 'Template' should be later split into separate repository for public use.
To build the template 
```
cd Template
mkdir build
cd build
cmake ..
make
```

## Running
```
cd build
./template
```

## Template modification
You can change directory name, as well as project name in CMakeLists.txt. The template will use *./lib* folder (for shared libraries), *./include* (for header files), and *./src* (for source files). Feel free to add your files to these folders. 

The project will automatially attach all source and header files (.cpp, .c, .h, .hpp). Any additionall libraries must be added manually to CMakeLists.txt. 
