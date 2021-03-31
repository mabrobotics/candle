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
```
mkdir build
cd build
cmake ..
make
```

## Running
```
./build/md80
