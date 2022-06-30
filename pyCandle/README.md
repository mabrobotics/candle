
# pyCandle module

Python module based on C++ Candle libraries. Pybind11 is used to create a *.so module that can be imported to python. In order to add new classes/methods/variales to the python module they have to be described in the main.cpp file and then ```make``` has to be called from the ./build directory. This will result in a generation of *.so module that can be imported to python source files using
``` import build.pyCandle as pyCandle ``` for development 
or 
```import mab.pyCandle as pyCandle``` for release

