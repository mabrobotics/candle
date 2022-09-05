#!/bin/bash
usage="$(basename "$0") [-h] [-l launch-file] [-b true|false] [-m true|false] [-s true|false] [-g log_dir] [-o output_folder]
-- this script runs menteebot light code

where:
    -p create candlePy.so to create pybind and also install with pip
    -h print usage
    "

compile_py=0

while getopts "ph:" flag
do
    case "${flag}" in
        p) compile_py=1;;
        h) echo "$usage"
          exit;;
        *) echo "usage: $0 [-r]" >&2
           exit;;
    esac
done

rm -rf build
mkdir build
cd build

if [ $compile_py -eq 1 ]; then
    echo "#############################Build pybind#################################33"
    cmake .. -DCANDLE_BUILD_PYTHON=TRUE    
else
    echo "#############################Build C++ library#################################33"
    cmake ..
fi

make


if [ $compile_py -eq 1 ]; then
    echo "Installing pybind via pip"
    cp pyCandle/pyCandle* ../candle_pip/src/mab/
    cd ../candle_pip/
    pip install .
else
    cp Candle/libcandle.so ../../ros2_workspace/candle_ros2/lib/
    cp -r ../Candle/include/* ../../ros2_workspace/candle_ros2/include/Candle/
fi

echo "I am DONE"
cd ..
