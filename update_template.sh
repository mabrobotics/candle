#!/bin/bash

set -e

CURRENT=`pwd`
BASENAME=`basename "$CURRENT"`

if test ${BASENAME} != "build" 
then
	echo "This script is designed to run from md80_usbcan/build/ directory. Quitting."
	exit -1
else
	echo "Copying..."
fi
cp ./Candle/libcandle.so ../Template/lib/libcandle.so
cp ./Md80/libmd80.so ../Template/lib/libmd80.so
cp ../Candle/candle.hpp ../Template/include/candle.hpp
cp ../Md80/md80.hpp ../Template/include/md80.hpp

echo "Copying complete"
