#!/bin/bash

cd libsegwayrmp
if [ ! -e lib ]; then
    mkdir lib
fi
if [ $(arch) = 'aarch64' ]; then
    cp ./ftd2xx/linux/arm64/libftd2xx.a ./lib/
elif [ $(arch) = 'x86_64' ]; then
    cp ./ftd2xx/linux/x64/libftd2xx.a ./lib/
fi

cd serial
make

cd ..

make
cd ../segway_rmp
if [ ! -e build ]; then
    mkdir build
fi
cd build && cmake .. && make
cmake .
cd ../../
