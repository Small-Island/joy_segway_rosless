#!/bin/bash

cd libsegwayrmp
cd serial
make
cd ..

make
cd ../segway_rmp
if [ ! -e build ]; then
    mkdir build
fi
cd build && cmake .. && make
cd ../../
