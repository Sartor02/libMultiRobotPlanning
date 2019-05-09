#!/bin/bash
mkdir build
mkdir release
cd build
cmake ..
make -j`nproc`
cd ..
cd release
cmake -DCMAKE_BUILD_TYPE=Release ..
cd ..
cd afs/AnytimeMAPF
make executable
cd ../../
git submodule update --recursive --init
cd mstar_public/cpp
make -j`nproc`
cd ../../
