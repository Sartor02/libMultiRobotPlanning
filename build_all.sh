#!/bin/bash
mkdir build;
mkdir release;
cd build;
cmake ..;
make clean
make -j`nproc`;
cd ..;
cd release;
cmake -DCMAKE_BUILD_TYPE=Release ..;
make clean;
make -j`nproc`;
cd ..;
cd afs/AnytimeMAPF;
make executable;
cd ../../;
git submodule update --recursive --init;
cd mstar_public/cpp;
make clean;
make -j`nproc`;
cd ../../;
cd Push-and-Rotate--CBS--PrioritizedPlanning;
mkdir build;
cd build;
cmake -DCMAKE_BUILD_TYPE=Release ..;
make clean;
make -j`nproc`
cd ../..;
