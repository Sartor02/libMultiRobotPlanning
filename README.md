# libMultiRobotPlanning

libMultiRobotPlanning is a library with search algorithms primarily for task and path planning for multi robot/agent systems.
It is written in C++(14), highly templated for good performance, and comes with useful examples.

## Building

Tested on Ubuntu 16.04 and Ubuntu 18.04. To install the necessary packages, run:

```
InstallPackagesUbuntu
```

To build in `Debug` mode (debug optimizations enabled):

```
mkdir build
cd build
cmake ..
make -j`nproc`
```

To build in `Release` mode (full optimizations enabled):

```
mkdir release
cd release
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j`nproc`
```

### Running X*


```
release/xstar -i <path to benchmark.yaml> -o <path to output file> -t <infix for timing file>
```

### Visualizing Resulting Path

```
example/visualize.py <path to benchmark.yaml> <path to output file>
```
