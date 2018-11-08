# libMultiRobotPlanning

libMultiRobotPlanning is a library with search algorithms primarily for task and path planning for multi robot/agent systems.
It is written in C++(14), highly templated for good performance, and comes with useful examples.

The following algorithms are currently supported:

* Single-Robot Algorithms
  * A*
  * A* epsilon (also known as focal search)
  * SIPP (Safe Interval Path Planning)

* Multi-Robot Algorithms
  * Conflict-Based Search (CBS)
  * Enhanced Conflict-Based Search (ECBS)
  * Conflict-Based Search with Optimal Task Assignment (CBS-TA)
  * Enhanced Conflict-Based Search with Optimal Task Assignment (ECBS-TA)
  * Prioritized Planning using SIPP (example code for SIPP)

* Assignment Algorithms
  * Minimum sum-of-cost (flow-based; integer costs; any number of agents/tasks)
  * Best Next Assignment (series of optimal solutions)

## Building

Tested on Ubuntu 18.04.

```
make
```

### Targets

* `make`: Build all code
* `make clang-format`: Re-format all source files
* `make test`: Run unit-tests

## Run example instances

### ECBS

````
./build/ecbs -i benchmark/32x32_obst204/map_32by32_obst204_agents10_ex1.yaml -o output.yaml -w 1.3
python3 example/visualize.py benchmark/32x32_obst204/map_32by32_obst204_agents10_ex1.yaml output.yaml
````
