#!/usr/bin/env python3
import subprocess
from colorama import Fore, Style
from matplotlib import pyplot as plt
import numpy as np

from shared_helpers import *

import signal
import sys

kTimeout = 30
kNumTrials = 500
kNumIterations = 3

wh_data = [100]
agents_data = range(10, 201, 10)
density_data = [0.01, 0.1, 0.2, 0.3]

total_iterations = len(wh_data) * len(agents_data) * len(density_data)

subprocess.call("./collect_data.py {} {} {} {} {} {} {}".format(10, 100, 100, 0.01, 3, 5, 10), shell=True)
exit(0)

f = open("gen_data_harness.result", 'w')
f.write("Beginning, writing current to 'current_run.output'\n")
f.close()

iter = 0
for wh in wh_data:
    for agents in agents_data:
        for density in density_data:
            f = open("gen_data_harness.result", 'a')
            f.write("Percent: " + str((iter / total_iterations) * 100) + " Agents: " + str(agents) + " Width/Height: " + str(wh) + " Density: " + str(density) + '\n')
            f.close()
            subprocess.call("./collect_data.py {} {} {} {} {} {} {}".format(agents, wh, wh, density, kNumIterations, kNumTrials, kTimeout), shell=True)
            f = open("gen_data_harness.result", 'a')
            f.write("Complete!\n")
            f.close()

    
