#!/usr/bin/env python3
import subprocess
from colorama import Fore, Style
from matplotlib import pyplot as plt
import numpy as np
import datetime

from shared_helpers import *

import signal
import sys

def get_gen_data_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("agents_section", type=int, help="Section of total agents")
    return parser.parse_args()

gen_args = get_gen_data_args()

def get_agents(gen_args):
    assert(gen_args.agents_section < 4)
    if gen_args.agents_section == 0:
        return [10, 20]
    elif gen_args.agents_section == 1:
        return [30, 40]
    elif gen_args.agents_section == 2:
        return [50, 60]
    elif gen_args.agents_section == 3:
        return [80, 100]



kTimeout = 300
kNumTrials = 200
kNumIterations = 3

wh_data = [100]
agents_data = get_agents(gen_args)
density_data = [0.01, 0.05, 0.1]

total_iterations = len(wh_data) * len(agents_data) * len(density_data)


status_file = "gen_data_harness_{}.status".format(gen_args.agents_section)

f = open(status_file, 'w')
f.write("Beginning! Time: {}\n".format(str(datetime.datetime.now())))
f.close()

iter = 0
for wh in wh_data:
    for agents in agents_data:
        for density in density_data:
            f = open(status_file, 'a')
            f.write("Percent: " + str((iter / total_iterations) * 100) + " Agents: " + str(agents) + " Width/Height: " + str(wh) + " Density: " + str(density) + '\n')
            f.close()
            ret_val = subprocess.call("./collect_data.py {} {} {} {} {} {} {}".format(agents, wh, wh, density, kNumIterations, kNumTrials, kTimeout), shell=True)
            f = open(status_file, 'a')
            f.write("Complete!\n")
            f.close()

    
