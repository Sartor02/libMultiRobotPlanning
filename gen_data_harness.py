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
    parser.add_argument("loru", type=str, help="Lower or upper [l | u]")
    return parser.parse_args()

def is_lower(gen_args):
    assert(gen_args.loru == 'l' or gen_args.loru == 'u')
    return (gen_args.loru == 'l')

gen_args = get_gen_data_args()

kTimeout = 300
kNumTrials = 200
kNumIterations = 3

wh_data = [100]
l_agents = [10, 20, 30, 40]
u_agents = [50, 60, 80, 100]
agents_data = l_agents if is_lower(gen_args) else u_agents
density_data = [0.01, 0.05, 0.1]

total_iterations = len(wh_data) * len(agents_data) * len(density_data)


f = open("gen_data_harness_{}.status".format(gen_args.loru), 'w')
f.write("Beginning! Time: {}\n".format(str(datetime.datetime.now())))
f.close()

iter = 0
for wh in wh_data:
    for agents in agents_data:
        for density in density_data:
            f = open("gen_data_harness.result", 'a')
            f.write("Percent: " + str((iter / total_iterations) * 100) + " Agents: " + str(agents) + " Width/Height: " + str(wh) + " Density: " + str(density) + '\n')
            f.close()
            ret_val = subprocess.call("./collect_data.py {} {} {} {} {} {} {}".format(agents, wh, wh, density, kNumIterations, kNumTrials, kTimeout), shell=True)
            f = open("gen_data_harness.result", 'a')
            f.write("Complete!\n")
            f.close()

    
