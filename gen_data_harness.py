#!/usr/bin/env python3
import subprocess
from colorama import Fore, Style
from matplotlib import pyplot as plt
import numpy as np
import datetime

from shared_helpers import *

import signal
import sys
import argparse

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("split", type=str, help="Split of agents [lower | upper]")
    return parser.parse_args()

args = parse_args()

if (args.split != "lower" and args.split != upper):
    print("Invalid split")
    exit(-1)

is_lower = (args.split == "lower")

kTimeout = 240
kNumTrials = 300
kNumIterations = 3

wh_data = [100]
lower_agents_data = [10, 20, 30, 50, 70, 90]
upper_agents_data = [100, 120, 140, 160, 180, 200]
agents_data = lower_agents_data if is_lower else upper_agents_data
density_data = [0.01, 0.05, 0.1]

total_iterations = len(wh_data) * len(agents_data) * len(density_data)


f = open("gen_data_harness.result", 'w')
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

    
