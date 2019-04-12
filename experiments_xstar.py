#!/usr/bin/env python3
import subprocess
from matplotlib import pyplot as plt
import shlex
import hashlib
import signal
import sys
import os
import argparse
import glob
from shutil import copyfile

current_proc = None
def signal_handler(sig, frame):
  print(sys.argv[0], 'Signal caught')
  if current_proc is not None:
    current_proc.terminate()
  sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def get_args():
  parser = argparse.ArgumentParser()
  parser.add_argument("iterations", type=int, help="X* iterations")
  parser.add_argument("trials", type=int, help="X* trials")
  return parser.parse_args()

args = get_args();

kMapBaseName = "map"
kNumAgents = [10, 20, 30, 40, 50]
kWidth = 100
kHeight = 100
kDensity = 0.05
kMemoryLimitGB = 60
kTimeout = 1200 # 20 minutes
#kNumAgentsWHTimeout = [(30, 122, 122, 450), (60, 173, 173, 900), (80, 200, 200, 1200)]

def std_map_name(map_base_name):
  return map_base_name + ".stdmap"

def afs_map_name(map_base_name):
  return map_base_name + ".afsmap"

def afs_agents_name(map_base_name):
  return map_base_name + ".afsagents"

def generate_new_scenario(agents, width, height, obs_density, seed, map_base_name):
  cmd = "./benchmark_generator.py {} {} {} {} {} {} {} --seed {}"\
    .format(agents, 
            width, 
            height, 
            obs_density, 
            std_map_name(map_base_name), 
            afs_map_name(map_base_name), 
            afs_agents_name(map_base_name), 
            seed)
  if subprocess.call(cmd, shell=True) != 0:
    print("Generate failed")
    exit(-1)
    
def select_seed(idx, num_agents, density, width, height):
  return int((idx + 1) * 3 + num_agents * 7 + 1000 * density + width + height * 2)

def run():
  global current_proc
  #for num_agents, kWidth, kHeight, kTimeout in kNumAgentsWHTimeout:
  for num_agents in kNumAgents:
    for idx in range(args.iterations):
      seed = select_seed(idx, num_agents, kDensity, kWidth, kHeight)
      generate_new_scenario(num_agents, kWidth, kHeight, kDensity, seed, kMapBaseName)
      print("Agents:", num_agents, "Iter:", idx, "Seed:", seed)
      cmd = "./collect_data_xstar.py {} {} {} {} datasave/xstar_ratio_agents_1200_timeout_{}_iter_{}_trial_ _seed_{}.result".format(std_map_name(kMapBaseName), kTimeout, args.trials, kMemoryLimitGB, num_agents, idx, seed)
      current_proc = subprocess.Popen(shlex.split(cmd))
      current_proc.wait()
      
run()

