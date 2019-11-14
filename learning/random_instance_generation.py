#!/usr/bin/env python3

import subprocess
import signal
import shlex
import sys
import os
import glob
import argparse

import run_cbs
import run_xstar
import run_mstar
import run_afs

def generate_new_scenario(agents, width, height, obs_density, seed):
  generic_map = "learning/random/data/seed{}_generic.map".format(seed)
  afs_map = "learning/random/data/seed{}_afs.map".format(seed)
  afs_agents = "learning/random/data/seed{}_afs.agents".format(seed)
  ret_val = subprocess.call("./benchmark_generator.py {} {} {} {} {} {} {} --seed {}".format(agents, width, height, obs_density, generic_map, afs_map, afs_agents, seed), shell=True)
  if ret_val != 0:
    print("Genrate failed")
    exit(-1)
  return (generic_map, afs_map, afs_agents)

kWidth = 32
kHeight = 32
kAgents = 20
kObsDensity = 0.1
kNumSeeds = 1000
kTimeout = 10

def get_optimal_runtime(bounds, runtimes, timeout):
  if bounds[-1] != 1:
    return timeout
  return runtimes[-1]

def get_first_runtime(bounds, runtimes, timeout):
  return runtimes[0]

def write_labels(optimal_times, first_times, seed):
  d = dict()
  d["optimal_times"] = optimal_times
  d["first_times"] = first_times
  f = open("learning/random/data/seed{}_label.labels".format(seed), 'w')
  f.write("{}\n".format(str(d)))
  f.close()

for seed in range(kNumSeeds):
  print("==================")
  generic_map, afs_map, afs_agents = generate_new_scenario(kAgents, kWidth, kHeight, kObsDensity, seed)
  cbs_time = run_cbs.run(generic_map, kTimeout)
  print("CBS Time: %.6f" % cbs_time)
  mstar_time = run_mstar.run(generic_map, kTimeout)
  print("M* Time:  %.6f" % mstar_time)
  xstar_bounds, xstar_times = run_xstar.run(generic_map, kTimeout)
  xstar_time = get_optimal_runtime(xstar_bounds, xstar_times, kTimeout)
  xstar_first_time = get_first_runtime(xstar_bounds, xstar_times, kTimeout)
  print("X* Time:  %.6f" % xstar_time)
  afs_bounds, afs_times = run_afs.run(afs_map, afs_agents, kTimeout)
  afs_time = get_optimal_runtime(afs_bounds, afs_times, kTimeout)
  afs_first_time = get_first_runtime(afs_bounds, afs_times, kTimeout)
  print("AFS Time: %.6f" % afs_time)
  optimal_times = [(cbs_time, "CBS"), (mstar_time, "M*"), (xstar_time, "X*"), (afs_time, "AFS")]
  optimal_times.sort(key = lambda k: k[0])
  first_times = [(cbs_time, "CBS"), (mstar_time, "M*"), (xstar_first_time, "X*"), (afs_first_time, "AFS")]
  first_times.sort(key = lambda k: k[0])
  print("Optimal Winner:", optimal_times[0][1])
  print("First Winner:  ", first_times[0][1])
  write_labels(optimal_times, first_times, seed)
