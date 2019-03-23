#!/usr/bin/env python3
import subprocess
from matplotlib import pyplot as plt

from shared_helpers import *

import signal
import sys

args = get_args();

def killall():
  print("Killing all")
  subprocess.call("killall -9 cbs; killall -9 xstar; killall -9 driver", shell=True)
  

def generate_new_scenario(agents, width, height, obs_density, seed):
  ret_val = subprocess.call("../benchmark_generator.py {} {} {} {} ../simple_test.yaml afs_map_file.map afs_agents_file.agents --seed {}".format(agents, width, height, obs_density, seed), shell=True)
  if ret_val != 0:
    print("Genrate failed")
    exit(-1)

def run_xstar(timeout):
  try:
    return_code = subprocess.call("./xstar -i ../simple_test.yaml -o simple_test.result > xstar_tmp.out", shell=True, timeout=timeout)
    if return_code != 0:
      return ([2], [timeout])
  except:
    print("X* timeout")
    killall()
    subprocess.call("sed -i '$ d' xstar_tmp.out", shell=True)
  f = open("xstar_tmp.out")
  content = [x.strip() for x in f.readlines()]
  bounds = [float(e.replace("Optimality bound:", "")) for e in content if "Optimality" in e]
  times = [float(e.replace("Time so far:", "")) for e in content if "Time so far" in e]
  if (len(times) <= 0):
    times.append(timeout)
  if (len(bounds) <= 0):
    bounds.append(2)
  return (bounds, times)

def run_afs(timeout):
  try:
    return_code = subprocess.call("../afs/AnytimeMAPF/driver --map afs_map_file.map --agents afs_agents_file.agents --export_results afs_results.out --time_limit {} > /dev/null".format(timeout), shell=True, timeout=(timeout + 2))
    if return_code != 0:
      return ([2], [timeout])
  except:
    print("AFS timeout")
    killall()
    return ([2], [timeout])
  f = open("afs_results.out")
  lines = f.readlines()[1:]
  csv = [e.split(',') for e in lines]
  times = [float(e[1]) for e in csv]
  bounds = [float(e[3]) for e in csv]
  return (bounds, times)
  
def run_cbs(timeout):
  try:
    if subprocess.call("./cbs -i ../simple_test.yaml -o simple_test.result > cbs_tmp.out", shell=True, timeout=timeout) != 0:
      return timeout
  except:
    print("CBS timeout")
    killall()
    return timeout
  f = open("simple_test.result")
  runtime = [float(x.strip().replace("runtime:", "")) for x in f.readlines() if "runtime:" in x][0]
  return runtime

xstar_data_lst = []
afs_data_lst = []
cbs_data_lst = []

for i in range(args.num_trials):
  print("Trial {}:==============================================".format(i))
  seed = (i + args.width) * args.agents
  generate_new_scenario(args.agents, args.width, args.height, args.obs_density, seed)
  
  print("X*")  
  xstar_runtimes = []
  xstar_bounds = []
  for j in range(args.iter_per_trial):
    bounds, times  = run_xstar(args.timeout)
    assert(type(times) == list)
    assert(type(bounds) == list)
    assert(len(times) > 0)
    assert(len(bounds) > 0)
    xstar_bounds = bounds
    xstar_runtimes.append(Runtime(times))
  xstar_data_lst.append(XStarData(args.obs_density, args.width, args.height, args.agents, args.timeout, xstar_bounds, xstar_runtimes))
  
  print("AFS")  
  afs_runtimes = []
  afs_bounds = []
  for j in range(args.iter_per_trial):
    bounds, times  = run_afs(args.timeout)
    assert(type(times) == list)
    assert(type(bounds) == list)
    assert(len(times) > 0)
    assert(len(bounds) > 0)
    afs_bounds = bounds
    afs_runtimes.append(Runtime(times))
  afs_data_lst.append(AFSData(args.obs_density, args.width, args.height, args.agents, args.timeout, afs_bounds, afs_runtimes))
  
  print("CBS")
  cbs_runtimes = []
  for j in range(args.iter_per_trial):
    runtime = run_cbs(args.timeout)
    cbs_runtimes.append(Runtime([runtime]))
  cbs_data_lst.append(CBSData(args.obs_density, args.width, args.height, args.agents, args.timeout, cbs_runtimes))
  
save_to_file("xstar_data_lst_{}".format(args_to_str(args)), xstar_data_lst)
save_to_file("cbs_data_lst_{}".format(args_to_str(args)), cbs_data_lst)
save_to_file("afs_data_lst_{}".format(args_to_str(args)), afs_data_lst)

# Ensures data can be reloaded properly
xstar_data_lst = read_from_file("xstar_data_lst_{}".format(args_to_str(args)))
cbs_data_lst = read_from_file("cbs_data_lst_{}".format(args_to_str(args)))
afs_data_lst = read_from_file("afs_data_lst_{}".format(args_to_str(args)))

    
    
    
