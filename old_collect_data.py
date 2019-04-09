#!/usr/bin/env python3
import subprocess
from matplotlib import pyplot as plt

from shared_helpers import *

import signal
import sys
import os
cwd = os.getcwd()
dir_path = os.path.dirname(os.path.realpath(__file__))

print(cwd)
print(dir_path)

args = get_args();

def args_to_string(args):
  return str(args).replace('(', '').replace(')', '').replace(' ', '').replace('=', '')

generic_map = "simple_test{}.yaml".format(args_to_string(args))
afs_map = "afs_map_file{}.map".format(args_to_string(args))
afs_agents = "afs_agents_file{}.agents".format(args_to_string(args))  

def generate_new_scenario(agents, width, height, obs_density, seed):
  ret_val = subprocess.call("./benchmark_generator.py {} {} {} {} {} {} {} --seed {}".format(agents, width, height, obs_density, generic_map, afs_map, afs_agents, seed), shell=True)
  if ret_val != 0:
    print("Genrate failed")
    exit(-1)

def run_mstar(timeout):
  if subprocess.call("timeout {} mstar_public/cpp/main -i {} -o simple_test{}.result".format(timeout, generic_map, args_to_string(args)), shell=True) != 0:
    print("M* timeout")
    return timeout
  f = open("simple_test{}.result".format(args_to_string(args)))
  runtime = [float(x.strip().replace("runtime:", "")) for x in f.readlines() if "runtime:" in x][0]
  return runtime

def run_xstar(timeout):
  return_code = subprocess.call("timeout {} release/xstar -i {} -o simple_test.result > xstar_tmp{}.out".format(timeout, generic_map, args_to_string(args)), shell=True)
  if return_code != 0:
    print("X* timeout")
    # Strip potential rubbish.
    subprocess.call("sed -i '$ d' xstar_tmp{}.out".format(args_to_string(args)), shell=True)
  f = open("xstar_tmp{}.out".format(args_to_string(args)))
  content = [x.strip() for x in f.readlines()]
  bounds = [float(e.replace("Optimality bound:", "")) for e in content if "Optimality" in e]
  times = [float(e.replace("Time so far:", "")) for e in content if "Time so far" in e]
  if (return_code != 0 or len(times) <= 0):
    times.append(timeout)
  if (return_code != 0 or len(bounds) <= 0):
    bounds.append(2)
  return (bounds, times)

def run_afs(timeout):
  return_code = subprocess.call("timeout {} afs/AnytimeMAPF/driver --map {} --agents {} --export_results afs_results{}.out --time_limit {} > /dev/null".format(timeout + 2, afs_map, afs_agents, args_to_string(args), timeout), shell=True)
  if return_code != 0:
    print("AFS timeout")
    # AFS dumps the file at the end; if it timed out, that mean AFS died and we have no data.
    return ([2], [timeout])
  f = open("afs_results{}.out".format(args_to_string(args)))
  lines = f.readlines()[1:]
  csv = [e.split(',') for e in lines]
  times = [float(e[1]) for e in csv]
  bounds = [float(e[3]) for e in csv]
  # If optimal solution was not found, add max time.
  if (len(bounds) == 0 or bounds[-1] != 1.0):
    times.append(timeout)
    bounds.append(1.0)
  return (bounds, times)
  
def run_cbs(timeout):
  if subprocess.call("timeout {} release/cbs -i {} -o simple_test{}.result".format(timeout, generic_map, args_to_string(args)), shell=True) != 0:
    print("CBS timeout")
    return timeout
  f = open("simple_test{}.result".format(args_to_string(args)))
  runtime = [float(x.strip().replace("runtime:", "")) for x in f.readlines() if "runtime:" in x][0]
  return runtime

xstar_data_lst = []
mstar_data_lst = []
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

  print("M*")
  mstar_runtimes = []
  for j in range(args.iter_per_trial):
    runtime = run_mstar(args.timeout)
    mstar_runtimes.append(Runtime([runtime]))
  mstar_data_lst.append(MStarData(args.obs_density, args.width, args.height, args.agents, args.timeout, mstar_runtimes))
  
save_to_file("xstar_data_lst_{}".format(args_to_str(args)), xstar_data_lst)
save_to_file("cbs_data_lst_{}".format(args_to_str(args)), cbs_data_lst)
save_to_file("afs_data_lst_{}".format(args_to_str(args)), afs_data_lst)
save_to_file("mstar_data_lst_{}".format(args_to_str(args)), mstar_data_lst)

# Ensures data can be reloaded properly
xstar_data_lst = read_from_file("xstar_data_lst_{}".format(args_to_str(args)))
cbs_data_lst = read_from_file("cbs_data_lst_{}".format(args_to_str(args)))
afs_data_lst = read_from_file("afs_data_lst_{}".format(args_to_str(args)))
mstar_data_lst = read_from_file("mstar_data_lst_{}".format(args_to_str(args)))

    
    
    
