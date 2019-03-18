#!/usr/bin/env python3
import subprocess
from colorama import Fore, Style
from matplotlib import pyplot as plt

def generate_new_scenario(agents, width, height):
  return subprocess.call("../benchmark_generator.py {} {} {} 0.01 ../simple_test.yaml afs_map_file afs_agents_file".format(agents, width, height), shell=True)

def run_xstar(timeout):
  try:
    return_code = subprocess.call("./xstar -i ../simple_test.yaml -o simple_test.result | tee tmp.out", timeout=timeout, shell=True)
    if return_code != 0:
      return None
  except:
      print("timeout")
  f = open("tmp.out")
  content = [x.strip() for x in f.readlines()]
  bounds = [float(e.replace("Optimality bound:", "")) for e in content if "Optimality" in e]
  times = [float(e.replace("Time so far:", "")) for e in content if "Time so far" in e]
  print(bounds)
  print(times)
  return (bounds, times)
  
def run_cbs(timeout):
  try:
    if subprocess.call("./cbs -i ../simple_test.yaml -o simple_test.result", shell=True, timeout=timeout) != 0:
      return None
  except:
    return timeout
  f = open("simple_test.result")
  runtime = [float(x.strip().replace("runtime:", "")) for x in f.readlines() if "runtime:" in x][0]
  return runtime

xstar_bounds_lst = []
xstar_times_lst = []
cbs_times_lst = []

for i in range(100):
  print(Fore.BLUE + "Iteration {}:==============================================".format(i) + Style.RESET_ALL)
  if generate_new_scenario(10, 100, 100) != 0:
    print("Genrate failed")
    exit(-1)
    
  xstar_result = run_xstar(10)
  if xstar_result is None:
    print("X* failed")
    exit(-1)
  xstar_bounds, xstar_times = xstar_result
  xstar_bounds_lst.append(xstar_bounds)
  xstar_times_lst.append(xstar_times)
    
  cbs_runtime = run_cbs(10)
  if cbs_runtime is None:
    print("cbs failed")
    exit(-1)
  cbs_times_lst.append(cbs_runtime)
  
  
assert(len(xstar_times_lst) == len(cbs_times_lst))

optimal_ratios = [x[-1]/c for x, c in zip(xstar_times_lst, cbs_times_lst)]
first_ratios = [x[0]/c for x, c in zip(xstar_times_lst, cbs_times_lst)]
n_bins = 300
plt.title("Optimal ratios")
plt.hist(optimal_ratios, bins=n_bins)
plt.show()

plt.title("First ratios")
plt.hist(first_ratios, bins=n_bins)
plt.show()
    
    
    
