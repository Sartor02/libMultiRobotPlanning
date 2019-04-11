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
import re
import collections
from functools import reduce



FileData = collections.namedtuple('FileData', ['filename', 'agents', 'iter', 'trial', 'seed'])
kTimeout = 300

def filename_to_filedata(l):
  regex = r"datasave\/xstar_idv_agents_(\d\d\d|\d\d|\d)_iter_(\d\d\d|\d\d|\d)_trial_(\d\d\d|\d\d|\d)_seed_(\d\d\d\d\d\d|\d\d\d\d\d|\d\d\d\d|\d\d\d|\d\d|\d).result"
  matches = re.finditer(regex, l, re.MULTILINE)
  match = list(matches)[0]
  agents = int(match.group(1))
  itr = int(match.group(2))
  trial = int(match.group(3))
  seed = int(match.group(4))
  return FileData(l, agents, itr, trial, seed)

def get_line(ls, string, t):
  fls = [l for l in ls if string in l]
  if len(fls) > 1:
    print(string,"is not unique")
  elif len(fls) < 1:
    print(string,"is not found")
  assert(len(fls) == 1)
  l = fls[0]
  l = l.replace(string, '').replace(':','').strip()
  return t(l)

def get_total_first_plan_time(filename):
  ls = open(filename, 'r').readlines()
  if len(ls) == 0:
    return kTimeout
  time_individual_plan = get_line(ls, "time_individual_plan", float)
  time_first_plan = get_line(ls, "time_first_plan", float)
  return time_individual_plan + time_first_plan

def get_optimal_time_or_timeout(filename):
  ls = open(filename, 'r').readlines()
  if len(ls) == 0:
    return kTimeout
  if get_line(ls, "is_optimal", bool):
    return get_line(ls, "Total time", float)
  else:
    return kTimeout
  
def get_field(filename, field, t, default):
  ls = open(filename, 'r').readlines()
  if len(ls) == 0:
    return default
  return get_line(ls, field, t)

def get_percentile(data, percentile):
  percentile = percentile / 100.0
  assert(percentile <= 1.0 and percentile > 0)
  data.sort()
  diff = (1.0 - percentile) / 2.0
  high_idx = int((1.0 - diff) * (len(data) - 1))
  mid_idx = int(0.5 * (len(data) - 1))
  low_idx = int(diff * (len(data) - 1))
  high = data[high_idx]
  mid = data[mid_idx]
  low = data[low_idx]
  return (high, mid, low)

def add_to_dict(acc, e):
  l = acc.get(e[0], list())
  l.append(e[1])
  acc[e[0]] = l
  return acc

  
file_datas = [filename_to_filedata(f) for f in glob.glob('datasave/*.result')]
agents_first_times_lst = sorted([(d.agents, get_total_first_plan_time(d.filename)) for d in file_datas])
agents_optimal_times_lst = sorted([(d.agents, get_optimal_time_or_timeout(d.filename)) for d in file_datas])

def plt_cis(agents_times_lst, title):
  agents_to_times_dict = reduce(add_to_dict, agents_times_lst , dict())
  agents_to_100_bounds_lst = [[k] + list(get_percentile(v, 100)) for k, v in agents_to_times_dict.items()]
  agents_to_95_bounds_lst = [[k] + list(get_percentile(v, 95)) for k, v in agents_to_times_dict.items()]
  agents_to_90_bounds_lst = [[k] + list(get_percentile(v, 90)) for k, v in agents_to_times_dict.items()]
  agents_to_75_bounds_lst = [[k] + list(get_percentile(v, 75)) for k, v in agents_to_times_dict.items()]
  xs, hs, ms, ls = zip(*agents_to_100_bounds_lst)
  plt.plot(xs, ls, color='gray')
  plt.plot(xs, hs, color='gray')
  xs, hs, ms, ls = zip(*agents_to_95_bounds_lst)
  plt.plot(xs, ls, color='red')
  plt.plot(xs, hs, color='red')
  plt.fill_between(xs, ls, hs,
                      where=ls <= hs,
                      facecolor=(0.8, 0, 0, 0.5), interpolate=True, linewidth=0.0)
  xs, hs, ms, ls = zip(*agents_to_90_bounds_lst)
  plt.plot(xs, ls, color='blue')
  plt.plot(xs, hs, color='blue')
  plt.fill_between(xs, ls, hs,
                      where=ls <= hs,
                      facecolor=(0, 0, 0.8, 0.5), interpolate=True, linewidth=0.0)
  xs, hs, ms, ls = zip(*agents_to_75_bounds_lst)
  plt.plot(xs, ls, color='green')
  plt.plot(xs, ms, color='green')
  plt.plot(xs, hs, color='green')
  plt.fill_between(xs, ls, hs,
                      where=ls <= hs,
                      facecolor=(0, 0.8, 0, 0.5), interpolate=True, linewidth=0.0)
  plt.yscale('log')
  plt.title(title)
  plt.ylabel("Time (seconds)")
  plt.xlabel("Number of agents")
  plt.xticks(xs)


plt.subplot(211)
plt_cis(agents_first_times_lst, "Time to first solution")
plt.axhline(kTimeout, color='black', lw=1, linestyle='--')
plt.subplot(212)
plt_cis(agents_optimal_times_lst, "Time to optimal solution")
plt.axhline(kTimeout, color='black', lw=1, linestyle='--')
plt.show()

num_agents_in_window_optimal_times_lst = sorted([(get_field(d.filename, "num_max_agents_in_window", int, None), get_optimal_time_or_timeout(d.filename)) for d in file_datas if get_field(d.filename, "num_max_agents_in_window", int, None) is not None])

def plt_boxplot(num_agents_in_window_times_lst, title):
  num_agents_in_window_to_optimal_times_dict = reduce(add_to_dict, num_agents_in_window_times_lst , dict())
  num_agents_in_window_to_optimal_times_dict = {k : v for k, v in num_agents_in_window_to_optimal_times_dict.items() if k >= 2}
  keys_values = [(k, sorted(v)) for k,v in sorted(num_agents_in_window_to_optimal_times_dict.items(), key=lambda kv: kv[0])]
  ks, vs = zip(*keys_values)
  plt.boxplot(vs, notch=False)
  plt.gca().set_xticklabels(ks)
  plt.xlabel("Maximum number of agents in single interaction")
  plt.ylabel("Time (seconds)")
  plt.title(title)
  
plt_boxplot(num_agents_in_window_optimal_times_lst, "Number of agents in interaction vs time to optimal solution time")
plt.axhline(kTimeout, color='black', lw=1, linestyle='--')
plt.yscale('log')
plt.show()


