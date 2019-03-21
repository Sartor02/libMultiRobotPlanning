#!/usr/bin/env python3
import subprocess
from colorama import Fore, Style
from matplotlib import pyplot as plt
import argparse
from shared_helpers import *
import numpy as np



args = get_args();

def mean_runtime_at_idx(idx, data):
  pts = []
  for r in data.runtimes:
    pts.append(r.datalst[idx])
  return np.average(pts)

def get_95_percent_slice(data):
  data.sort()
  l = float(len(data))
  min_idx = int(l * 0.025)
  max_idx = int(l * 0.975)
  return data[min_idx:max_idx]

def compute_bins(data, n_bins):
  return np.logspace(np.log10(min(data)),np.log10(max(data)), n_bins)

xstar_data_lst = read_from_file("xstar_data_lst_{}".format(args_to_str(args)))
cbs_data_lst = read_from_file("cbs_data_lst_{}".format(args_to_str(args)))

num_agents = xstar_data_lst[0].num_agents
width = xstar_data_lst[0].width
height = xstar_data_lst[0].height
obs_density = xstar_data_lst[0].obs_density
num_trials = len(xstar_data_lst)
timeout = xstar_data_lst[0].timeout


xstar_first_runtimes = [mean_runtime_at_idx(0, e) for e in xstar_data_lst]
xstar_final_runtimes = [mean_runtime_at_idx(-1, e) for e in xstar_data_lst]
cbs_final_runtimes = [mean_runtime_at_idx(-1, e) for e in cbs_data_lst]



optimal_ratios = [c/x for c, x in zip(cbs_final_runtimes, xstar_final_runtimes)]
first_ratios = [c/x for c, x in zip(cbs_final_runtimes, xstar_first_runtimes)]
percent_optimal_1_or_more = len([e for e in optimal_ratios if e >= 1.0]) / len(optimal_ratios)
percent_first_1_or_more = len([e for e in first_ratios if e >= 1.0]) / len(first_ratios)

n_bins = 50

plt.subplot(211)
plt.title("First solution for {} agents, on {}x{} grid with {}% obstacle density over {} trials with {} second timeout".format(num_agents, width, height, obs_density * 100, num_trials, timeout))
plt.hist(get_95_percent_slice(first_ratios), bins=compute_bins(get_95_percent_slice(first_ratios), n_bins))
plt.axvline(x=1, color="red", alpha = 0.4, linestyle='--')
plt.gca().set_xscale("log")
plt.xlabel("Middle 95% of data for CBS Runtime / X* Runtime (larger is better for X*) Percent X* same or better runtime: {}%".format(percent_first_1_or_more * 100))
plt.ylabel("# occurrences")

plt.subplot(212)
plt.title("Optimal solution for {} agents, on {}x{} grid with {}% obstacle density over {} trials with {} second timeout".format(num_agents, width, height, obs_density * 100, num_trials, timeout))
plt.hist(get_95_percent_slice(optimal_ratios), bins=compute_bins(get_95_percent_slice(optimal_ratios), n_bins))
plt.axvline(x=1, color="red", alpha = 0.4, linestyle='--')
plt.gca().set_xscale("log")
plt.xlabel("Middle 95% of data for CBS Runtime / X* Runtime (larger is better for X*) Percent X* same or better runtime: {}%".format(percent_optimal_1_or_more * 100))
plt.ylabel("# occurrences")

plt.show()

    
    
