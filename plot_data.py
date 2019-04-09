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

def percent_with_timeouts(runtimes, p):
  filtered = sorted([e for e in runtimes])
  return filtered[int(len(filtered) * p)]

def percent_without_timeouts(runtimes, p):
  filtered = sorted([e for e in runtimes if e != timeout])
  return filtered[int(len(filtered) * p)]

def fraction_not_timeouts(runtimes):
  not_timeouts = len([e for e in runtimes if e != timeout])
  return not_timeouts / len(runtimes)

xstar_data_lst = read_from_file("xstar_data_lst_{}".format(args_to_str(args)))
cbs_data_lst = read_from_file("cbs_data_lst_{}".format(args_to_str(args)))
afs_data_lst = read_from_file("afs_data_lst_{}".format(args_to_str(args)))
mstar_data_lst = read_from_file("mstar_data_lst_{}".format(args_to_str(args)))

num_agents = xstar_data_lst[0].num_agents
width = xstar_data_lst[0].width
height = xstar_data_lst[0].height
obs_density = xstar_data_lst[0].obs_density
num_trials = len(xstar_data_lst)
timeout = xstar_data_lst[0].timeout


xstar_first_runtimes = [mean_runtime_at_idx(0, e) for e in xstar_data_lst]
xstar_final_runtimes = [mean_runtime_at_idx(-1, e) for e in xstar_data_lst]
xstar_first_failures = len([e for e in xstar_first_runtimes if e == timeout])
xstar_final_failures = len([e for e in xstar_final_runtimes if e == timeout])
afs_first_runtimes = [mean_runtime_at_idx(0, e) for e in afs_data_lst]
afs_final_runtimes = [mean_runtime_at_idx(-1, e) for e in afs_data_lst]
afs_first_failures = len([e for e in afs_first_runtimes if e == timeout])
afs_final_failures = len([e for e in afs_final_runtimes if e == timeout])
cbs_final_runtimes = [mean_runtime_at_idx(-1, e) for e in cbs_data_lst]
cbs_final_failures = len([e for e in cbs_final_runtimes if e == timeout])
mstar_final_runtimes = [mean_runtime_at_idx(-1, e) for e in mstar_data_lst]
mstar_final_failures = len([e for e in cbs_final_runtimes if e == timeout])


cbs_optimal_ratios = [c/x for c, x in zip(cbs_final_runtimes, xstar_final_runtimes)]
cbs_first_ratios = [c/x for c, x in zip(cbs_final_runtimes, xstar_first_runtimes)]
cbs_percent_optimal_1_or_more = len([e for e in cbs_optimal_ratios if e > 1.0]) / len(cbs_optimal_ratios)
cbs_percent_first_1_or_more = len([e for e in cbs_first_ratios if e > 1.0]) / len(cbs_first_ratios)
cbs_percent_optimal_1_or_less = len([e for e in cbs_optimal_ratios if e < 1.0]) / len(cbs_optimal_ratios)
cbs_percent_first_1_or_less = len([e for e in cbs_first_ratios if e < 1.0]) / len(cbs_first_ratios)

afs_optimal_ratios = [a/x for a, x in zip(afs_final_runtimes, xstar_final_runtimes)]
afs_first_ratios = [a/x for a, x in zip(afs_first_runtimes, xstar_first_runtimes)]
afs_percent_optimal_1_or_more = len([e for e in afs_optimal_ratios if e > 1.0]) / len(afs_optimal_ratios)
afs_percent_first_1_or_more = len([e for e in afs_first_ratios if e > 1.0]) / len(afs_first_ratios)
afs_percent_optimal_1_or_less = len([e for e in afs_optimal_ratios if e < 1.0]) / len(afs_optimal_ratios)
afs_percent_first_1_or_less = len([e for e in afs_first_ratios if e < 1.0]) / len(afs_first_ratios)

n_bins = 50

# plt.subplot(211)
# plt.title("First solution for {} agents, on {}x{} grid with {}% obstacle density over {} trials with {} second timeout".format(num_agents, width, height, obs_density * 100, num_trials, timeout))
# plt.hist(get_95_percent_slice(cbs_first_ratios), bins=compute_bins(get_95_percent_slice(cbs_first_ratios), n_bins))
# plt.axvline(x=1, color="red", alpha = 0.4, linestyle='--')
# plt.gca().set_xscale("log")
# plt.xlabel("Middle 95% of data for CBS Runtime / X* Runtime (larger is better for X*)\nPercent X* better runtime: {:3.3f}% Worse: {:3.3f}%\nCBS Failures: {} First X* Failures: {}".format(cbs_percent_first_1_or_more * 100, cbs_percent_first_1_or_less * 100, cbs_final_failures, xstar_first_failures))
# plt.ylabel("# occurrences")

# plt.subplot(212)
# plt.title("Optimal solution for {} agents, on {}x{} grid with {}% obstacle density over {} trials with {} second timeout".format(num_agents, width, height, obs_density * 100, num_trials, timeout))
# plt.hist(get_95_percent_slice(cbs_optimal_ratios), bins=compute_bins(get_95_percent_slice(cbs_optimal_ratios), n_bins))
# plt.axvline(x=1, color="red", alpha = 0.4, linestyle='--')
# plt.gca().set_xscale("log")
# plt.xlabel("Middle 95% of data for CBS Runtime / X* Runtime (larger is better for X*)\nPercent X* better runtime: {:3.3f}% Worse: {:3.3f}%\nCBS Failures: {} Final X* Failures: {}".format(cbs_percent_optimal_1_or_more * 100, cbs_percent_optimal_1_or_less * 100, cbs_final_failures, xstar_final_failures))
# plt.ylabel("# occurrences")

# plt.show()

# plt.subplot(211)
# plt.title("First solution for {} agents, on {}x{} grid with {}% obstacle density over {} trials with {} second timeout".format(num_agents, width, height, obs_density * 100, num_trials, timeout))
# plt.hist(get_95_percent_slice(afs_first_ratios), bins=compute_bins(get_95_percent_slice(afs_first_ratios), n_bins))
# plt.axvline(x=1, color="red", alpha = 0.4, linestyle='--')
# plt.gca().set_xscale("log")
# plt.xlabel("Middle 95% of data for AFS Runtime / X* Runtime (larger is better for X*)\nPercent X* better runtime: {:3.3f}% Worse: {:3.3f}%\nFirst AFS Failures: {} First X* Failures: {}".format(afs_percent_first_1_or_more * 100, afs_percent_first_1_or_less * 100, afs_first_failures, xstar_first_failures))
# plt.ylabel("# occurrences")

# plt.subplot(212)
# plt.title("Optimal solution for {} agents, on {}x{} grid with {}% obstacle density over {} trials with {} second timeout".format(num_agents, width, height, obs_density * 100, num_trials, timeout))
# plt.hist(get_95_percent_slice(afs_optimal_ratios), bins=compute_bins(get_95_percent_slice(afs_optimal_ratios), n_bins))
# plt.axvline(x=1, color="red", alpha = 0.4, linestyle='--')
# plt.gca().set_xscale("log")
# plt.xlabel("Middle 95% of data for AFS Runtime / X* Runtime (larger is better for X*)\nPercent X* better runtime: {:3.3f}% Worse: {:3.3f}%\nFinal AFS Failures: {} Final X* Failures: {}".format(afs_percent_optimal_1_or_more * 100, afs_percent_optimal_1_or_less * 100, afs_final_failures, xstar_final_failures))
# plt.ylabel("# occurrences")

# plt.show()

# plt.subplot(211)
# plt.title("First runtimes")
# plt.xlabel("X* time to first solution (seconds) Green: 50%, Blue: 75%, Red: 90%; Dotted: with timeout, Dashed: without timeout\nPercentage not timed out: {:3.3f}".format(fraction_not_timeouts(xstar_first_runtimes) * 100))
# plt.axvline(x=percent_without_timeouts(xstar_first_runtimes, 0.5), color="green", alpha = 0.4, linestyle='--')
# plt.axvline(x=percent_without_timeouts(xstar_first_runtimes, 0.75), color="blue", alpha = 0.4, linestyle='--')
# plt.axvline(x=percent_without_timeouts(xstar_first_runtimes, 0.9), color="red", alpha = 0.4, linestyle='--')
# plt.axvline(x=percent_with_timeouts(xstar_first_runtimes, 0.5), color="green", alpha = 0.4, linestyle=':')
# plt.axvline(x=percent_with_timeouts(xstar_first_runtimes, 0.75), color="blue", alpha = 0.4, linestyle=':')
# plt.axvline(x=percent_with_timeouts(xstar_first_runtimes, 0.9), color="red", alpha = 0.4, linestyle=':')
# plt.hist(xstar_first_runtimes, bins=120)

# plt.subplot(212)
# plt.title("Optimal runtimes")
# plt.axvline(x=percent_without_timeouts(xstar_final_runtimes, 0.5), color="green", alpha = 0.4, linestyle='--')
# plt.axvline(x=percent_without_timeouts(xstar_final_runtimes, 0.75), color="blue", alpha = 0.4, linestyle='--')
# plt.axvline(x=percent_without_timeouts(xstar_final_runtimes, 0.9), color="red", alpha = 0.4, linestyle='--')
# plt.axvline(x=percent_with_timeouts(xstar_final_runtimes, 0.5), color="green", alpha = 0.4, linestyle=':')
# plt.axvline(x=percent_with_timeouts(xstar_final_runtimes, 0.75), color="blue", alpha = 0.4, linestyle=':')
# plt.axvline(x=percent_with_timeouts(xstar_final_runtimes, 0.9), color="red", alpha = 0.4, linestyle=':')
# plt.hist(xstar_final_runtimes, bins=120)
# plt.xlabel("X* time to optimal solution (seconds) Green: 50%, Blue: 75%, Red: 90%; Dotted: with timeout, Dashed: without timeout\nPercentage not timed out: {:3.3f}".format(fraction_not_timeouts(xstar_final_runtimes) * 100))

# plt.show()

# def get_95_max(lsts):
#   return max([max(get_95_percent_slice(lst)) for lst in lsts])

# max_val = get_95_max([xstar_first_runtimes, mstar_final_runtimes, cbs_final_runtimes, afs_first_runtimes])
# print(max_val)
# bin_size = 2

# bins = np.linspace(0, max_val, int(max_val / bin_size))

# plt.hist(get_95_percent_slice(xstar_first_runtimes), bins=bins, alpha=0.5, color='red')
# plt.hist(get_95_percent_slice(mstar_final_runtimes), bins=bins, alpha=0.5, color='green')
# plt.hist(get_95_percent_slice(cbs_final_runtimes), bins=bins, alpha=0.5, color='blue')
# plt.hist(get_95_percent_slice(afs_first_runtimes), bins=bins, alpha=0.5, color='purple')

# plt.show()

plt.subplot(211)
plt.title("First runtimes")
plt.plot()

plt.subplot(212)
plt.title("Optimal runtimes")
plt.axvline(x=percent_without_timeouts(xstar_final_runtimes, 0.5), color="green", alpha = 0.4, linestyle='--')
plt.axvline(x=percent_without_timeouts(xstar_final_runtimes, 0.75), color="blue", alpha = 0.4, linestyle='--')
plt.axvline(x=percent_without_timeouts(xstar_final_runtimes, 0.9), color="red", alpha = 0.4, linestyle='--')
plt.axvline(x=percent_with_timeouts(xstar_final_runtimes, 0.5), color="green", alpha = 0.4, linestyle=':')
plt.axvline(x=percent_with_timeouts(xstar_final_runtimes, 0.75), color="blue", alpha = 0.4, linestyle=':')
plt.axvline(x=percent_with_timeouts(xstar_final_runtimes, 0.9), color="red", alpha = 0.4, linestyle=':')
plt.hist(xstar_final_runtimes, bins=120)
plt.xlabel("X* time to optimal solution (seconds) Green: 50%, Blue: 75%, Red: 90%; Dotted: with timeout, Dashed: without timeout\nPercentage not timed out: {:3.3f}".format(fraction_not_timeouts(xstar_final_runtimes) * 100))

