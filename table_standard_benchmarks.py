#!/usr/bin/env python3
import subprocess
import shared_helpers as sh
import signal
import shlex
import sys
import os
import glob
import argparse
cwd = os.getcwd()
dir_path = os.path.dirname(os.path.realpath(__file__))

name = "Namespaceagents50memory_limit30scen_name{}timeout300trials25".format("w_woundedcoast")

print("xstar_data_lst_{}".format(name))

# Ensures data can be reloaded properly
xstar_data_lst = sh.read_from_file("xstar_data_lst_{}".format(name))
cbs_data_lst = sh.read_from_file("cbs_data_lst_{}".format(name))
print("afs_data_lst_{}".format(name))
afs_data_lst = sh.read_from_file("afs_data_lst_{}".format(name))
mstar_data_lst = sh.read_from_file("mstar_data_lst_{}".format(name))

def get_first_runtime(e):
  try:
    return e.runtimes[0]
  except:
    return e.runtimes

def get_optimal_runtime(e):
  try:
    return e.runtimes[-1]
  except:
    return e.runtimes

def percentiles(lst):
    lst.sort()
    l = len(lst)
    b = lst[int(l * 0.025)]
    m = lst[int(l * 0.5)]
    t = lst[int(l * 0.975)]
    return b, m, t



for n, lst in [("X*", xstar_data_lst), ("AFS", afs_data_lst), ("CBS", cbs_data_lst), ("M*", mstar_data_lst)]:
  firsts = [get_first_runtime(e) for e in lst]
  optimals = [get_optimal_runtime(e) for e in lst]
  
  firsts_str = str(percentiles(firsts))[1:-1]
  optimals_str = str(percentiles(optimals))[1:-1]
  
  if firsts_str == optimals_str:
    firsts_str = "--"
  
  print(n, "&", firsts_str, "&", optimals_str, " \\\\ \\hline")


