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

current_proc = None

def clean_up():
    if current_proc is not None:
        current_proc.terminate()
    for f in glob.glob("*.timing"):
        os.remove(f)
    for f in glob.glob("*.yaml"):
        os.remove(f)
    for f in glob.glob("*.out"):
        os.remove(f)
    for f in glob.glob("*.map"):
        os.remove(f)
    for f in glob.glob("*.agents"):
        os.remove(f)
    for f in glob.glob("*.result"):
        os.remove(f)


def signal_handler(sig, frame):
    return
    print(sys.argv[0], 'Signal caught')
    clean_up()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

assert(os.path.isdir("./datasave"))


generic_map = "swap_scenario"

def get_line(ls, string, t):
    string = string.strip()
    if string[-1] != ':':
        string += ':'
    fls = [l for l in ls if string in l]
    if len(fls) > 1:
        print(string, "is not unique")
    elif len(fls) < 1:
        print(string, "is not found")
    assert(len(fls) == 1)
    line = fls[0]
    line = line.replace(string, '').replace(':', '').strip()
    return t(line)

def run_with_current_proc(cmd):
    global current_proc
    FNULL = open(os.devnull, 'w')
    current_proc = subprocess.Popen(shlex.split(cmd), stdout=FNULL, stderr=subprocess.STDOUT)
    current_proc.wait()
    retcode = current_proc.returncode
    current_proc = None
    return retcode
  
# def generate_new_scenario(agents, obs_density, seed):
#     ret_val = run_with_current_proc("./benchmark_generator.py {} {} {} {} {} {} {} --seed {}".format(agents, 100, 100, obs_density, generic_map, "DELETEME", "DELETEME", seed))
#     if ret_val != 0:
#         print("Genrate failed")
#         exit(-1)

def get_idx_greater_than(lst, val):
    for i, v in enumerate(lst):
        if (v >  val):
            return i
    return "never"

def run_once(idx):
    clean_up()
    #generate_new_scenario(10, 0.1, idx)
    cmd = "timeout 30 ./release/xstar -i {} -o DELETEME -t swap_tst".format(generic_map)
    print(cmd)
    run_with_current_proc(cmd)
    sorted_lst = sorted([f for f in glob.glob("iteration_swap_tst_*.timing")], key=lambda f:
 int(f.replace("iteration_swap_tst_", "").replace(".timing", "")))
    if len(sorted_lst) == 0:
        return (30, 30)
    final_file = sorted_lst[-1]
    lines = open(final_file).readlines()
    runtimes = get_line(lines, "successive_runtimes:", eval)
    first_runtime = runtimes[0]
    optimal_runtime = runtimes[-1]
    overlap_astar_median_idx = get_idx_greater_than(runtimes, 0.2623)
    if not get_line(lines, "is_optimal:", bool):
        optimal_runtime = 30
    print("First", first_runtime, "Optimal", optimal_runtime, "Overlap A*", overlap_astar_median_idx, "Total iterations:", len(runtimes))
    clean_up()
    return (first_runtime, optimal_runtime)


def percentiles(lst):
    lst.sort()
    l = len(lst)
    b = lst[int(l * 0.025)]
    m = lst[int(l * 0.5)]
    t = lst[int(l * 0.975)]
    return b, m, t

firsts = []
optimals = []
for i in range(30):
    f, o = run_once(i)
    firsts.append(f)
    optimals.append(o)
    
firsts.sort()
optimals.sort()

print(percentiles(firsts))
print(percentiles(optimals))

