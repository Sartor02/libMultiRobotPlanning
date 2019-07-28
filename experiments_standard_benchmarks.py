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


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("agents", type=int, help="Number of agents")
    parser.add_argument("scen_name", type=str, help="Scen name")
    parser.add_argument("trials", type=int, help="Number of trials")
    parser.add_argument("timeout", type=int, help="Run timeout")
    parser.add_argument("memory_limit", type=int, help="Memory limit (GB)")
    return parser.parse_args()

args = get_args()

assert(os.path.isdir("./datasave"))

def args_to_string(args):
    return str(args).replace('(', '').replace(')', '').replace(' ', '').replace('=', '').replace(',', '').replace("'", "")


generic_map = "simple_test{}.yaml".format(args_to_string(args))
afs_map = "afs_map_file{}.map".format(args_to_string(args))
afs_agents = "afs_agents_file{}.agents".format(args_to_string(args))  


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

def generate_new_scenario(agents, scen_name, iteration):
    cmd = "./example/standard_benchmark_converter.py ./scenarios/scen-random/{}-random-{}.scen ./scenarios/{}.map tmp".format(scen_name, iteration + 1, scen_name)
    run_with_current_proc(cmd)
    cmd = "mv tmp_{}_agents.yaml {}".format(agents, generic_map)
    run_with_current_proc(cmd)
    for f in  glob.glob("tmp_*.yaml"):
        os.remove(f)
    cmd = "./yaml_to_afs_converter.py {} {} {}".format(generic_map, afs_map, afs_agents)
    print(cmd)
    ret_val = run_with_current_proc(cmd)


def run_mstar(timeout):
    cmd = "timeout {} mstar_public/cpp/main -i {} -o simple_test_mstar.result".format(timeout, generic_map)
    retcode = run_with_current_proc(cmd)
    if retcode != 0:
        return timeout
    f = open("simple_test_mstar.result")
    runtime = [float(x.strip().replace("runtime:", "")) for x in f.readlines() if "runtime:" in x][0]
    return runtime


def run_xstar(timeout, iteration):
    prefix = "datasave/xstar_compare_runtime_agents_{}_iter_{}_density_{}_trial_"\
             .format(args.agents,
                     iteration,
                     0)
    cmd = "./collect_data_xstar.py {} {} {} {} {} _seed_{}.result"\
          .format(generic_map,
                  args.timeout,
                  1,
                  args.memory_limit,
                  prefix,
                  0)
    if run_with_current_proc(cmd) != 0:
        print("Error running X*")
        return ([0], [args.timeout])
    result_file = list(glob.glob(prefix + "*"))[0]
    try:
        f = open(result_file, 'r')
        ls = f.readlines()
        bounds = get_line(ls, "successive_bounds", eval)
        times = get_line(ls, "successive_runtimes", eval)
        return (bounds, times)
    except:
        return ([0], [args.timeout])


def run_afs(timeout):
    cmd = "timeout {} afs/AnytimeMAPF/driver --map {} --agents {} --export_results afs_results{}.out --time_limit {}".format(timeout + 2, afs_map, afs_agents, args_to_string(args), timeout)
    print(cmd)
    retcode = run_with_current_proc(cmd)
    if retcode != 0:
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
    cmd = "timeout {} release/cbs -i {} -o simple_test_cbs.result".format(timeout, generic_map)
    retcode = run_with_current_proc(cmd)
    if retcode != 0:
        print("CBS timeout")
        return timeout
    f = open("simple_test_cbs.result")
    runtime = [float(x.strip().replace("runtime:", "")) for x in f.readlines() if "runtime:" in x][0]
    return runtime

xstar_data_lst = []
mstar_data_lst = []
afs_data_lst = []
cbs_data_lst = []

for i in range(args.trials):
    print("Trial {}:==============================================".format(i))
    
    clean_up()
    
    generate_new_scenario(args.agents, args.scen_name, i)

    print("X*")
    xstar_bounds, xstar_runtimes = run_xstar(args.timeout, i)
    xstar_data_lst.append(sh.XStarData(0,
                                       0,
                                       0,
                                       args.agents,
                                       args.timeout,
                                       xstar_bounds,
                                       xstar_runtimes))

    print("AFS")  
    afs_bounds, afs_runtimes  = run_afs(args.timeout)
    afs_data_lst.append(sh.AFSData(0,
                                   0,
                                   0,
                                   args.agents,
                                   args.timeout,
                                   afs_bounds,
                                   afs_runtimes))
  
    print("CBS")
    cbs_runtime = run_cbs(args.timeout)
    cbs_data_lst.append(sh.CBSData(0,
                                   0,
                                   0,
                                   args.agents,
                                   args.timeout,
                                   cbs_runtime))

    print("M*")
    mstar_runtime = run_mstar(args.timeout)
    mstar_data_lst.append(sh.MStarData(0,
                                       0,
                                       0,
                                       args.agents,
                                       args.timeout,
                                       mstar_runtime))

xstar_out_file = "xstar_data_lst_{}".format(args_to_string(args))
print("X* out file:", xstar_out_file)
sh.save_to_file(xstar_out_file, xstar_data_lst)
cbs_out_file = "cbs_data_lst_{}".format(args_to_string(args))
print(cbs_out_file)
sh.save_to_file(cbs_out_file, cbs_data_lst)
afs_out_file = "afs_data_lst_{}".format(args_to_string(args))
print(afs_out_file)
sh.save_to_file(afs_out_file, afs_data_lst)
mstar_out_file = "mstar_data_lst_{}".format(args_to_string(args))
sh.save_to_file(mstar_out_file, mstar_data_lst)

# Ensures data can be reloaded properly
xstar_data_lst = sh.read_from_file(xstar_out_file)
cbs_data_lst = sh.read_from_file(cbs_out_file)
afs_data_lst = sh.read_from_file(afs_out_file)
mstar_data_lst = sh.read_from_file(mstar_out_file)

clean_up()
