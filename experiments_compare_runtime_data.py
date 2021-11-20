#!/usr/bin/env python3
import subprocess
import shared_helpers as sh
import yaml_to_movingai as yaml2mvai
import signal
import shlex
import sys
import io
import os
import glob
import argparse
import pandas as pd
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
    for f in glob.glob("TEMPOUT*"):
        os.remove(f)
    for f in glob.glob("*.xml"):
        os.remove(f)
    for f in glob.glob("*_log"):
        os.remove(f)

def signal_handler(sig, frame):
    print(sys.argv[0], 'Signal caught')
    clean_up()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("agents", type=int, help="Number of agents")
    parser.add_argument("width", type=int, help="Map width")
    parser.add_argument("height", type=int, help="Map height")
    parser.add_argument("obs_density", type=float, help="Obstacle density")
    parser.add_argument("trials", type=int, help="Number of trials")
    parser.add_argument("timeout", type=int, help="Run timeout")
    parser.add_argument("memory_limit", type=int, help="Memory limit (GB)")
    return parser.parse_args()

args = get_args()
outfile_infix = "supplemental_"

assert(os.path.isdir("./datasave"))

def args_to_string(args):
    return str(args).replace('(', '').replace(')', '').replace(' ', '').replace('=', '').replace(',', '')


generic_map = "simple_test{}.yaml".format(args_to_string(args))
afs_map = "afs_map_file{}.map".format(args_to_string(args))
afs_agents = "afs_agents_file{}.agents".format(args_to_string(args))
pr_map = "pr_map_file{}.map".format(args_to_string(args))
pr_agents = "pr_agents_file{}.agents".format(args_to_string(args))
lns_map = "lns_map_file{}.map".format(args_to_string(args))
lns_agents = "lns_agents_file{}.agents".format(args_to_string(args))


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


def generate_new_scenario(agents, width, height, obs_density, seed):
    cmd = "./benchmark_generator.py {} {} {} {} {} {} {} {} {} --seed {}".format(agents, width, height, obs_density, generic_map, afs_map, afs_agents, pr_map, pr_agents, seed)
    ret_val = subprocess.call(cmd, shell=True)
    if ret_val != 0:
        print("Genrate failed")
        exit(-1)


def run_with_current_proc(cmd, shell=False):
    global current_proc
    FNULL = open(os.devnull, 'w')
    if not shell:
        current_proc = subprocess.Popen(shlex.split(cmd), stdout=FNULL, stderr=subprocess.STDOUT)
    else:
        current_proc = subprocess.Popen(cmd, shell=True, stdout=FNULL, stderr=subprocess.STDOUT)
    current_proc.wait()
    retcode = current_proc.returncode
    current_proc = None
    return retcode

def run_and_get_output(cmd):
    # return subprocess.run(shlex.split(cmd), capture_output=True, text=True).stdout
    return subprocess.run(shlex.split(cmd), stdout=subprocess.PIPE).stdout.decode('utf-8')


def run_mstar(timeout):
    cmd = "timeout {} mstar_public/cpp/main -i {} -o simple_test_mstar{}.result".format(timeout, generic_map, args_to_string(args))
    retcode = run_with_current_proc(cmd)
    if retcode != 0:
        return timeout
    f = open("simple_test_mstar{}.result".format(args_to_string(args)))
    runtime = [float(x.strip().replace("runtime:", "")) for x in f.readlines() if "runtime:" in x][0]
    return runtime


def run_xstar(timeout, iteration):
    prefix = "datasave/xstar_compare_runtime_agents_{}_iter_{}_density_{}_trial_"\
             .format(args.agents,
                     iteration,
                     args.obs_density)
    cmd = "./collect_data_xstar.py {} {} {} {} {} _seed_{}.result"\
          .format(generic_map,
                  args.timeout,
                  1,
                  args.memory_limit,
                  prefix,
                  seed)
    run_with_current_proc(cmd)
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
    cmd = "timeout {} release/cbs -i {} -o simple_test_cbs{}.result".format(timeout, generic_map, args_to_string(args))
    retcode = run_with_current_proc(cmd)
    if retcode != 0:
        print("CBS timeout")
        return timeout
    f = open("simple_test_cbs{}.result".format(args_to_string(args)))
    runtime = [float(x.strip().replace("runtime:", "")) for x in f.readlines() if "runtime:" in x][0]
    return runtime

def run_acbs(timeout):
    cmd = "timeout {} release/acbs -i {} -o simple_test_acbs{}.result".format(timeout, generic_map, args_to_string(args))
    retcode = run_with_current_proc(cmd)
    df = pd.read_csv("simple_test_acbs{}.result".format(args_to_string(args)))
    runtimes = list(df['Runtime'])
    
    ratios = list(df['Ratio'])
    if len(runtimes) == 0:
        print('ACBS no first solution')
        return [timeout], [-1]
    if runtimes[-1] != -1:
        print('ACBS no optimal')
        runtimes.append(timeout)
        ratios.append(-1)
    if retcode != 0:
        print('ACBS timeout')
    return runtimes, ratios

def run_nrwcbs(timeout):
    cmd = "timeout {} release/nrwcbs -i {} -o simple_test_nrwcbs{}.result".format(timeout, generic_map, args_to_string(args))
    # sys.exit()
    retcode = run_with_current_proc(cmd)
    df = pd.read_csv("simple_test_nrwcbs{}.result".format(args_to_string(args)))
    runtimes = list(df['Runtime'])    
    ratios = list(df['Ratio'])
    
    if len(runtimes) == 0:
        print('NRWCBS no first solution')
        return [timeout], [-1]
    if runtimes[-1] != -1:
        print('NRWCBS no optimal')
        runtimes.append(timeout)
        ratios.append(-1)
    if retcode != 0:
        print('NRWCBS timeout')
    return runtimes, ratios

def run_lns(timeout):
    yaml2mvai.yaml_to_mvai(generic_map, lns_map, lns_agents)
    cmd = "release/lns -m {} -a {} -o temp.csv -k {} -t {} -s 1".format(lns_map, lns_agents, args.agents, timeout)
    data = run_and_get_output(cmd)
    print(cmd)
    df = pd.read_csv(io.StringIO(data))

    print(df)
    sys.exit()
    runtimes = list(df['Runtime'])
    costs = list(df['Cost'])
    if len(runtimes) == 0:
        print('LNS no first solution')
        return [timeout], [-1]
    return runtimes, costs

def run_nwcbs(timeout):
    cmd = "timeout {} release/nwcbs -i {} -o simple_test_nwcbs{}.result".format(timeout, generic_map, args_to_string(args))
    retcode = run_with_current_proc(cmd)
    df = pd.read_csv("simple_test_nwcbs{}.result".format(args_to_string(args)))
    runtimes = list(df['Runtime'])
    ratios = list(df['Ratio'])
    if len(runtimes) == 0:
        print('NWCBS no first solution')
        return [timeout], [-1]
    if runtimes[-1] != -1:
        print('NWCBS no optimal')
        runtimes.append(timeout)
        ratios.append(-1)
    if retcode != 0:
        print('NWCBS timeout')
    return runtimes, ratios

def run_pr(timeout):
    tempout = "TEMPOUT{}".format(args_to_string(args))
    cmd = "timeout {} Push-and-Rotate--CBS--PrioritizedPlanning/build/ASearch {} > {}".format(timeout, pr_map, tempout)
    retcode = run_with_current_proc(cmd, shell=True)
    if retcode != 0:
        print(retcode)
        print("PR timeout")
        return 0, timeout
    f = open(tempout)
    lines = f.readlines()
    f.close()
    try:
        runtime = [float(x.strip().replace("Runtime:", "")) for x in lines if "Runtime:" in x][0]
    except:
        print("Failed to read runtime")
        return 0, timeout
    try:
        planned_cost = [int(x.strip().replace("Cost:", "")) for x in lines if "Cost:" in x][0]
    except:
        print("Failed to get planned cost")
        return 0, runtime


    cmd = "timeout {} release/cbs -i {} -o simple_test_cbs{}.result".format(timeout*2, generic_map, args_to_string(args))
    retcode = run_with_current_proc(cmd)
    if retcode != 0:
        print("CBS timeout")
        return 0, runtime
    f = open("simple_test_cbs{}.result".format(args_to_string(args)))
    opt_cost = [int(x.strip().replace("cost:", "")) for x in f.readlines() if "cost:" in x][0]
    f.close()
    return (planned_cost / opt_cost, runtime)

xstar_data_lst = []
mstar_data_lst = []
afs_data_lst = []
cbs_data_lst = []
acbs_data_lst = []
nrwcbs_data_lst = []
nwcbs_data_lst = []
pr_data_lst = []
lns_data_lst = []

# xstar_first_list = []
# acbs_first_list = []
# cbs_list = []
# acbs_list = []
# xstar_list = []

DO_X = 1
DO_NRWCBS = 1
DO_CBS = 1
DO_NWCBS = 1
DO_LNS = 0

for i in range(args.trials):
    print("Trial {}:==============================================".format(i))
    seed = (i + args.width) * args.agents
    generate_new_scenario(args.agents, args.width, args.height, args.obs_density, seed)

    print(generic_map)

    if DO_X:
        print("X*")
        xstar_bounds, xstar_runtimes = run_xstar(args.timeout, i)
        xstar_data_lst.append(sh.XStarData(args.obs_density,
                                        args.width,
                                        args.height,
                                        args.agents,
                                        args.timeout,
                                        xstar_bounds,
                                        xstar_runtimes))

    # print("AFS")  
    # afs_bounds, afs_runtimes  = run_afs(args.timeout)
    # afs_data_lst.append(sh.AFSData(args.obs_density,
    #                                args.width,
    #                                args.height,
    #                                args.agents,
    #                                args.timeout,
    #                                afs_bounds,
    #                                afs_runtimes))
    
    if DO_CBS:
        print("CBS")
        cbs_runtime = run_cbs(args.timeout)
        cbs_data_lst.append(sh.CBSData(args.obs_density,
                                    args.width,
                                    args.height,
                                    args.agents,
                                    args.timeout,
                                    cbs_runtime))

    # print("ACBS")
    # acbs_runtimes, acbs_ratios = run_acbs(args.timeout)
    # acbs_data_lst.append(sh.ACBSData(args.obs_density,
    #                                args.width,
    #                                args.height,
    #                                args.agents,
    #                                args.timeout,
    #                                acbs_runtimes,
    #                                acbs_ratios))
    # if len(acbs_runtimes) > 2:
    #     acbs_runt = acbs_runtimes[-2]

    if DO_NRWCBS:
        print("NRWCBS")
        nrwcbs_runtimes, nrwcbs_ratios = run_nrwcbs(args.timeout)
        nrwcbs_data_lst.append(sh.ACBSData(args.obs_density,
                                    args.width,
                                    args.height,
                                    args.agents,
                                    args.timeout,
                                    nrwcbs_runtimes,
                                    nrwcbs_ratios))
        # print(nrwcbs_runtimes)
        # for i in range(1, len(nrwcbs_ratios)):
        #     if nrwcbs_ratios[i] > nrwcbs_ratios[i-1]:
        #         print("{}, {}".format(nrwcbs_ratios[i-1], nrwcbs_ratios[i]))
        #         sys.exit()

    # print(nrwcbs_runtimes, nrwcbs_ratios)
    
    # if len(nrwcbs_runtimes) > 2:
    #     nrw_runt = nrwcbs_runtimes[-2]

    if DO_LNS:
        print("LNS")
        lns_runtimes, lns_costs = run_lns(args.timeout)
        lns_data_lst.append(sh.LNSData(args.obs_density,
                                    args.width,
                                    args.height,
                                    args.agents,
                                    args.timeout,
                                    lns_runtimes,
                                    lns_costs))

    if DO_NWCBS:
        print("NWCBS")
        nwcbs_runtimes, nwcbs_ratios = run_nwcbs(args.timeout)
        nwcbs_data_lst.append(sh.NWCBSData(args.obs_density,
                                    args.width,
                                    args.height,
                                    args.agents,
                                    args.timeout,
                                    nwcbs_runtimes,
                                    nwcbs_ratios))

        # for i in range(1, len(nwcbs_ratios)):
        #     if nwcbs_ratios[i] > nwcbs_ratios[i-1]:
        #         print("{}, {}".format(nwcbs_ratios[i-1], nwcbs_ratios[i]))
        #         sys.exit()
            
    
    # if len(acbs_runtimes) > 2 and len(nrwcbs_runtimes) > 2 and acbs_runt > 2.5* nrw_runt:
    #     print("acbs: " + str(acbs_runt))
    #     print("nrw: " + str(nrw_runt))
    #     exit(0)

    # print("M*")
    # mstar_runtime = run_mstar(args.timeout)
    # mstar_data_lst.append(sh.MStarData(args.obs_density,
    #                                    args.width,
    #                                    args.height,
    #                                    args.agents,
    #                                    args.timeout,
    #                                    mstar_runtime))

    # print("PR")
    # pr_bounds, pr_runtimes = run_pr(args.timeout)
    # pr_data_lst.append(sh.PRData(args.obs_density,
    #                                args.width,
    #                                args.height,
    #                                args.agents,
    #                                args.timeout,
    #                                pr_bounds,
    #                                pr_runtimes))

if DO_X:
    sh.save_to_file("xstar_{}data_lst_{}".format(outfile_infix, args_to_string(args)), xstar_data_lst)
    xstar_data_lst = sh.read_from_file("xstar_{}data_lst_{}".format(outfile_infix, args_to_string(args)))

if DO_CBS:
    sh.save_to_file("cbs_{}data_lst_{}".format(outfile_infix, args_to_string(args)), cbs_data_lst)
    cbs_data_lst = sh.read_from_file("cbs_{}data_lst_{}".format(outfile_infix, args_to_string(args)))
# sh.save_to_file("afs_{}data_lst_{}".format(outfile_infix, args_to_string(args)), afs_data_lst)
# sh.save_to_file("mstar_{}data_lst_{}".format(outfile_infix, args_to_string(args)), mstar_data_lst)
# sh.save_to_file("pr_{}data_lst_{}".format(outfile_infix, args_to_string(args)), pr_data_lst)
# sh.save_to_file("acbs_{}data_lst_{}".format(outfile_infix, args_to_string(args)), acbs_data_lst)

if DO_NWCBS:
    sh.save_to_file("nwcbs_{}data_lst_{}".format(outfile_infix, args_to_string(args)), nwcbs_data_lst)
    nwcbs_data_lst = sh.read_from_file("nwcbs_{}data_lst_{}".format(outfile_infix, args_to_string(args)))
if DO_NRWCBS:
    sh.save_to_file("nrwcbs_{}data_lst_{}".format(outfile_infix, args_to_string(args)), nrwcbs_data_lst)
    nrwcbs_data_lst = sh.read_from_file("nrwcbs_{}data_lst_{}".format(outfile_infix, args_to_string(args)))
if DO_LNS:
    sh.save_to_file("lns_{}data_lst_{}".format(outfile_infix, args_to_string(args)), lns_data_lst)
    lns_data_lst = sh.read_from_file("lns_{}data_lst_{}".format(outfile_infix, args_to_string(args)))


# Ensures data can be reloaded properly


# acbs_data_lst = sh.read_from_file("acbs_{}data_lst_{}".format(outfile_infix, args_to_string(args)))


# afs_data_lst = sh.read_from_file("afs_{}data_lst_{}".format(outfile_infix, args_to_string(args)))
# mstar_data_lst = sh.read_from_file("mstar_{}data_lst_{}".format(outfile_infix, args_to_string(args)))
# pr_data_lst = sh.read_from_file("pra_{}data_lst_{}".format(outfile_infix, args_to_string(args)))


clean_up()