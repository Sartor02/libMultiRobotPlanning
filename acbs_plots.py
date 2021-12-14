import collections
import glob
import matplotlib
import re
from functools import reduce
import numpy as np
import sys

import plot_styling as ps
from matplotlib import pyplot as plt
from shared_helpers import XStarData
from shared_helpers import MStarData
from shared_helpers import ACBSData
from shared_helpers import NWCBSData
from shared_helpers import CBSData
from shared_helpers import AFSData
from shared_helpers import PRData
from shared_helpers import LNSData
from shared_helpers import DCBSData



FileData = collections.namedtuple('FileData', ['filename',
                                               'agents',
                                               'iter',
                                               'trial',
                                               'seed'])
kTimeout = 1200

def filename_to_filedata(l):
    l_orig = l
    l = l.replace("_density_0.1", "")
    l = l.replace("_density_0.05", "")
    l = l.replace("_density_0.01", "")
    regex = r"[a-zA-Z_\/]*(\d\d\d|\d\d|\d)_iter_(\d\d\d|\d\d|\d)_trial_(\d\d\d|\d\d|\d)_seed_(\d\d\d\d\d\d|\d\d\d\d\d|\d\d\d\d|\d\d\d|\d\d|\d).result" # noqa
    matches = list(re.finditer(regex, l, re.MULTILINE))
    match = None
    try:
        match = matches[0]
    except:
        # print("Exception caught:", l)
        return None
    agents = int(match.group(1))
    itr = int(match.group(2))
    trial = int(match.group(3))
    seed = int(match.group(4))
    return FileData(l_orig, agents, itr, trial, seed)


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


def get_total_first_plan_time(filename, timeout=kTimeout):
    ls = open(filename, 'r').readlines()
    if len(ls) == 0:
        return timeout
    time_individual_plan = get_line(ls, "time_individual_plan", float)
    time_first_plan = get_line(ls, "time_first_plan", float)
    return time_individual_plan + time_first_plan


def get_optimal_time_or_timeout(filename, timeout):
    ls = open(filename, 'r').readlines()
    if len(ls) == 0:
        return timeout
    if get_line(ls, "is_optimal", bool):
        return get_line(ls, "Total time", float)
    else:
        return timeout


def get_field(filename, field, t, default):
    ls = open(filename, 'r').readlines()
    if len(ls) == 0:
        return default
    return get_line(ls, field, t)


def get_ci(data, percentile):
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


def get_percentile(data, percentile):
    percentile = percentile / 100.0
    assert(percentile <= 1.0 and percentile > 0)
    data.sort()
    high_idx = int(percentile * (len(data) - 1))
    low_idx = 0
    high = data[high_idx]
    low = data[low_idx]
    return (high, low)


def add_to_dict(acc, e):
    lst = acc.get(e[0], [])
    lst.append(e[1])
    acc[e[0]] = lst
    return acc

def read_from_file(name):  
    f = open("{}".format(name), 'r')
    data = eval(f.read())
    f.close()
    return data

def get_first_runtimes(data):
    return data.runtimes[0]


kBoundsAgentCount = 30

# 1 percent obstacle density
xstar_supp_datas_density_01 = [read_from_file(f) for f in glob.glob('datasave/xstar_supplemental_data_lst_*density0.01timeout{}*'.format(kTimeout))]
xstar_supp_datas_density_01 = [x for lst in xstar_supp_datas_density_01 for x in lst]
xstar_agents_bounds_01 = [x.bounds for x in xstar_supp_datas_density_01 if x.num_agents == kBoundsAgentCount]
xstar_agents_first_times_density_01 = [(x.num_agents, x.runtimes[0]) for x in xstar_supp_datas_density_01]
xstar_agents_optimal_times_density_01 = [(x.num_agents, x.runtimes[-1]) for x in xstar_supp_datas_density_01]

cbs_datas_density_01 = [read_from_file(f) for f in glob.glob('datasave/cbs_supplemental_data_lst_*density0.01timeout{}*'.format(kTimeout))]
cbs_datas_density_01 = [x for lst in cbs_datas_density_01 for x in lst]
cbs_agents_times_density_01 = [(x.num_agents, x.runtimes) for x in cbs_datas_density_01]

nrwcbs_datas_density_01 = [read_from_file(f) for f in glob.glob('datasave/nrwcbs_supplemental_data_lst_*density0.01timeout{}*'.format(kTimeout))]
nrwcbs_datas_density_01 = [x for lst in nrwcbs_datas_density_01 for x in lst]
nrwcbs_agents_bounds_01 = [x.ratios for x in nrwcbs_datas_density_01 if x.num_agents == kBoundsAgentCount]
nrwcbs_agents_first_times_density_01 = [(x.num_agents, x.runtimes[0]) for x in nrwcbs_datas_density_01]
nrwcbs_agents_optimal_times_density_01 = [(x.num_agents, x.runtimes[-1] if x.runtimes[-1] != -1 else x.runtimes[-2]) for x in nrwcbs_datas_density_01]

nwcbs_datas_density_01 = [read_from_file(f) for f in glob.glob('datasave/nwcbs_supplemental_data_lst_*density0.01timeout{}*'.format(kTimeout))]
nwcbs_datas_density_01 = [x for lst in nwcbs_datas_density_01 for x in lst]
nwcbs_agents_bounds_01 = [x.ratios for x in nwcbs_datas_density_01 if x.num_agents == kBoundsAgentCount]
nwcbs_agents_first_times_density_01 = [(x.num_agents, x.runtimes[0]) for x in nwcbs_datas_density_01]
nwcbs_agents_optimal_times_density_01 = [(x.num_agents, x.runtimes[-1] if x.runtimes[-1] != -1 else x.runtimes[-2]) for x in nwcbs_datas_density_01]


# 5 percent obstacle density
xstar_supp_datas_density_05 = [read_from_file(f) for f in glob.glob('datasave/xstar_supplemental_data_lst_*density0.05timeout{}*'.format(kTimeout))]
xstar_supp_datas_density_05 = [x for lst in xstar_supp_datas_density_05 for x in lst]
xstar_agents_bounds_05 = [x.bounds for x in xstar_supp_datas_density_05 if x.num_agents == kBoundsAgentCount]
xstar_agents_first_times_density_05 = [(x.num_agents, x.runtimes[0]) for x in xstar_supp_datas_density_05]
xstar_agents_optimal_times_density_05 = [(x.num_agents, x.runtimes[-1]) for x in xstar_supp_datas_density_05]

cbs_datas_density_05 = [read_from_file(f) for f in glob.glob('datasave/cbs_supplemental_data_lst_*density0.05timeout{}*'.format(kTimeout))]
cbs_datas_density_05 = [x for lst in cbs_datas_density_05 for x in lst]
cbs_agents_times_density_05 = [(x.num_agents, x.runtimes) for x in cbs_datas_density_05]

nrwcbs_datas_density_05 = [read_from_file(f) for f in glob.glob('datasave/nrwcbs_supplemental_data_lst_*density0.05timeout{}*'.format(kTimeout))]
nrwcbs_datas_density_05 = [x for lst in nrwcbs_datas_density_05 for x in lst]
nrwcbs_agents_bounds_05 = [x.ratios for x in nrwcbs_datas_density_05 if x.num_agents == kBoundsAgentCount]
nrwcbs_agents_first_times_density_05 = [(x.num_agents, x.runtimes[0]) for x in nrwcbs_datas_density_05]
nrwcbs_agents_optimal_times_density_05 = [(x.num_agents, x.runtimes[-1] if x.runtimes[-1] != -1 else x.runtimes[-2]) for x in nrwcbs_datas_density_05]

lns_datas_density_05 = [read_from_file(f) for f in glob.glob('datasave/lns_supplemental_data_lst_*density0.05timeout{}*'.format(2))]
lns_datas_density_05 = [x for lst in lns_datas_density_05 for x in lst]
lns_agents_first_times_density_05 = [(x.num_agents, x.runtimes[0]) for x in lns_datas_density_05]

# For Comparing against LNS
nrwcbs2_datas_density_05 = [read_from_file(f) for f in glob.glob('datasave/nrwcbs_supplemental_data_lst_*density0.05timeout{}*'.format(2))]
nrwcbs2_datas_density_05 = [x for lst in nrwcbs2_datas_density_05 for x in lst]
nrwcbs2_agents_first_times_density_05 = [(x.num_agents, x.runtimes[0]) for x in nrwcbs2_datas_density_05]

nwcbs_datas_density_05 = [read_from_file(f) for f in glob.glob('datasave/nwcbs_supplemental_data_lst_*density0.05timeout{}*'.format(kTimeout))]
nwcbs_datas_density_05 = [x for lst in nwcbs_datas_density_05 for x in lst]
nwcbs_agents_bounds_05 = [x.ratios for x in nwcbs_datas_density_05 if x.num_agents == kBoundsAgentCount]
nwcbs_agents_first_times_density_05 = [(x.num_agents, x.runtimes[0]) for x in nwcbs_datas_density_05]
nwcbs_agents_optimal_times_density_05 = [(x.num_agents, x.runtimes[-1] if x.runtimes[-1] != -1 else x.runtimes[-2]) for x in nwcbs_datas_density_05]

# nwcbs_datas_density_05 = [read_from_file(f) for f in glob.glob('datasave/nwcbs_supplemental_data_lst_*density0.05timeout{}*'.format(kTimeout))]
# nwcbs_datas_density_05 = [x for lst in nwcbs_datas_density_05 for x in lst]
# nwcbs_agents_bounds_05 = [x.ratios for x in nwcbs_datas_density_05 if x.num_agents == kBoundsAgentCount]

# fails = 0
# for x, a in zip(xstar_agents_bounds_05, nrwcbs_agents_bounds_05):
#     if 0 in x:
#         fails += 1
#         continue
#     l = len(a)
#     if -1 in a:
#         l -= 1
#     lx = len(x)
#     print(l-lx, end=", ")
# print("fails: " + str(fails))

# fails = 0
# ct = 0
# for bounds in nwcbs_agents_bounds_05:
#     print("Trial {}".format(ct))
#     ct += 1
#     for i in range(1, len(bounds)):
#         if bounds[i] > bounds[i-1]:
#             print(str(bounds[i-1]) + ", " + str(bounds[i]))
#             fails += 1
#     print("\n", end="")
# print("fails: " + str(fails))

# 10 percent obstacle density
xstar_supp_datas_density_1 = [read_from_file(f) for f in glob.glob('datasave/xstar_supplemental_data_lst_*density0.1timeout{}*'.format(kTimeout))]
xstar_supp_datas_density_1 = [x for lst in xstar_supp_datas_density_1 for x in lst]
xstar_agents_bounds_1 = [x.bounds for x in xstar_supp_datas_density_1 if x.num_agents == kBoundsAgentCount]
xstar_agents_first_times_density_1 = [(x.num_agents, x.runtimes[0]) for x in xstar_supp_datas_density_1]
xstar_agents_optimal_times_density_1 = [(x.num_agents, x.runtimes[-1]) for x in xstar_supp_datas_density_1]

cbs_datas_density_1 = [read_from_file(f) for f in glob.glob('datasave/cbs_supplemental_data_lst_*density0.1timeout{}*'.format(kTimeout))]
cbs_datas_density_1 = [x for lst in cbs_datas_density_1 for x in lst]
cbs_datas_density_1 = [x for x in cbs_datas_density_1 if int(x.num_agents) <= 60]
cbs_agents_times_density_1 = [(x.num_agents, x.runtimes) for x in cbs_datas_density_1]

nrwcbs_datas_density_1 = [read_from_file(f) for f in glob.glob('datasave/nrwcbs_supplemental_data_lst_*density0.1timeout{}*'.format(kTimeout))]
nrwcbs_datas_density_1 = [x for lst in nrwcbs_datas_density_1 for x in lst]
nrwcbs_agents_bounds_1 = [x.ratios for x in nrwcbs_datas_density_1 if x.num_agents == kBoundsAgentCount]
nrwcbs_agents_first_times_density_1 = [(x.num_agents, x.runtimes[0]) for x in nrwcbs_datas_density_1]
nrwcbs_agents_optimal_times_density_1 = [(x.num_agents, x.runtimes[-1] if x.runtimes[-1] != -1 else x.runtimes[-2]) for x in nrwcbs_datas_density_1]

nwcbs_datas_density_1 = [read_from_file(f) for f in glob.glob('datasave/nwcbs_supplemental_data_lst_*density0.1timeout{}*'.format(kTimeout))]
nwcbs_datas_density_1 = [x for lst in nwcbs_datas_density_1 for x in lst]
nwcbs_agents_bounds_1 = [x.ratios for x in nwcbs_datas_density_1 if x.num_agents == kBoundsAgentCount]
nwcbs_agents_first_times_density_1 = [(x.num_agents, x.runtimes[0]) for x in nwcbs_datas_density_1]
nwcbs_agents_optimal_times_density_1 = [(x.num_agents, x.runtimes[-1] if x.runtimes[-1] != -1 else x.runtimes[-2]) for x in nwcbs_datas_density_1]

# print(glob.glob('datasave/cbs_supplemental_data_lst_*density0.1timeout{}*'.format(10)))
fp_data_c1 = [read_from_file(f) for f in glob.glob('datasave/cbs_supplemental_data_lst_*density0.01timeout{}*'.format(10))]
fp_data_c1 = [x for lst in fp_data_c1 for x in lst]
fp_data_c1 = [x for x in fp_data_c1 if int(x.num_agents) <= 40]
fp_cbs_times = [(x.num_agents, x.runtimes) for x in fp_data_c1]
fp_cbs_lln = [(x.num_agents, x.llnex) for x in fp_data_c1]
fp_cbs_hlln = [(x.num_agents, x.llnex / x.hlnex) for x in fp_data_c1]

fp_data_d1 = [read_from_file(f) for f in glob.glob('datasave/dcbs_supplemental_data_lst_*density0.01timeout{}*'.format(10))]
fp_data_d1 = [x for lst in fp_data_d1 for x in lst]
fp_data_d1 = [x for x in fp_data_d1 if int(x.num_agents) <= 40]
fp_dcbs_times = [(x.num_agents, x.runtimes) for x in fp_data_d1]
fp_dcbs_lln = [(x.num_agents, x.llnex) for x in fp_data_d1]
fp_dcbs_hlln = [(x.num_agents, x.llnex / x.hlnex) for x in fp_data_d1]

kRadiusTimeout = 300

def draw_timeout(timeout, xs, plt=plt):
    linestyle = (0, (1, 3))
    if type(timeout) is int:
        plt.axhline(timeout, color='black', lw=0.7, linestyle=linestyle)
    elif type(timeout) is dict:
        ys = [timeout[x] for x in xs]
        plt.plot(xs, ys, linestyle=linestyle,
                 color='black')

def draw_timeout_data(timeout, data_lst, plt=plt):
    data_lst.sort()
    agents_to_times_dict = reduce(add_to_dict, data_lst, dict())
    xs = sorted(agents_to_times_dict.keys())
    draw_timeout(timeout, xs, plt=plt)

def plt_bw(agents_times_lst, name, plt_idx, max_idx, show_y_axis, pos_idx, num_pos=2, position_offset = 0.2, plot_width = 0.15):
    agents_times_lst.sort()
    agents_to_times_dict = reduce(add_to_dict, agents_times_lst, dict())
    
    current_offset = -(num_pos + 1) / 2 * position_offset + position_offset * (pos_idx + 1)
    
    xs = sorted(agents_to_times_dict.keys())
    positions = [e + current_offset for e in range(1, len(xs) + 1)]
    values = [agents_to_times_dict[k] for k in xs]
    color = ps.color(plt_idx, max_idx)
    colort = ps.alpha(color, 0.5)
    bplot = plt.boxplot(values, 
        patch_artist=True, 
        whis=1.0,
        boxprops=dict(facecolor=colort, color=colort),
        flierprops=dict(marker='.', markersize=1, color=colort, markeredgecolor=color),
        capprops=dict(color=color),
        whiskerprops=dict(color=colort),
        medianprops=dict(color=color),
        widths=plot_width,
        positions=positions
        # 
        )
    label_string="{}".format(name)

    ps.add_legend(bplot["boxes"][0], label_string)

    plt.yscale('log')
    if show_y_axis:
        plt.ylabel("Time (seconds)")
    plt.xlabel("Number of agents")
    plt.xticks([e + 1 for e in range(len(xs))], xs)

def plt_bounds(bounds_data, show_y_axis, planner_name,  acbs=False):
    num_iters = 85

    for i in range(len(bounds_data)):
        if acbs and bounds_data[i][-1] == -1:
            if len(bounds_data[i]) == 1:
                bounds_data[i][0] = 0
            else:
                bounds_data[i] = bounds_data[i][:-1]
                # if bounds_data[i][-1] != 1:
                #     print(bounds_data[i][-1 ])
        # elif acbs and bounds_data[i][-1] != 1:
        #     print(bounds_data[i])

        if bounds_data[i][-2:] == [1, 1]:
            bounds_data[i] = bounds_data[i][:-1]
        if (len(bounds_data[i]) < num_iters):
            bounds_data[i] += ([bounds_data[i][-1]] * (num_iters - len(bounds_data[i])))

    steps_bounds = {}

    no_result = 0
    for run in bounds_data:
        if 0 in run:
            no_result += 1
            continue

        for idx, bound in enumerate(run):
            existing = steps_bounds.get(idx, [])
            existing.append(bound)
            steps_bounds[idx] = existing

    print("Failures:", no_result)

    plot_width = 0.3

    steps_list = [k for k in sorted(steps_bounds.keys())[:num_iters]]
    bounds_list = [steps_bounds[k] for k in steps_list]

    color = ps.color(0, 4)
    colort = ps.alpha(color, 0.5)

    bplot = plt.boxplot(bounds_list, 
        patch_artist=True, 
        whis=1.0,
        boxprops=dict(facecolor=colort, color=colort),
        flierprops=dict(marker='.', markersize=1, color=colort, markeredgecolor=color),
        capprops=dict(color=color),
        whiskerprops=dict(color=colort),
        medianprops=dict(color=color),
        widths=plot_width)

    label_string = "{}".format(planner_name)
    ps.add_legend(bplot["boxes"][0], label_string)

    plt.gca().set_ylim(bottom=0.999)
    plt.gca().set_ylim(top=1.0375)
    plt.xticks(   [1] + [i for i in range(5, num_iters+5, 5)]   , [1] + [i for i in range(5, num_iters+5, 5)])
    if show_y_axis:
        plt.ylabel("$\epsilon$-suboptimality Bound")
    plt.xlabel("{} Iterations".format(planner_name))

# PLOTS

min_time = 1 / 200000
max_time = 2000
PLOT_AGENTS_DENSITY = 0
PLOT_BOUNDS = 0
PLOT_SANDBOX = 0
PLOT_DCBS = 0

if PLOT_DCBS:
    ps.setupfig(quartersize=True)
    draw_timeout_data(10, xstar_agents_first_times_density_01)
    plt_bw(fp_cbs_times, "CBS", 0, 4, False, 0)
    plt_bw(fp_dcbs_times, "LRCBS", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("lrcbs_times")

if PLOT_AGENTS_DENSITY:
    # ALL FIRST SOLUTIONS


    # XSTAR AND ACBS FIRST SOLUTION
    ps.setupfig(quartersize=True)
    draw_timeout_data(kTimeout, xstar_agents_first_times_density_01)
    plt_bw(xstar_agents_first_times_density_01, "X* Valid", 0, 4, False, 0)
    plt_bw(nrwcbs_agents_first_times_density_01, "ACBS Valid", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("xstar_acbs_first_times_density_01_bw")

    ps.setupfig(quartersize=True)
    draw_timeout_data(kTimeout, xstar_agents_first_times_density_05)
    plt_bw(xstar_agents_first_times_density_05, "X* Valid", 0, 4, False, 0)
    plt_bw(nrwcbs_agents_first_times_density_05, "ACBS Valid", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("xstar_acbs_first_times_density_05_bw")

    ps.setupfig(quartersize=True)
    draw_timeout_data(kTimeout, xstar_agents_first_times_density_1)
    plt_bw(xstar_agents_first_times_density_1, "X* Valid", 0, 4, False, 0)
    plt_bw(nrwcbs_agents_first_times_density_1, "ACBS Valid", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("xstar_acbs_first_times_density_1_bw")


    # XSTAR AND ACBS OPTIMAL
    ps.setupfig(quartersize=True)
    draw_timeout_data(kTimeout, xstar_agents_optimal_times_density_01)
    plt_bw(xstar_agents_optimal_times_density_01, "X* Opt", 0, 4, False, 0)
    plt_bw(nrwcbs_agents_optimal_times_density_01, "ACBS Opt", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("xstar_acbs_optimal_times_density_01_bw")

    ps.setupfig(quartersize=True)
    draw_timeout_data(kTimeout, xstar_agents_optimal_times_density_05)
    plt_bw(xstar_agents_optimal_times_density_05, "X* Opt", 0, 4, False, 0)
    plt_bw(nrwcbs_agents_optimal_times_density_05, "ACBS Opt", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("xstar_acbs_optimal_times_density_05_bw")

    draw_timeout_data(kTimeout, xstar_agents_optimal_times_density_1)
    plt_bw(xstar_agents_optimal_times_density_1, "X* Opt", 0, 4, False, 0)
    plt_bw(nrwcbs_agents_optimal_times_density_1, "ACBS Opt", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("xstar_acbs_optimal_times_density_1_bw")


    # CBS AND ACBS FIRST SOLUTION
    ps.setupfig(quartersize=True)
    draw_timeout_data(kTimeout, cbs_agents_times_density_01)
    plt_bw(cbs_agents_times_density_01, "CBS Valid/Opt.", 0, 4, False, 0)
    plt_bw(nrwcbs_agents_first_times_density_01, "ACBS Valid", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("cbs_acbs_first_times_density_01_bw")

    ps.setupfig(quartersize=True)
    draw_timeout_data(kTimeout, cbs_agents_times_density_05)
    plt_bw(cbs_agents_times_density_05, "CBS Valid/Opt.", 0, 4, False, 0)
    plt_bw(nrwcbs_agents_first_times_density_05, "ACBS Valid", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("cbs_acbs_first_times_density_05_bw")

    ps.setupfig(quartersize=True)
    draw_timeout_data(kTimeout, cbs_agents_times_density_1)
    plt_bw(cbs_agents_times_density_1, "CBS Valid/Opt.", 0, 4, False, 0)
    plt_bw(nrwcbs_agents_first_times_density_1, "ACBS Valid", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("cbs_acbs_first_times_density_1_bw")


    # CBS AND ACBS OPTIMAL
    ps.setupfig(quartersize=True)
    draw_timeout_data(kTimeout, cbs_agents_times_density_01)
    plt_bw(cbs_agents_times_density_01, "CBS Opt", 0, 4, False, 0)
    plt_bw(nrwcbs_agents_optimal_times_density_01, "ACBS Opt", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("cbs_acbs_optimal_times_density_01_bw")

    ps.setupfig(quartersize=True)
    draw_timeout_data(kTimeout, cbs_agents_times_density_05)
    plt_bw(cbs_agents_times_density_05, "CBS Opt", 0, 4, False, 0)
    plt_bw(nrwcbs_agents_optimal_times_density_05, "ACBS Opt", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("cbs_acbs_optimal_times_density_05_bw")

    ps.setupfig(quartersize=True)
    draw_timeout_data(kTimeout, cbs_agents_times_density_1)
    plt_bw(cbs_agents_times_density_1, "CBS Opt", 0, 4, False, 0)
    plt_bw(nrwcbs_agents_optimal_times_density_1, "ACBS Opt", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("cbs_acbs_optimal_times_density_1_bw")


    # NRWCBS AND NWCBS ABLATION OPTIMAL
    ps.setupfig(quartersize=True)
    draw_timeout_data(kTimeout, nwcbs_agents_optimal_times_density_01)
    plt_bw(nwcbs_agents_optimal_times_density_01, "NWCBS Opt", 0, 4, False, 0)
    plt_bw(nrwcbs_agents_optimal_times_density_01, "ACBS Opt", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("nwcbs_acbs_optimal_times_density_01_bw")

    ps.setupfig(quartersize=True)
    draw_timeout_data(kTimeout, nwcbs_agents_optimal_times_density_05)
    plt_bw(nwcbs_agents_optimal_times_density_05, "NWCBS Opt", 0, 4, False, 0)
    plt_bw(nrwcbs_agents_optimal_times_density_05, "ACBS Opt", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("nwcbs_acbs_optimal_times_density_05_bw")

    ps.setupfig(quartersize=True)
    draw_timeout_data(kTimeout, nwcbs_agents_optimal_times_density_1)
    plt_bw(nwcbs_agents_optimal_times_density_1, "NWCBS Opt", 0, 4, False, 0)
    plt_bw(nrwcbs_agents_optimal_times_density_1, "ACBS Opt", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("nwcbs_acbs_optimal_times_density_1_bw")

    ps.setupfig(quartersize=True)
    draw_timeout_data(kTimeout, nwcbs_agents_first_times_density_01)
    plt_bw(nwcbs_agents_first_times_density_01, "NWCBS Valid", 0, 4, False, 0)
    plt_bw(nrwcbs_agents_first_times_density_01, "ACBS Valid", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("nwcbs_acbs_first_times_density_01_bw")

    
    # NWCBS and NRWCBS ABLATION FIRST SOLUTION
    ps.setupfig(quartersize=True)
    draw_timeout_data(kTimeout, nwcbs_agents_first_times_density_05)
    plt_bw(nwcbs_agents_first_times_density_05, "NWCBS Valid", 0, 4, False, 0)
    plt_bw(nrwcbs_agents_first_times_density_05, "ACBS Valid", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("nwcbs_acbs_first_times_density_05_bw")

    ps.setupfig(quartersize=True)
    draw_timeout_data(kTimeout, nwcbs_agents_first_times_density_1)
    plt_bw(nwcbs_agents_first_times_density_1, "NWCBS Valid", 0, 4, False, 0)
    plt_bw(nrwcbs_agents_first_times_density_1, "ACBS Valid", 2, 4, False, 1)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("nwcbs_acbs_first_times_density_1_bw")

if PLOT_BOUNDS:
    # SUBOPTIMALITY BOUNDS XSTAR
    ps.setupfig(quartersize=True)
    plt_bounds(xstar_agents_bounds_01, True, "X*")
    ps.grid()
    ps.save_fig("xstar_bounds_01")

    ps.setupfig(quartersize=True)
    plt_bounds(xstar_agents_bounds_05, False, "X*")
    ps.grid()
    ps.save_fig("xstar_bounds_05")

    ps.setupfig(quartersize=True)
    plt_bounds(xstar_agents_bounds_1, False, "X*")
    ps.grid()
    ps.save_fig("xstar_bounds_1")

    # SUBOPTIMALITY BOUNDS ACBS
    ps.setupfig(quartersize=True)
    plt_bounds(nrwcbs_agents_bounds_01, True, "ACBS", True)
    ps.grid()
    ps.save_fig("acbs_bounds_01")

    ps.setupfig(quartersize=True)
    plt_bounds(nrwcbs_agents_bounds_05, False, "ACBS", True)
    ps.grid()
    ps.save_fig("acbs_bounds_05")

    ps.setupfig(quartersize=True)
    plt_bounds(nrwcbs_agents_bounds_1, False, "ACBS", True)
    ps.grid()
    ps.save_fig("acbs_bounds_1")

if PLOT_SANDBOX:
    # SUBOPTIMALITY BOUNDS NWCBS
    # ps.setupfig(quartersize=True)
    # plt_bounds(nwcbs_agents_bounds_01, True, "NWCBS", True)
    # ps.grid()
    # ps.save_fig("nwcbs_bounds_01")

    # ps.setupfig(quartersize=True)
    # plt_bw(lns_agents_first_times_density_05, "LNS Valid", 0, 4, False, 0)
    # plt_bw(nrwcbs2_agents_first_times_density_05, "ACBS Valid", 2, 4, False, 1)
    # ps.grid()
    # ps.legend('br')
    # plt.ylim(min_time, 10)
    # ps.save_fig("lns_acbs_first_times_density_05_bw")

    ps.setupfig(halfsize=True)
    draw_timeout_data(kTimeout, xstar_agents_first_times_density_05)
    plt_bw(nrwcbs_agents_first_times_density_05, "ACBS Valid", 0, 5, False, 0, num_pos=5, position_offset=0.12, plot_width=0.06)
    plt_bw(nwcbs_agents_first_times_density_05, "NWCBS Valid", 1, 5, False, 1, num_pos=5, position_offset=0.12, plot_width=0.06)
    plt_bw(lns_agents_first_times_density_05, "LNS Valid", 2, 5, False, 2, num_pos=5, position_offset=0.12, plot_width=0.06)
    plt_bw(xstar_agents_first_times_density_05, "X* Valid", 3, 5, False, 3, num_pos=5, position_offset=0.12, plot_width=0.06)
    plt_bw(cbs_agents_times_density_05, "CBS Valid/Opt.", 4, 5, False, 4, num_pos=5, position_offset=0.12, plot_width=0.06)
    ps.grid()
    ps.legend('br')
    plt.ylim(min_time, max_time)
    ps.save_fig("all_first_times_density_05_bw")