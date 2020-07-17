#!/usr/bin/env python3
import collections
import glob
import matplotlib
import re
from functools import reduce
import numpy as np

import plot_styling as ps
from matplotlib import pyplot as plt
from shared_helpers import XStarData
from shared_helpers import MStarData
from shared_helpers import CBSData
from shared_helpers import AFSData
from shared_helpers import PRData



FileData = collections.namedtuple('FileData', ['filename',
                                               'agents',
                                               'iter',
                                               'trial',
                                               'seed'])
kTimeout = 1200

def show_notification(text):
    try:
        Notify.Notification.new("\n     {}     \n".format(text)).show()
    except:
        return

try:
    from gi.repository import Notify
    Notify.init("X* Plots")
    show_notification("Starting plot generation")
except:
    print("Notification loading failed.")

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


idv_file_datas = \
    [filename_to_filedata(f) for f in glob.glob('datasave/xstar_compare*density_0.*.result') if filename_to_filedata(f) is not None]
assert(len(idv_file_datas) > 0)
idv_file_datas = [e for e in idv_file_datas if e.agents <= 60]
idv_file_datas = [e for e in idv_file_datas if e.agents != 40 or e.seed < 5720]

agents_first_times_lst = \
    sorted([(d.agents, get_total_first_plan_time(d.filename))
            for d in idv_file_datas])
agents_optimal_times_lst = \
    sorted([(d.agents, get_optimal_time_or_timeout(d.filename, kTimeout))
            for d in idv_file_datas])
num_agents_in_window_optimal_times_lst = \
    sorted([(get_field(d.filename, "num_max_agents_in_window", int, None),
             get_optimal_time_or_timeout(d.filename, kTimeout)) for d in idv_file_datas
            if get_field(d.filename, "num_max_agents_in_window", int, None)
            is not None])
num_agents_in_window_first_times_lst = \
    sorted([(get_field(d.filename, "num_max_agents_in_window_first_iteration", int, None),
            get_total_first_plan_time(d.filename)) for d in idv_file_datas
            if get_field(d.filename, "num_max_agents_in_window_first_iteration", int, None) is not None])
num_agents_in_window_optimal_times_01_lst = \
    sorted([(get_field(d.filename, "num_max_agents_in_window", int, None),
             get_optimal_time_or_timeout(d.filename, kTimeout)) for d in idv_file_datas
            if get_field(d.filename, "num_max_agents_in_window", int, None)
            is not None and "density_0.01" in d.filename])
num_agents_in_window_first_times_01_lst = \
    sorted([(get_field(d.filename, "num_max_agents_in_window_first_iteration", int, None),
            get_total_first_plan_time(d.filename)) for d in idv_file_datas
            if get_field(d.filename, "num_max_agents_in_window_first_iteration", int, None) 
            is not None and "density_0.01" in d.filename])
num_agents_in_window_optimal_times_05_lst = \
    sorted([(get_field(d.filename, "num_max_agents_in_window", int, None),
             get_optimal_time_or_timeout(d.filename, kTimeout)) for d in idv_file_datas
            if get_field(d.filename, "num_max_agents_in_window", int, None)
            is not None and "density_0.05" in d.filename])
num_agents_in_window_first_times_05_lst = \
    sorted([(get_field(d.filename, "num_max_agents_in_window_first_iteration", int, None),
            get_total_first_plan_time(d.filename)) for d in idv_file_datas
            if get_field(d.filename, "num_max_agents_in_window_first_iteration", int, None) 
            is not None and "density_0.05" in d.filename])
num_agents_in_window_optimal_times_1_lst = \
    sorted([(get_field(d.filename, "num_max_agents_in_window", int, None),
             get_optimal_time_or_timeout(d.filename, kTimeout)) for d in idv_file_datas
            if get_field(d.filename, "num_max_agents_in_window", int, None)
            is not None and "density_0.1" in d.filename])
num_agents_in_window_first_times_1_lst = \
    sorted([(get_field(d.filename, "num_max_agents_in_window_first_iteration", int, None),
            get_total_first_plan_time(d.filename)) for d in idv_file_datas
            if get_field(d.filename, "num_max_agents_in_window_first_iteration", int, None) 
            is not None and "density_0.1" in d.filename])


def read_from_file(name):  
    f = open("{}".format(name), 'r')
    data = eval(f.read())
    f.close()
    return data


def get_first_runtimes(data):
    return data.runtimes[0]

kBoundsAgentCount = 30

xstar_datas_density_01 = [read_from_file(f) for f in glob.glob('datasave/xstar_data_lst_*density0.01*')]
xstar_datas_density_01 = [x for lst in xstar_datas_density_01 for x in lst]
xstar_agents_bounds_01 = [x.bounds for x in xstar_datas_density_01 if x.num_agents == kBoundsAgentCount]
xstar_agents_first_times_density_01 = [(x.num_agents, x.runtimes[0]) for x in xstar_datas_density_01]
xstar_agents_optimal_times_density_01 = [(x.num_agents, x.runtimes[-1]) for x in xstar_datas_density_01]

xstar_supp_datas_density_01 = [read_from_file(f) for f in glob.glob('datasave/xstar_supplemental_data_lst_*density0.01*')]
xstar_supp_datas_density_01 = [x for lst in xstar_supp_datas_density_01 for x in lst]
xstar_supp_agents_bounds_01 = [x.bounds for x in xstar_supp_datas_density_01 if x.num_agents == kBoundsAgentCount]
xstar_supp_agents_first_times_density_01 = [(x.num_agents, x.runtimes[0]) for x in xstar_supp_datas_density_01]

cbs_datas_density_01 = [read_from_file(f) for f in glob.glob('datasave/cbs_data_lst_*density0.01*')]
cbs_datas_density_01 = [x for lst in cbs_datas_density_01 for x in lst]
cbs_agents_times_density_01 = [(x.num_agents, x.runtimes) for x in cbs_datas_density_01]

afs_datas_density_01 = [read_from_file(f) for f in glob.glob('datasave/afs_data_lst_*density0.01*')]
afs_datas_density_01 = [x for lst in afs_datas_density_01 for x in lst]
afs_agents_bounds_01 = [x.bounds for x in afs_datas_density_01 if x.num_agents == kBoundsAgentCount]
afs_agents_first_times_density_01 = [(x.num_agents, x.runtimes[0]) for x in afs_datas_density_01]
afs_agents_optimal_times_density_01 = [(x.num_agents, x.runtimes[-1]) for x in afs_datas_density_01]

mstar_datas_density_01 = [read_from_file(f) for f in glob.glob('datasave/mstar_data_lst_*density0.01*')]
mstar_datas_density_01 = [x for lst in mstar_datas_density_01 for x in lst]
mstar_agents_times_density_01 = [(x.num_agents, x.runtimes) for x in mstar_datas_density_01]

pr_supp_datas_density_01 = [read_from_file(f) for f in glob.glob('datasave/pr_supplemental_data_lst_*density0.01*')]
pr_supp_datas_density_01 = [x for lst in pr_supp_datas_density_01 for x in lst]
pr_supp_agents_bounds_01 = [x.bounds for x in pr_supp_datas_density_01 if x.num_agents == kBoundsAgentCount]
pr_supp_agents_first_times_density_01 = [(x.num_agents, x.runtimes) for x in pr_supp_datas_density_01]

xstar_datas_density_05 = [read_from_file(f) for f in glob.glob('datasave/xstar_data_lst_*density0.05*')]
xstar_datas_density_05 = [x for lst in xstar_datas_density_05 for x in lst]
xstar_agents_bounds_05 = [x.bounds for x in xstar_datas_density_05  if x.num_agents == kBoundsAgentCount]
xstar_agents_first_times_density_05 = [(x.num_agents, x.runtimes[0]) for x in xstar_datas_density_05]
xstar_agents_optimal_times_density_05 = [(x.num_agents, x.runtimes[-1]) for x in xstar_datas_density_05]

xstar_supp_datas_density_05 = [read_from_file(f) for f in glob.glob('datasave/xstar_supplemental_data_lst_*density0.05*')]
xstar_supp_datas_density_05 = [x for lst in xstar_supp_datas_density_05 for x in lst]
xstar_supp_agents_bounds_05 = [x.bounds for x in xstar_supp_datas_density_05 if x.num_agents == kBoundsAgentCount]
xstar_supp_agents_first_times_density_05 = [(x.num_agents, x.runtimes[0]) for x in xstar_supp_datas_density_05]

cbs_datas_density_05 = [read_from_file(f) for f in glob.glob('datasave/cbs_data_lst_*density0.05*')]
cbs_datas_density_05 = [x for lst in cbs_datas_density_05 for x in lst]
cbs_agents_times_density_05 = [(x.num_agents, x.runtimes) for x in cbs_datas_density_05]

afs_datas_density_05 = [read_from_file(f) for f in glob.glob('datasave/afs_data_lst_*density0.05*')]
afs_datas_density_05 = [x for lst in afs_datas_density_05 for x in lst]
afs_agents_bounds_05 = [x.bounds for x in afs_datas_density_05 if x.num_agents == kBoundsAgentCount]
afs_agents_first_times_density_05 = [(x.num_agents, x.runtimes[0]) for x in afs_datas_density_05]
afs_agents_optimal_times_density_05 = [(x.num_agents, x.runtimes[-1]) for x in afs_datas_density_05]

mstar_datas_density_05 = [read_from_file(f) for f in glob.glob('datasave/mstar_data_lst_*density0.05*')]
mstar_datas_density_05 = [x for lst in mstar_datas_density_05 for x in lst]
mstar_agents_times_density_05 = [(x.num_agents, x.runtimes) for x in mstar_datas_density_05]

pr_supp_datas_density_05 = [read_from_file(f) for f in glob.glob('datasave/pr_supplemental_data_lst_*density0.05*')]
pr_supp_datas_density_05 = [x for lst in pr_supp_datas_density_05 for x in lst]
pr_supp_agents_bounds_05 = [x.bounds for x in pr_supp_datas_density_05 if x.num_agents == kBoundsAgentCount]
pr_supp_agents_first_times_density_05 = [(x.num_agents, x.runtimes) for x in pr_supp_datas_density_05]

xstar_datas_density_1 = [read_from_file(f) for f in glob.glob('datasave/xstar_data_lst_*density0.1*')]
xstar_datas_density_1 = [x for lst in xstar_datas_density_1 for x in lst]
xstar_datas_density_1 = [x for x in xstar_datas_density_1 if int(x.num_agents) <= 60]
xstar_agents_bounds_1 = [x.bounds for x in xstar_datas_density_1 if x.num_agents == kBoundsAgentCount]
xstar_agents_first_times_density_1 = [(x.num_agents, x.runtimes[0]) for x in xstar_datas_density_1]
xstar_agents_optimal_times_density_1 = [(x.num_agents, x.runtimes[-1]) for x in xstar_datas_density_1]

xstar_supp_datas_density_1 = [read_from_file(f) for f in glob.glob('datasave/xstar_supplemental_data_lst_*density0.1*')]
xstar_supp_datas_density_1 = [x for lst in xstar_supp_datas_density_1 for x in lst]
xstar_supp_agents_bounds_1 = [x.bounds for x in xstar_supp_datas_density_1 if x.num_agents == kBoundsAgentCount]
xstar_supp_agents_first_times_density_1 = [(x.num_agents, x.runtimes[0]) for x in xstar_supp_datas_density_1]

cbs_datas_density_1 = [read_from_file(f) for f in glob.glob('datasave/cbs_data_lst_*density0.1*')]
cbs_datas_density_1 = [x for lst in cbs_datas_density_1 for x in lst]
cbs_datas_density_1 = [x for x in cbs_datas_density_1 if int(x.num_agents) <= 60]
cbs_agents_times_density_1 = [(x.num_agents, x.runtimes) for x in cbs_datas_density_1]

afs_datas_density_1 = [read_from_file(f) for f in glob.glob('datasave/afs_data_lst_*density0.1*')]
afs_datas_density_1 = [x for lst in afs_datas_density_1 for x in lst]
afs_datas_density_1 = [x for x in afs_datas_density_1 if int(x.num_agents) <= 60]
afs_agents_bounds_1 = [x.bounds for x in afs_datas_density_1 if x.num_agents == kBoundsAgentCount]
afs_agents_first_times_density_1 = [(x.num_agents, x.runtimes[0]) for x in afs_datas_density_1]
afs_agents_optimal_times_density_1 = [(x.num_agents, x.runtimes[-1]) for x in afs_datas_density_1]

mstar_datas_density_1 = [read_from_file(f) for f in glob.glob('datasave/mstar_data_lst_*density0.1*')]
mstar_datas_density_1 = [x for lst in mstar_datas_density_1 for x in lst]
mstar_datas_density_1 = [x for x in mstar_datas_density_1 if int(x.num_agents) <= 60]
mstar_agents_times_density_1 = [(x.num_agents, x.runtimes) for x in mstar_datas_density_1]

pr_supp_datas_density_1 = [read_from_file(f) for f in glob.glob('datasave/pr_supplemental_data_lst_*density0.1*')]
pr_supp_datas_density_1 = [x for lst in pr_supp_datas_density_1 for x in lst]
pr_supp_agents_bounds_1 = [x.bounds for x in pr_supp_datas_density_1 if x.num_agents == kBoundsAgentCount]
pr_supp_agents_first_times_density_1 = [(x.num_agents, x.runtimes) for x in pr_supp_datas_density_1]

constant_density_agents = [20, 40, 80, 160, 320]

xstar_datas_density_const = [read_from_file(f) for f in glob.glob('datasave/xstar_data_lst_*density0.1*')]
xstar_datas_density_const = [x for lst in xstar_datas_density_const for x in lst]
xstar_datas_density_const = [x for x in xstar_datas_density_const if int(x.num_agents) in constant_density_agents]
xstar_agents_first_times_density_const = [(x.num_agents, x.runtimes[0]) for x in xstar_datas_density_const]
xstar_agents_optimal_times_density_const = [(x.num_agents, x.runtimes[-1]) for x in xstar_datas_density_const]

cbs_datas_density_const = [read_from_file(f) for f in glob.glob('datasave/cbs_data_lst_*density0.1*')]
cbs_datas_density_const = [x for lst in cbs_datas_density_const for x in lst]
cbs_datas_density_const = [x for x in cbs_datas_density_const if int(x.num_agents) in constant_density_agents]
cbs_agents_times_density_const = [(x.num_agents, x.runtimes) for x in cbs_datas_density_const]

afs_datas_density_const = [read_from_file(f) for f in glob.glob('datasave/afs_data_lst_*density0.1*')]
afs_datas_density_const = [x for lst in afs_datas_density_const for x in lst]
afs_datas_density_const = [x for x in afs_datas_density_const if int(x.num_agents) in constant_density_agents]
afs_agents_first_times_density_const = [(x.num_agents, x.runtimes[0]) for x in afs_datas_density_const]
afs_agents_optimal_times_density_const = [(x.num_agents, x.runtimes[-1]) for x in afs_datas_density_const]

mstar_datas_density_const = [read_from_file(f) for f in glob.glob('datasave/mstar_data_lst_*density0.1*')]
mstar_datas_density_const = [x for lst in mstar_datas_density_const for x in lst]
mstar_datas_density_const = [x for x in mstar_datas_density_const if int(x.num_agents) in constant_density_agents]
mstar_agents_times_density_const = [(x.num_agents, x.runtimes) for x in mstar_datas_density_const]

kRadiusTimeout = 300

def get_radius(filename):
    fr = filename.find("_xstar")+ 6;
    return int(filename[fr: fr+1])

def get_radius_first_runtime(filename):
    try:
        return float([l for l in open(filename, 'r').readlines() if "time_first_plan" in l][0].replace("time_first_plan: ", ""))
    except:
        return kRadiusTimeout

def get_radius_total_runtime(filename):
    try:
        return float([l for l in open(filename, 'r').readlines() if "Total time" in l][0].replace("Total time: ", ""))
    except:
        return kRadiusTimeout

xstar_datas_scale_radius = [(get_radius(f), get_radius_first_runtime(f), get_radius_total_runtime(f)) for f in glob.glob('datasave/xstar_grow_search_window*')]
xstar_radius_first_times = [(r, f) for r, f, t in xstar_datas_scale_radius]
xstar_radius_optimal_times = [(r, t) for r, f, t in xstar_datas_scale_radius]


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


def plt_cis(agents_times_lst, title, timeout=None):
    agents_times_lst.sort()
    agents_to_times_dict = reduce(add_to_dict, agents_times_lst, dict())
    agents_to_100_bounds_lst = \
        [[k] + list(get_ci(v, 100)) for k, v in agents_to_times_dict.items()]
    agents_to_95_bounds_lst = \
        [[k] + list(get_ci(v, 95)) for k, v in agents_to_times_dict.items()]
    agents_to_90_bounds_lst = \
        [[k] + list(get_ci(v, 90)) for k, v in agents_to_times_dict.items()]
    agents_to_75_bounds_lst = \
        [[k] + list(get_ci(v, 75)) for k, v in agents_to_times_dict.items()]
    medians = []
    xs, hs, ms, ls = zip(*agents_to_100_bounds_lst)
    medians = list(ms)
    plt.plot(xs, ls, color=ps.color(0, 4), label="Max bounds")
    plt.plot(xs, hs, color=ps.color(0, 4))
    plt.fill_between(xs, ls, hs,
                     where=ls <= hs,
                     facecolor=ps.alpha(ps.color(0, 4)),
                     interpolate=True,
                     linewidth=0.0)
    xs, hs, ms, ls = zip(*agents_to_95_bounds_lst)
    plt.plot(xs, ls, color=ps.color(1, 4), label="95% CI")
    plt.plot(xs, hs, color=ps.color(1, 4))
    plt.fill_between(xs, ls, hs,
                     where=ls <= hs,
                     facecolor=ps.alpha(ps.color(1, 4)),
                     interpolate=True,
                     linewidth=0.0)
    xs, hs, ms, ls = zip(*agents_to_90_bounds_lst)
    plt.plot(xs, ls, color=ps.color(2, 4), label="90% CI")
    plt.plot(xs, hs, color=ps.color(2, 4))
    plt.fill_between(xs, ls, hs,
                     where=ls <= hs,
                     facecolor=ps.alpha(ps.color(2, 4)),
                     interpolate=True,
                     linewidth=0.0)
    xs, hs, ms, ls = zip(*agents_to_75_bounds_lst)
    plt.plot(xs, ls, color=ps.color(3, 4), label="75% CI")
    plt.plot(xs, ms, color=ps.color(3, 4))
    plt.plot(xs, hs, color=ps.color(3, 4))
    plt.fill_between(xs, ls, hs,
                     where=ls <= hs,
                     facecolor=ps.alpha(ps.color(3, 4)),
                     interpolate=True,
                     linewidth=0.0)
    plt.yscale('log')
    plt.ylabel("Time (seconds)")
    plt.xlabel("Number of agents")
    plt.xticks(xs)

    draw_timeout(timeout, xs)


def plt_95_ci(agents_times_lst, name, plt_idx, max_idx, timeout=None, show_y_axis=True):
    agents_times_lst.sort()
    agents_to_times_dict = reduce(add_to_dict, agents_times_lst, dict())

    agents_to_95_bounds_lst = \
        [[k] + list(get_ci(v, 95)) for k, v in agents_to_times_dict.items()]
    medians = []
    xs, hs, ms, ls = zip(*agents_to_95_bounds_lst)
    plt.plot(xs, ls, color=ps.color(plt_idx, max_idx), linestyle='--')
    plt.plot(xs, hs, color=ps.color(plt_idx, max_idx), linestyle='--')
    plt.plot(xs, ms, color=ps.color(plt_idx, max_idx), label="{} 95% CI".format(name))
    plt.fill_between(xs, ls, hs,
                     where=ls <= hs,
                     facecolor=ps.alpha(ps.color(plt_idx, max_idx), 0.2),
                     interpolate=True,
                     linewidth=0.0)
    plt.yscale('log')
    if show_y_axis:
        plt.ylabel("Time (seconds)")
    plt.xlabel("Number of agents")
    plt.xticks(xs)

    draw_timeout(timeout, xs)


def plt_bw(agents_times_lst, name, plt_idx, max_idx, timeout, show_y_axis, pos_idx, num_pos=2, position_offset = 0.2, plot_width = 0.15):
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
    label_string="{} Performance".format(name)

    handles, labels = plt.gca().get_legend_handles_labels()
    ps.add_legend(bplot["boxes"][0], label_string)

    plt.yscale('log')
    if show_y_axis:
        plt.ylabel("Time (seconds)")
    plt.xlabel("Number of agents")
    plt.xticks([e + 1 for e in range(len(xs))], xs)


def plt_percentiles(agents_times_lst, title, timeout=None):
    agents_to_times_dict = reduce(add_to_dict, agents_times_lst, dict())
    agents_to_100_bounds_lst = \
        [[k] + list(get_percentile(v, 100))
         for k, v in agents_to_times_dict.items()]
    agents_to_95_bounds_lst = \
        [[k] + list(get_percentile(v, 95))
         for k, v in agents_to_times_dict.items()]
    agents_to_90_bounds_lst = \
        [[k] + list(get_percentile(v, 90))
         for k, v in agents_to_times_dict.items()]
    agents_to_75_bounds_lst = \
        [[k] + list(get_percentile(v, 75))
         for k, v in agents_to_times_dict.items()]
    agents_to_50_bounds_lst = \
        [[k] + list(get_percentile(v, 50))
         for k, v in agents_to_times_dict.items()]
    xs, hs, ls = zip(*agents_to_100_bounds_lst)
    plt.plot(xs, hs, color=ps.color(0, 5), label="Max bounds")
    plt.fill_between(xs, ls, hs,
                     where=ls <= hs,
                     facecolor=ps.alpha(ps.color(0, 5)),
                     interpolate=True,
                     linewidth=0.0)
    xs, hs, ls = zip(*agents_to_95_bounds_lst)
    plt.plot(xs, hs, color=ps.color(1, 5), label="95th percentile")
    plt.fill_between(xs, ls, hs,
                     where=ls <= hs,
                     facecolor=ps.alpha(ps.color(1, 5)),
                     interpolate=True,
                     linewidth=0.0)
    xs, hs, ls = zip(*agents_to_90_bounds_lst)
    plt.plot(xs, hs, color=ps.color(2, 5), label="90th percentile")
    plt.fill_between(xs, ls, hs,
                     where=ls <= hs,
                     facecolor=ps.alpha(ps.color(2, 5)),
                     interpolate=True,
                     linewidth=0.0)
    xs, hs, ls = zip(*agents_to_75_bounds_lst)
    plt.plot(xs, hs, color=ps.color(3, 5), label="75th percentile")
    plt.fill_between(xs, ls, hs,
                     where=ls <= hs,
                     facecolor=ps.alpha(ps.color(3, 5)),
                     interpolate=True,
                     linewidth=0.0)
    xs, hs, ls = zip(*agents_to_50_bounds_lst)
    plt.plot(xs, ls, color=ps.color(4, 5), label="50th percentile")
    plt.plot(xs, hs, color=ps.color(4, 5))
    plt.fill_between(xs, ls, hs,
                     where=ls <= hs,
                     facecolor=ps.alpha(ps.color(4, 5)),
                     interpolate=True,
                     linewidth=0.0)
    plt.yscale('log')
    plt.ylabel("Time (seconds)")
    plt.xlabel("Number of agents")
    plt.xticks(xs)

    draw_timeout(timeout, xs)


def plt_bounds(bounds_data, show_y_axis, planner_name):
    for i in range(len(bounds_data)):
        if bounds_data[i][-2:] == [1, 1]:
            bounds_data[i] = bounds_data[i][:-1]
        if (len(bounds_data[i]) < 20):
            bounds_data[i] += ([bounds_data[i][-1]] * (20 - len(bounds_data[i])))

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

    steps_list = [k for k in sorted(steps_bounds.keys())[:20]]
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
    plt.xticks([1,5,10,15, 20], [1,5,10,15, 20])
    if show_y_axis:
        plt.ylabel("$\epsilon$-suboptimality Bound")
    plt.xlabel("{} Iterations".format(planner_name))


# ########################
# # PR vs X* Performance #
# ########################

print("PR vs X* Plotting")

ps.setupfig(thirdsize=True)
draw_timeout_data(60, xstar_supp_agents_first_times_density_01)
plt_bw(xstar_supp_agents_first_times_density_01, "X* Valid", 0, 4, 60, True, 0)
plt_bw(pr_supp_agents_first_times_density_01, "PR Valid", 1, 4, 60, True, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_pr_first_times_density_01_bw")

ps.setupfig(thirdsize=True)
draw_timeout_data(60, xstar_supp_agents_first_times_density_05)
plt_bw(xstar_supp_agents_first_times_density_05, "X* Valid", 0, 4, 60, False, 0)
plt_bw(pr_supp_agents_first_times_density_05, "PR Valid", 1, 4, 60, False, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_pr_first_times_density_05_bw")

ps.setupfig(thirdsize=True)
draw_timeout_data(60, xstar_supp_agents_first_times_density_1)
plt_bw(xstar_supp_agents_first_times_density_1, "X* Valid", 0, 4, 60, False, 0)
plt_bw(pr_supp_agents_first_times_density_1, "PR Valid", 1, 4, 60, False, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_pr_first_times_density_1_bw")

###################
# Bounds Plotting #
###################

print("Bounds Plotting")

ps.setupfig(thirdsize=True)
plt_bounds(xstar_agents_bounds_01, True, "X*")
ps.grid()
# ps.legend('br')
ps.save_fig("xstar_bounds_01")

ps.setupfig(thirdsize=True)
plt_bounds(xstar_agents_bounds_05, False, "X*")
ps.grid()
# ps.legend('br')
ps.save_fig("xstar_bounds_05")

ps.setupfig(thirdsize=True)
plt_bounds(xstar_agents_bounds_1, False, "X*")
ps.grid()
# ps.legend('br')
ps.save_fig("xstar_bounds_1")

############################
# Head to head comparisons #
############################
print("Head to head comparisons")

# ps.setupfig()
# plt_95_ci(xstar_agents_optimal_times_density_01, "X* Optimal", 0, 4, 1200)
# plt_95_ci(afs_agents_optimal_times_density_01, "AFS Optimal", 1, 4, 1200)
# plt_95_ci(xstar_agents_first_times_density_01, "X* Valid", 2, 4, 1200)
# plt_95_ci(afs_agents_first_times_density_01, "AFS Valid", 3, 4, 1200)
# ps.grid()
# ps.legend('br')
# ps.save_fig("xstar_afs_first_optimal_times_density_01")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_first_times_density_01, "X* Valid", 0, 4, 1200)
plt_95_ci(afs_agents_first_times_density_01, "AFS Valid", 1, 4, 1200)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_afs_first_times_density_01")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_first_times_density_01)
plt_bw(xstar_agents_first_times_density_01, "X* Valid", 0, 4, 1200, True, 0)
plt_bw(afs_agents_first_times_density_01, "AFS Valid", 1, 4, 1200, True, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_afs_first_times_density_01_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_optimal_times_density_01, "X* Optimal", 0, 4, 1200)
plt_95_ci(afs_agents_optimal_times_density_01, "AFS Optimal", 1, 4, 1200)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_afs_optimal_times_density_01")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_optimal_times_density_01)
plt_bw(xstar_agents_optimal_times_density_01, "X* Optimal", 0, 4, 1200, True, 0)
plt_bw(afs_agents_optimal_times_density_01, "AFS Optimal", 1, 4, 1200, True, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_afs_optimal_times_density_01_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_first_times_density_01, "X* Valid", 0, 4, 1200, False)
plt_95_ci(cbs_agents_times_density_01, "CBS Valid/Opt.", 2, 4, 1200, False)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_cbs_first_times_density_01")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_first_times_density_01)
plt_bw(xstar_agents_first_times_density_01, "X* Valid", 0, 4, 1200, False, 0)
plt_bw(cbs_agents_times_density_01, "CBS Valid/Opt.", 2, 4, 1200, False, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_cbs_first_times_density_01_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_optimal_times_density_01, "X* Optimal", 0, 4, 1200, False)
plt_95_ci(cbs_agents_times_density_01, "CBS Optimal", 2, 4, 1200, False)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_cbs_optimal_times_density_01")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_optimal_times_density_01)
plt_bw(xstar_agents_optimal_times_density_01, "X* Optimal", 0, 4, 1200, False, 0)
plt_bw(cbs_agents_times_density_01, "CBS Optimal", 2, 4, 1200, False, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_cbs_optimal_times_density_01_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_first_times_density_01, "X* Valid", 0, 4, 1200, False)
plt_95_ci(mstar_agents_times_density_01, "M* Valid/Opt.", 3, 4, 1200, False)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_mstar_first_times_density_01")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_first_times_density_01)
plt_bw(xstar_agents_first_times_density_01, "X* Valid", 0, 4, 1200, False, 0)
plt_bw(mstar_agents_times_density_01, "M* Valid/Opt.", 3, 4, 1200, False, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_mstar_first_times_density_01_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_optimal_times_density_01, "X* Optimal", 0, 4, 1200, False)
plt_95_ci(mstar_agents_times_density_01, "M* Optimal", 3, 4, 1200, False)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_mstar_optimal_times_density_01")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_optimal_times_density_01)
plt_bw(xstar_agents_optimal_times_density_01, "X* Optimal", 0, 4, 1200, False, 0)
plt_bw(mstar_agents_times_density_01, "M* Optimal", 3, 4, 1200, False, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_mstar_optimal_times_density_01_bw")

# ps.setupfig(halfsize=True)
# plt_95_ci(xstar_agents_first_times_density_01, "X* Valid", 0, 4, 1200)
# plt_95_ci(afs_agents_first_times_density_01, "AFS Valid", 1, 4, 1200)
# plt_95_ci(cbs_agents_times_density_01, "CBS Valid/Opt.", 2, 4, 1200)
# plt_95_ci(mstar_agents_times_density_01, "M* Valid/Opt.", 3, 4, 1200)
# ps.grid()
# ps.legend('br')
# ps.save_fig("xstar_vs_all_first_times_density_01")

# ps.setupfig(halfsize=True)
# plt_95_ci(xstar_agents_optimal_times_density_01, "X* Optimal", 0, 4, 1200)
# plt_95_ci(afs_agents_optimal_times_density_01, "AFS Optimal", 1, 4, 1200)
# plt_95_ci(cbs_agents_times_density_01, "CBS Optimal", 2, 4, 1200)
# plt_95_ci(mstar_agents_times_density_01, "M* Optimal", 3, 4, 1200)
# ps.grid()
# ps.legend('br')
# ps.save_fig("xstar_vs_all_optimal_times_density_01")

# ======================================

# ps.setupfig(halfsize=True)
# plt_95_ci(xstar_agents_optimal_times_density_05, "X* Optimal", 0, 4, 1200)
# plt_95_ci(afs_agents_optimal_times_density_05, "AFS Optimal", 1, 4, 1200)
# plt_95_ci(xstar_agents_first_times_density_05, "X* Valid", 2, 4, 1200)
# plt_95_ci(afs_agents_first_times_density_05, "AFS Valid", 3, 4, 1200)
# ps.grid()
# ps.legend('br')
# ps.save_fig("xstar_afs_first_optimal_times_density_05")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_first_times_density_05, "X* Valid", 0, 4, 1200)
plt_95_ci(afs_agents_first_times_density_05, "AFS Valid", 1, 4, 1200)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_afs_first_times_density_05")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_first_times_density_05)
plt_bw(xstar_agents_first_times_density_05, "X* Valid", 0, 4, 1200, True, 0)
plt_bw(afs_agents_first_times_density_05, "AFS Valid", 1, 4, 1200, True, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_afs_first_times_density_05_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_optimal_times_density_05, "X* Optimal", 0, 4, 1200)
plt_95_ci(afs_agents_optimal_times_density_05, "AFS Optimal", 1, 4, 1200)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_afs_optimal_times_density_05")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_optimal_times_density_05)
plt_bw(xstar_agents_optimal_times_density_05, "X* Optimal", 0, 4, 1200, True, 0)
plt_bw(afs_agents_optimal_times_density_05, "AFS Optimal", 1, 4, 1200, True, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_afs_optimal_times_density_05_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_first_times_density_05, "X* Valid", 0, 4, 1200, False)
plt_95_ci(cbs_agents_times_density_05, "CBS Valid/Opt.", 2, 4, 1200, False)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_cbs_first_times_density_05")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_first_times_density_05)
plt_bw(xstar_agents_first_times_density_05, "X* Valid", 0, 4, 1200, False, 0)
plt_bw(cbs_agents_times_density_05, "CBS Valid/Opt.", 2, 4, 1200, False, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_cbs_first_times_density_05_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_optimal_times_density_05, "X* Optimal", 0, 4, 1200, False)
plt_95_ci(cbs_agents_times_density_05, "CBS Optimal", 2, 4, 1200, False)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_cbs_optimal_times_density_05")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_optimal_times_density_05)
plt_bw(xstar_agents_optimal_times_density_05, "X* Optimal", 0, 4, 1200, False, 0)
plt_bw(cbs_agents_times_density_05, "CBS Optimal", 2, 4, 1200, False, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_cbs_optimal_times_density_05_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_first_times_density_05, "X* Valid", 0, 4, 1200, False)
plt_95_ci(mstar_agents_times_density_05, "M* Valid/Opt.", 3, 4, 1200, False)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_mstar_first_times_density_05")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_first_times_density_05)
plt_bw(xstar_agents_first_times_density_05, "X* Valid", 0, 4, 1200, False, 0)
plt_bw(mstar_agents_times_density_05, "M* Valid/Opt.", 3, 4, 1200, False, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_mstar_first_times_density_05_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_optimal_times_density_05, "X* Optimal", 0, 4, 1200, False)
plt_95_ci(mstar_agents_times_density_05, "M* Optimal", 3, 4, 1200, False)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_mstar_optimal_times_density_05")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_optimal_times_density_05)
plt_bw(xstar_agents_optimal_times_density_05, "X* Optimal", 0, 4, 1200, False, 0)
plt_bw(mstar_agents_times_density_05, "M* Optimal", 3, 4, 1200, False, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_mstar_optimal_times_density_05_bw")

# ps.setupfig(halfsize=True)
# plt_95_ci(xstar_agents_first_times_density_05, "X* Valid", 0, 4, 1200)
# plt_95_ci(afs_agents_first_times_density_05, "AFS Valid", 1, 4, 1200)
# plt_95_ci(cbs_agents_times_density_05, "CBS Valid/Opt.", 2, 4, 1200)
# plt_95_ci(mstar_agents_times_density_05, "M* Valid/Opt.", 3, 4, 1200)
# ps.grid()
# ps.legend('br')
# ps.save_fig("xstar_vs_all_first_times_density_05")

# ps.setupfig(halfsize=True)
# plt_95_ci(xstar_agents_optimal_times_density_05, "X* Optimal", 0, 4, 1200)
# plt_95_ci(afs_agents_optimal_times_density_05, "AFS Optimal", 1, 4, 1200)
# plt_95_ci(cbs_agents_times_density_05, "CBS Optimal", 2, 4, 1200)
# plt_95_ci(mstar_agents_times_density_05, "M* Optimal", 3, 4, 1200)
# ps.grid()
# ps.legend('br')
# ps.save_fig("xstar_vs_all_optimal_times_density_05")

# ======================================

# ps.setupfig(halfsize=True)
# plt_95_ci(xstar_agents_optimal_times_density_1, "X* Optimal", 0, 4, 1200)
# plt_95_ci(afs_agents_optimal_times_density_1, "AFS Optimal", 1, 4, 1200)
# plt_95_ci(xstar_agents_first_times_density_1, "X* Valid", 2, 4, 1200)
# plt_95_ci(afs_agents_first_times_density_1, "AFS Valid", 3, 4, 1200)
# ps.grid()
# ps.legend('br')
# ps.save_fig("xstar_afs_first_optimal_times_density_1")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_first_times_density_1, "X* Valid", 0, 4, 1200)
plt_95_ci(afs_agents_first_times_density_1, "AFS Valid", 1, 4, 1200)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_afs_first_times_density_1")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_first_times_density_1)
plt_bw(xstar_agents_first_times_density_1, "X* Valid", 0, 4, 1200, True, 0)
plt_bw(afs_agents_first_times_density_1, "AFS Valid", 1, 4, 1200, True, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_afs_first_times_density_1_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_optimal_times_density_1, "X* Optimal", 0, 4, 1200)
plt_95_ci(afs_agents_optimal_times_density_1, "AFS Optimal", 1, 4, 1200)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_afs_optimal_times_density_1")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_optimal_times_density_1)
plt_bw(xstar_agents_optimal_times_density_1, "X* Optimal", 0, 4, 1200, True, 0)
plt_bw(afs_agents_optimal_times_density_1, "AFS Optimal", 1, 4, 1200, True, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_afs_optimal_times_density_1_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_first_times_density_1, "X* Valid", 0, 4, 1200, False)
plt_95_ci(cbs_agents_times_density_1, "CBS Valid/Opt.", 2, 4, 1200, False)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_cbs_first_times_density_1")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_first_times_density_1)
plt_bw(xstar_agents_first_times_density_1, "X* Valid", 0, 4, 1200, False, 0)
plt_bw(cbs_agents_times_density_1, "CBS Valid/Opt.", 2, 4, 1200, False, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_cbs_first_times_density_1_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_optimal_times_density_1, "X* Optimal", 0, 4, 1200, False)
plt_95_ci(cbs_agents_times_density_1, "CBS Optimal", 2, 4, 1200, False)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_cbs_optimal_times_density_1")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_optimal_times_density_1)
plt_bw(xstar_agents_optimal_times_density_1, "X* Optimal", 0, 4, 1200, False, 0)
plt_bw(cbs_agents_times_density_1, "CBS Optimal", 2, 4, 1200, False, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_cbs_optimal_times_density_1_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_first_times_density_1, "X* Valid", 0, 4, 1200, False)
plt_95_ci(mstar_agents_times_density_1, "M* Valid/Opt.", 3, 4, 1200, False)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_mstar_first_times_density_1")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_first_times_density_1)
plt_bw(xstar_agents_first_times_density_1, "X* Valid", 0, 4, 1200, False, 0)
plt_bw(mstar_agents_times_density_1, "M* Valid/Opt.", 3, 4, 1200, False, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_mstar_first_times_density_1_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_optimal_times_density_1, "X* Optimal", 0, 4, 1200, False)
plt_95_ci(mstar_agents_times_density_1, "M* Optimal", 3, 4, 1200, False)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_mstar_optimal_times_density_1")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_optimal_times_density_1)
plt_bw(xstar_agents_optimal_times_density_1, "X* Optimal", 0, 4, 1200, False, 0)
plt_bw(mstar_agents_times_density_1, "M* Optimal", 3, 4, 1200, False, 1)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_mstar_optimal_times_density_1_bw")

# ps.setupfig(halfsize=True)
# plt_95_ci(xstar_agents_first_times_density_1, "X* Valid", 0, 4, 1200)
# plt_95_ci(afs_agents_first_times_density_1, "AFS Valid", 1, 4, 1200)
# plt_95_ci(cbs_agents_times_density_1, "CBS Valid/Opt.", 2, 4, 1200)
# plt_95_ci(mstar_agents_times_density_1, "M* Valid/Opt.", 3, 4, 1200)
# ps.grid()
# ps.legend('br')
# ps.save_fig("xstar_vs_all_first_times_density_1")

# ps.setupfig(halfsize=True)
# plt_95_ci(xstar_agents_optimal_times_density_1, "X* Optimal", 0, 4, 1200)
# plt_95_ci(afs_agents_optimal_times_density_1, "AFS Optimal", 1, 4, 1200)
# plt_95_ci(cbs_agents_times_density_1, "CBS Optimal", 2, 4, 1200)
# plt_95_ci(mstar_agents_times_density_1, "M* Optimal", 3, 4, 1200)
# ps.grid()
# ps.legend('br')
# ps.save_fig("xstar_vs_all_optimal_times_density_1")


###########################################
# X* Only Agents vs Performance Breakdown #
###########################################
print("X* Only Agents vs Performance Breakdown")

ps.setupfig()
plt_cis(agents_first_times_lst, "Time to Valid solution", kTimeout)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_first_solution_ci")

ps.setupfig()
plt_cis(agents_optimal_times_lst, "Time to optimal solution", kTimeout)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_optimal_solution_ci")

ps.setupfig()
plt_percentiles(agents_first_times_lst, "Time to Valid solution", kTimeout)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_first_solution_percentile")

ps.setupfig()
plt_percentiles(agents_optimal_times_lst, "Time to optimal solution", kTimeout)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_optimal_solution_percentile")


def plt_window_agents_boxplot(num_agents_in_window_times_lst, title, pos_idx, num_pos, timeout=None, set_y_label=True, min_y=None, max_y=None, position_offset = 0.2, plot_width = 0.15, legend_label=None):
    # Build {agent : runtimes} dict
    num_agents_in_window_to_optimal_times_dict = \
        reduce(add_to_dict, num_agents_in_window_times_lst, dict())
    # Remove keys less than 2
    num_agents_in_window_to_optimal_times_dict = \
        {k: v for k, v in num_agents_in_window_to_optimal_times_dict.items()}
    # Fill in any missing count in the range with empty list
    max_key = max(num_agents_in_window_to_optimal_times_dict.keys())
    min_key = min(num_agents_in_window_to_optimal_times_dict.keys())
    # num_agents_in_window_to_optimal_times_dict = \
    #     {e: num_agents_in_window_to_optimal_times_dict.get(e, []) for e in
    #      range(min_key, max_key + 1)}
    # Convert to list of (k, list(v)) pairs sorted by k
    keys_values = \
        [(k, sorted(v)) for k, v in
         sorted(num_agents_in_window_to_optimal_times_dict.items(),
                key=lambda kv: kv[0])]
    ks, vs = zip(*keys_values)
    color = ps.color(pos_idx, 4)
    colort = ps.alpha(color, 0.5)

    
    current_offset = -((num_pos - 1) / 2) * position_offset + position_offset * pos_idx
    positions = [e + current_offset for e in range(1, len(ks) + 1)]

    bplot = plt.boxplot(vs, 
        patch_artist=True, 
        whis=1.0,
        boxprops=dict(facecolor=colort, color=colort),
        flierprops=dict(marker='.', markersize=1, color=colort, markeredgecolor=color),
        capprops=dict(color=color),
        whiskerprops=dict(color=colort),
        medianprops=dict(color=color),        
        widths=0.15,
        positions=positions)

    if legend_label is not None:
        label_string=legend_label
        ps.add_legend(bplot["boxes"][0], label_string)    

    draw_timeout(timeout, None)
    plt.xticks([e + 1 for e in range(len(ks))], ks)
    plt.gca().set_xticklabels(ks)
    plt.xlabel("Largest Number of Agents\nIn Any Window (LNAIAW)")
    if set_y_label:
        plt.ylabel("Time (seconds)")
    if min_y is not None:
        plt.ylim(bottom=min_y)
    if max_y is not None:
        plt.ylim(top=max_y)


def plt_window_agents_hist(num_agents_in_window_times_lst, title, pos_idx=0, num_pos=1, plt=plt, draw_y_label=True, xlabel="Largest Number of Agents\nIn Any Window (LNAIAW)",  position_offset = 0.2, plot_width = 0.15, ymax = None):
    # Build {agent : runtimes} dict
    num_agents_in_window_to_optimal_times_dict = \
        reduce(add_to_dict, num_agents_in_window_times_lst, dict())
    # Remove keys less than 2
    num_agents_in_window_to_optimal_times_dict = \
        {k: v for k, v in num_agents_in_window_to_optimal_times_dict.items()}
    assert(not 1 in num_agents_in_window_to_optimal_times_dict.keys())
    # Fill in any missing count in the range with empty list
    max_key = max(num_agents_in_window_to_optimal_times_dict.keys())
    min_key = min(num_agents_in_window_to_optimal_times_dict.keys())
    # num_agents_in_window_to_optimal_times_dict = \
    #     {e: num_agents_in_window_to_optimal_times_dict.get(e, []) for e in
    #      range(min_key, max_key + 1)}
    # Convert to list of (k, list(v)) pairs sorted by k
    k_vs = \
        [(k, len(v)) for k, v in
         num_agents_in_window_to_optimal_times_dict.items()]
    ks, vs = zip(*k_vs)
    current_offset = -((num_pos - 1) / 2) * position_offset + position_offset * pos_idx
    positions = [e + current_offset for e in range(1, len(ks) + 1)]
    centers = [e for e in range(1, len(ks) + 1)]

    plt.bar(positions, vs, width=plot_width, color=ps.alpha(ps.color(pos_idx, 4), 0.75))
    if plt is matplotlib.pyplot:
        plt.xticks(centers, ks)
        plt.xlabel(xlabel)
        if ymax is not None:
            plt.ylim(1, ymax)
        if draw_y_label:
            plt.ylabel("Occurrences")
    else:
        plt.set_xticks(centers, ks)
        plt.set_xlabel(xlabel)
        if ymax is not None:
            plt.set_ylim(1, ymax)
        if draw_y_label:
            plt.set_ylabel("Occurrences")

######################################################
# Histogram and boxplot of window dimensions vs time #
######################################################
print("Histogram and boxplot of window dimensions vs time")

min_time_boxplot = min([t for a, t in num_agents_in_window_first_times_lst])
max_time_boxplot = max([t for a, t in num_agents_in_window_optimal_times_lst]) * 2
max_count_hist = max([len(v) for v in reduce(add_to_dict, num_agents_in_window_first_times_01_lst, dict()).values()]) * 1.1
ps.setupfig(quartersize=True)
plt_window_agents_boxplot(num_agents_in_window_first_times_lst,
                          "Largest number of agents in window vs time to "
                          "first solution", 0, 1, kTimeout, min_y=min_time_boxplot,
                          max_y=max_time_boxplot)
plt.yscale('log')
ps.grid()
ps.save_fig("window_vs_time_to_first_boxplot")

ps.setupfig(quartersize=True)
plt_window_agents_hist(num_agents_in_window_first_times_lst,
                       "Largest number of agents in window vs occurrences in "
                       "first solution")
plt.yscale('log')
ps.grid()
ps.save_fig("window_vs_time_to_first_hist")

ps.setupfig(quartersize=True)
plt_window_agents_boxplot(num_agents_in_window_optimal_times_lst,
                          "Largest number of agents in any window vs time to "
                          "optimal solution", 0, 1, kTimeout, min_y=min_time_boxplot,
                          max_y=max_time_boxplot)
plt.yscale('log')
ps.grid()
ps.save_fig("window_vs_time_to_optimal_boxplot")

ps.setupfig(quartersize=True)
plt_window_agents_hist(num_agents_in_window_optimal_times_lst,
                          "Largest number of agents in window vs occurrences in "
                          "optimal solution")
plt.yscale('log')
ps.grid()
ps.save_fig("window_vs_time_to_optimal_hist")

ps.setupfig(halfsize=True)
plt_window_agents_boxplot(num_agents_in_window_first_times_01_lst,
                          "Largest number of agents in window vs time to "
                          "first solution", 0, 3, timeout=kTimeout, min_y=min_time_boxplot,
                          max_y=max_time_boxplot, legend_label="1% Obstacles")
plt_window_agents_boxplot(num_agents_in_window_first_times_05_lst,
                          "Largest number of agents in window vs time to "
                          "first solution", 1, 3, min_y=min_time_boxplot,
                          max_y=max_time_boxplot, legend_label="5% Obstacles")
plt_window_agents_boxplot(num_agents_in_window_first_times_1_lst,
                          "Largest number of agents in window vs time to "
                          "first solution", 2, 3, min_y=min_time_boxplot,
                          max_y=max_time_boxplot, legend_label="10% Obstacles", 
                          timeout=60)
ps.legend('br')
plt.yscale('log')
ps.grid()
ps.save_fig("window_vs_time_to_first_multi_boxplot")

ps.setupfig(halfsize=True)
plt_window_agents_hist(num_agents_in_window_first_times_01_lst,
                          "Largest number of agents in window vs occurrences in "
                          "first solution", 0, 3, ymax=max_count_hist)
plt_window_agents_hist(num_agents_in_window_first_times_05_lst,
                          "Largest number of agents in window vs occurrences in "
                          "first solution", 1, 3, ymax=max_count_hist)
plt_window_agents_hist(num_agents_in_window_first_times_1_lst,
                          "Largest number of agents in window vs occurrences in "
                          "first solution", 2, 3, ymax=max_count_hist)
# plt.yscale('log')
ps.grid()
ps.save_fig("window_vs_time_to_first_multi_hist")

ps.setupfig(halfsize=True)
plt_window_agents_boxplot(num_agents_in_window_optimal_times_01_lst,
                          "Largest number of agents in window vs time to "
                          "optimal solution", 0, 3, timeout=kTimeout, min_y=min_time_boxplot,
                          max_y=max_time_boxplot, legend_label="1% Obstacles")
plt_window_agents_boxplot(num_agents_in_window_optimal_times_05_lst,
                          "Largest number of agents in window vs time to "
                          "optimal solution", 1, 3, min_y=min_time_boxplot,
                          max_y=max_time_boxplot, legend_label="5% Obstacles")
plt_window_agents_boxplot(num_agents_in_window_optimal_times_1_lst,
                          "Largest number of agents in window vs time to "
                          "optimal solution", 2, 3, min_y=min_time_boxplot,
                          max_y=max_time_boxplot, legend_label="10% Obstacles",
                          timeout=60)
ps.legend('br')
plt.yscale('log')
ps.grid()
ps.save_fig("window_vs_time_to_optimal_multi_boxplot")

ps.setupfig(halfsize=True)
plt_window_agents_hist(num_agents_in_window_optimal_times_01_lst,
                          "Largest number of agents in window vs occurrences in "
                          "optimal solution", 0, 3, ymax=max_count_hist)
plt_window_agents_hist(num_agents_in_window_optimal_times_05_lst,
                          "Largest number of agents in window vs occurrences in "
                          "optimal solution", 1, 3, ymax=max_count_hist)
plt_window_agents_hist(num_agents_in_window_optimal_times_1_lst,
                          "Largest number of agents in window vs occurrences in "
                          "optimal solution", 2, 3, ymax=max_count_hist)
# plt.yscale('log')
ps.grid()
ps.save_fig("window_vs_time_to_optimal_multi_hist")

f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
ps.setupfig(f)
ps.grid(ax1)
plt_window_agents_hist(num_agents_in_window_first_times_lst,
                       "Largest number of agents in window vs occurrences in "
                       "first solution", plt=ax1, xlabel="Largest number of agents in window\nfor Valid solution")
ax1.set_yscale('log')
ps.grid(ax2)

plt_window_agents_hist(num_agents_in_window_optimal_times_lst,
                       "Largest number of agents in window vs occurrences in "
                       "optimal solution", plt=ax2, draw_y_label=False, xlabel="Largest number of agents in window\nfor optimal solution")
ps.save_fig("window_vs_time_both_hist")


def plt_radius_vs_runtimes(data, printylabel, plt=plt, timeout=kRadiusTimeout):
    data_dict = reduce(add_to_dict, data, dict())
    for k in data_dict:
        data_dict[k].sort()
    radius_to_100_bounds_lst = \
        [[k] + list(get_ci(v, 100)) for k, v in sorted(data_dict.items())]
    radius_to_95_bounds_lst = \
        [[k] + list(get_ci(v, 95)) for k, v in sorted(data_dict.items())]
    radius_to_90_bounds_lst = \
        [[k] + list(get_ci(v, 90)) for k, v in sorted(data_dict.items())]
    radius_to_75_bounds_lst = \
        [[k] + list(get_ci(v, 75)) for k, v in sorted(data_dict.items())]
    xs, hs, ms, ls = zip(*radius_to_100_bounds_lst)
    plt.plot(xs, ls, color=ps.color(0, 4), label="Max bounds")
    plt.plot(xs, hs, color=ps.color(0, 4))
    plt.fill_between(xs, ls, hs,
                     where=ls <= hs,
                     facecolor=ps.alpha(ps.color(0, 4)),
                     interpolate=True,
                     linewidth=0.0)
    xs, hs, ms, ls = zip(*radius_to_95_bounds_lst)
    plt.plot(xs, ls, color=ps.color(1, 4), label="95% CI")
    plt.plot(xs, hs, color=ps.color(1, 4))
    plt.fill_between(xs, ls, hs,
                     where=ls <= hs,
                     facecolor=ps.alpha(ps.color(1, 4)),
                     interpolate=True,
                     linewidth=0.0)
    xs, hs, ms, ls = zip(*radius_to_90_bounds_lst)
    plt.plot(xs, ls, color=ps.color(2, 4), label="90% CI")
    plt.plot(xs, hs, color=ps.color(2, 4))
    plt.fill_between(xs, ls, hs,
                     where=ls <= hs,
                     facecolor=ps.alpha(ps.color(2, 4)),
                     interpolate=True,
                     linewidth=0.0)
    xs, hs, ms, ls = zip(*radius_to_75_bounds_lst)
    plt.plot(xs, ls, color=ps.color(3, 4), label="75% CI")
    plt.plot(xs, ms, color=ps.color(3, 4))
    plt.plot(xs, hs, color=ps.color(3, 4))
    plt.fill_between(xs, ls, hs,
                     where=ls <= hs,
                     facecolor=ps.alpha(ps.color(3, 4)),
                     interpolate=True,
                     linewidth=0.0)
    if plt is matplotlib.pyplot:
        plt.yscale('log')
        if printylabel:
            plt.ylabel("Time (seconds)")
        plt.xlabel("Initial window $L_{\infty}$ radius")
        plt.xticks(xs)
    else:
        plt.set_yscale('log')
        if printylabel:
            plt.set_ylabel("Time (seconds)")
        plt.set_xlabel("Initial window $L_{\infty}$ radius")
        plt.set_xticks(xs)

    draw_timeout(timeout, xs, plt)

def plt_radius_vs_runtimes_bw(data, printylabel, plt=plt, timeout=kRadiusTimeout):
    data_dict = reduce(add_to_dict, data, dict())
    for k in data_dict:
        data_dict[k].sort()
    xs = list(sorted(data_dict.keys()))
    values = [data_dict[k] for k in xs]

    plot_width = 0.15
    color = ps.color(0, 4)
    colort = ps.alpha(color, 0.5)
    bplot = plt.boxplot(values, 
        patch_artist=True, 
        whis=1.0,
        boxprops=dict(facecolor=colort, color=colort),
        flierprops=dict(marker='.', markersize=1, color=colort, markeredgecolor=color),
        capprops=dict(color=color),
        whiskerprops=dict(color=colort),
        medianprops=dict(color=color),
        widths=plot_width)

    handles, labels = plt.gca().get_legend_handles_labels()

    if plt is matplotlib.pyplot:
        plt.yscale('log')
        if printylabel:
            plt.ylabel("Time (seconds)")
        plt.xlabel("Initial window $L_{\infty}$ radius")
        plt.xticks(xs)
    else:
        plt.set_yscale('log')
        if printylabel:
            plt.set_ylabel("Time (seconds)")
        plt.set_xlabel("Initial window $L_{\infty}$ radius")
        plt.set_xticks(xs)
    

######################
# Density vs runtime #
######################
print("Density vs runtime")

ps.setupfig()
plt_95_ci(xstar_agents_optimal_times_density_const, "X* Optimal", 0, 4, 1200)
plt_95_ci(afs_agents_optimal_times_density_const, "AFS Optimal", 1, 4, 1200)
plt_95_ci(xstar_agents_first_times_density_const, "X* Valid", 2, 4, 1200)
plt_95_ci(afs_agents_first_times_density_const, "AFS Valid", 3, 4, 1200)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_afs_first_optimal_times_density_const")

ps.setupfig(halfsize=True)
plt_95_ci(afs_agents_first_times_density_const, "AFS Valid", 0, 4, 1200)
plt_95_ci(afs_agents_optimal_times_density_const, "AFS Optimal", 1, 4, 1200)
plt_95_ci(xstar_agents_first_times_density_const, "X* Valid", 2, 4, 1200)
plt_95_ci(xstar_agents_optimal_times_density_const, "X* Optimal", 3, 4, 1200)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_afs_first_optimal_times_density_const_half")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_first_times_density_const)
plt_bw(xstar_agents_first_times_density_const, "X* Valid", 0, 4, 1200, True, 0, 2)
plt_bw(afs_agents_first_times_density_const, "AFS Valid", 1, 4, 1200, True, 1, 2)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_afs_first_times_density_const_half_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_first_times_density_const, "X* Valid", 0, 4, 1200)
plt_95_ci(afs_agents_first_times_density_const, "AFS Valid", 1, 4, 1200)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_afs_first_times_density_const_half")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_optimal_times_density_const)
plt_bw(xstar_agents_optimal_times_density_const, "X* Optimal", 0, 4, 1200, True, 0, 2)
plt_bw(afs_agents_optimal_times_density_const, "AFS Optimal", 1, 4, 1200, True, 1, 2)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_afs_optimal_times_density_const_half_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_optimal_times_density_const, "X* Optimal", 0, 4, 1200)
plt_95_ci(afs_agents_optimal_times_density_const, "AFS Optimal", 1, 4, 1200)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_afs_optimal_times_density_const_half")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_first_times_density_const)
plt_bw(xstar_agents_first_times_density_const, "X* Valid", 0, 4, 1200, False, 0, 2)
plt_bw(cbs_agents_times_density_const, "CBS Valid/Opt.", 2, 4, 1200, False, 1, 2)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_cbs_first_times_density_const_half_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_first_times_density_const, "X* Valid", 0, 4, 1200, False)
plt_95_ci(cbs_agents_times_density_const, "CBS Valid/Opt.", 2, 4, 1200, False)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_cbs_first_times_density_const_half")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_first_times_density_const)
plt_bw(xstar_agents_optimal_times_density_const, "X* Optimal", 0, 4, 1200, False, 0, 2)
plt_bw(cbs_agents_times_density_const, "CBS Optimal", 2, 4, 1200, False, 1, 2)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_cbs_optimal_times_density_const_half_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_optimal_times_density_const, "X* Optimal", 0, 4, 1200, False)
plt_95_ci(cbs_agents_times_density_const, "CBS Optimal", 2, 4, 1200, False)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_cbs_optimal_times_density_const_half")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_first_times_density_const)
plt_bw(xstar_agents_first_times_density_const, "X* Valid", 0, 4, 1200, False, 0, 2)
plt_bw(mstar_agents_times_density_const, "M* Valid/Opt.", 3, 4, 1200, False, 1, 2)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_mstar_first_times_density_const_half_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_first_times_density_const, "X* Valid", 0, 4, 1200, False)
plt_95_ci(mstar_agents_times_density_const, "M* Valid/Opt.", 3, 4, 1200, False)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_mstar_first_times_density_const_half")

ps.setupfig(thirdsize=True)
draw_timeout_data(1200, xstar_agents_first_times_density_const)
plt_bw(xstar_agents_optimal_times_density_const, "X* Optimal", 0, 4, 1200, False, 0, 2)
plt_bw(mstar_agents_times_density_const, "M* Optimal", 3, 4, 1200, False, 1, 2)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_mstar_optimal_times_density_const_half_bw")

ps.setupfig(thirdsize=True)
plt_95_ci(xstar_agents_optimal_times_density_const, "X* Optimal", 0, 4, 1200, False)
plt_95_ci(mstar_agents_times_density_const, "M* Optimal", 3, 4, 1200, False)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_mstar_optimal_times_density_const_half")

ps.setupfig()
plt_95_ci(xstar_agents_first_times_density_const, "X* Valid", 0, 4, 1200)
plt_95_ci(afs_agents_first_times_density_const, "AFS Valid", 1, 4, 1200)
plt_95_ci(cbs_agents_times_density_const, "CBS Valid/Opt.", 2, 4, 1200)
plt_95_ci(mstar_agents_times_density_const, "M* Valid/Opt.", 3, 4, 1200)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_vs_all_first_times_density_const")

ps.setupfig(halfsize=True)
plt_95_ci(xstar_agents_first_times_density_const, "X* Valid", 0, 4, 1200)
plt_95_ci(afs_agents_first_times_density_const, "AFS Valid", 1, 4, 1200)
plt_95_ci(cbs_agents_times_density_const, "CBS Valid/Opt.", 2, 4, 1200)
plt_95_ci(mstar_agents_times_density_const, "M* Valid/Opt.", 3, 4, 1200)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_vs_all_first_times_density_const_half")

ps.setupfig()
plt_95_ci(xstar_agents_optimal_times_density_const, "X* Optimal", 0, 4, 1200)
plt_95_ci(afs_agents_optimal_times_density_const, "AFS Optimal", 1, 4, 1200)
plt_95_ci(cbs_agents_times_density_const, "CBS Optimal", 2, 4, 1200)
plt_95_ci(mstar_agents_times_density_const, "M* Optimal", 3, 4, 1200)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_vs_all_optimal_times_density_const")

ps.setupfig(halfsize=True)
plt_95_ci(xstar_agents_optimal_times_density_const, "X* Optimal", 0, 4, 1200)
plt_95_ci(afs_agents_optimal_times_density_const, "AFS Optimal", 1, 4, 1200)
plt_95_ci(cbs_agents_times_density_const, "CBS Optimal", 2, 4, 1200)
plt_95_ci(mstar_agents_times_density_const, "M* Optimal", 3, 4, 1200)
ps.grid()
ps.legend('br')
ps.save_fig("xstar_vs_all_optimal_times_density_const_half")

ps.setupfig(halfsize=True)
plt_radius_vs_runtimes(xstar_radius_first_times, True)
ps.grid()
# ps.legend('ul')
ps.save_fig("radius_first_times")

ps.setupfig(halfsize=True)
plt_radius_vs_runtimes_bw(xstar_radius_first_times, True)
ps.grid()
# ps.legend('ul')
ps.save_fig("radius_first_times_bw")

ps.setupfig(halfsize=True)
plt_radius_vs_runtimes(xstar_radius_optimal_times, False)
ps.grid()
#ps.legend('ul')
ps.save_fig("radius_optimal_times")

ps.setupfig(halfsize=True)
plt_radius_vs_runtimes_bw(xstar_radius_optimal_times, False)
ps.grid()
#ps.legend('ul')
ps.save_fig("radius_optimal_times_bw")

# f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
# ps.setupfig(f)
# ps.grid(ax1)
# plt_radius_vs_runtimes(radius_30_first_times_lst, True, plt=ax1)
# ps.legend('ul', plt=ax1)

# ps.grid(ax2)
# plt_radius_vs_agents(radius_30_optimal_times_lst, False, plt=ax2)
# # ps.legend('ul')
# ps.save_fig("radius_30_first_optimal_times")

show_notification("Plot generation done")
