#!/usr/bin/env python3
import collections
import glob
import matplotlib
import re
from functools import reduce

import plot_styling as ps
from matplotlib import pyplot as plt
from shared_helpers import XStarData
from shared_helpers import MStarData
from shared_helpers import CBSData
from shared_helpers import AFSData

FileData = collections.namedtuple('FileData', ['filename',
                                               'agents',
                                               'iter',
                                               'trial',
                                               'seed'])
kTimeout = 1200


def filename_to_filedata(l):
    regex = r"[a-zA-Z_\/]*(\d\d\d|\d\d|\d)_iter_(\d\d\d|\d\d|\d)_trial_(\d\d\d|\d\d|\d)_seed_(\d\d\d\d\d\d|\d\d\d\d\d|\d\d\d\d|\d\d\d|\d\d|\d).result" # noqa
    matches = list(re.finditer(regex, l, re.MULTILINE))
    match = matches[0]
    agents = int(match.group(1))
    itr = int(match.group(2))
    trial = int(match.group(3))
    seed = int(match.group(4))
    return FileData(l, agents, itr, trial, seed)


def get_search_radius(filename):
    regex = r"search_window_xstar(\d)"
    match = list(re.finditer(regex, filename, re.MULTILINE))[0]
    return int(match.group(1))


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
    [filename_to_filedata(f) for f in glob.glob('datasave/xstar*.result')]
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

ratio_file_datas = \
    [filename_to_filedata(f)
     for f in glob.glob('datasave/xstar_ratio_*.result') if "1200" not in f]
ratio_timeout_dict = {20: 300, 30: 450, 40: 600, 60: 900, 80: 1200}
ratio_agents_first_times_lst = \
    sorted([(d.agents, get_total_first_plan_time(d.filename))
            for d in ratio_file_datas])
ratio_agents_optimal_times_lst = \
    sorted([(d.agents, get_optimal_time_or_timeout(d.filename, kTimeout))
            for d in ratio_file_datas])

kRadiusTimeout = 300

radius_40_file_datas = [(filename_to_filedata(f), get_search_radius(f)) for f in glob.glob('datasave/xstar_grow_search_window*.result') if "agents_40" in f]
radius_40_first_times_lst = sorted([(r, get_total_first_plan_time(d.filename, kRadiusTimeout)) for d, r in radius_40_file_datas])
radius_40_optimal_times_lst = sorted([(r, get_optimal_time_or_timeout(d.filename, kRadiusTimeout)) for d, r in radius_40_file_datas])

radius_30_file_datas = [(filename_to_filedata(f), get_search_radius(f)) for f in glob.glob('datasave/xstar_grow_search_window*.result') if "agents_30" in f]
radius_30_first_times_lst = sorted([(r, get_total_first_plan_time(d.filename, kRadiusTimeout)) for d, r in radius_30_file_datas])
radius_30_optimal_times_lst = sorted([(r, get_optimal_time_or_timeout(d.filename, kRadiusTimeout)) for d, r in radius_30_file_datas])

radius_20_file_datas = [(filename_to_filedata(f), get_search_radius(f)) for f in glob.glob('datasave/xstar_grow_search_window*.result') if "agents_20" in f]
radius_20_first_times_lst = sorted([(r, get_total_first_plan_time(d.filename, kRadiusTimeout)) for d, r in radius_20_file_datas])
radius_20_optimal_times_lst = sorted([(r, get_optimal_time_or_timeout(d.filename, kRadiusTimeout)) for d, r in radius_20_file_datas])


def read_from_file(name):  
    f = open("{}".format(name), 'r')
    data = eval(f.read())
    f.close()
    return data


def get_first_runtimes(data):
    return data.runtimes[0]


xstar_datas = [read_from_file(f) for f in glob.glob('datasave/xstar_data_lst_*density0.05*')]
xstar_datas = [x for lst in xstar_datas for x in lst]
xstar_agents_first_times = [(x.num_agents, x.runtimes[0]) for x in xstar_datas]
xstar_agents_optimal_times = [(x.num_agents, x.runtimes[-1]) for x in xstar_datas]

cbs_datas = [read_from_file(f) for f in glob.glob('datasave/cbs_data_lst_*density0.05*')]
cbs_datas = [x for lst in cbs_datas for x in lst]
cbs_agents_times = [(x.num_agents, x.runtimes) for x in cbs_datas]

afs_datas = [read_from_file(f) for f in glob.glob('datasave/afs_data_lst_*density0.05*')]
afs_datas = [x for lst in afs_datas for x in lst]
afs_agents_first_times = [(x.num_agents, x.runtimes[0]) for x in afs_datas]
afs_agents_optimal_times = [(x.num_agents, x.runtimes[-1]) for x in afs_datas]

mstar_datas = [read_from_file(f) for f in glob.glob('datasave/mstar_data_lst_*density0.05*')]
mstar_datas = [x for lst in mstar_datas for x in lst]
mstar_agents_times = [(x.num_agents, x.runtimes) for x in mstar_datas]


def draw_timeout(timeout, xs, plt=plt):
    if type(timeout) is int:
        plt.axhline(timeout, color='black', lw=0.7, linestyle='--')
    elif type(timeout) is dict:
        ys = [timeout[x] for x in xs]
        plt.plot(xs, ys, linestyle='--',
                 color='black')

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


def plt_95_ci(agents_times_lst, name, plt_idx, max_idx, timeout=None):
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
    plt.ylabel("Time (seconds)")
    plt.xlabel("Number of agents")
    plt.xticks(xs)

    draw_timeout(timeout, xs)


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


############################
# Head to head comparisons #
############################
print("Head to head comparisons")

ps.setupfig()
plt_95_ci(xstar_agents_optimal_times, "X* Optimal", 0, 4, 1200)
plt_95_ci(afs_agents_optimal_times, "AFS Optimal", 1, 4, 1200)
plt_95_ci(xstar_agents_first_times, "X* First", 2, 4, 1200)
plt_95_ci(afs_agents_first_times, "AFS First", 3, 4, 1200)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_afs_first_optimal_times")

ps.setupfig()
plt_95_ci(xstar_agents_first_times, "X* First", 0, 2, 1200)
plt_95_ci(afs_agents_first_times, "AFS First", 1, 2, 1200)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_afs_first_times")

ps.setupfig()
plt_95_ci(xstar_agents_optimal_times, "X* Optimal", 0, 2, 1200)
plt_95_ci(afs_agents_optimal_times, "AFS Optimal", 1, 2, 1200)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_afs_optimal_times")

ps.setupfig()
plt_95_ci(xstar_agents_first_times, "X* First", 0, 2, 1200)
plt_95_ci(cbs_agents_times, "CBS First/Optimal", 1, 2, 1200)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_cbs_first_times")

ps.setupfig()
plt_95_ci(xstar_agents_optimal_times, "X* Optimal", 0, 2, 1200)
plt_95_ci(cbs_agents_times, "CBS Optimal", 1, 2, 1200)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_cbs_optimal_times")

ps.setupfig()
plt_95_ci(xstar_agents_first_times, "X* First", 0, 2, 1200)
plt_95_ci(mstar_agents_times, "M* First/Optimal", 1, 2, 1200)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_mstar_first_times")

ps.setupfig()
plt_95_ci(xstar_agents_optimal_times, "X* Optimal", 0, 2, 1200)
plt_95_ci(mstar_agents_times, "M* Optimal", 1, 2, 1200)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_mstar_optimal_times")

ps.setupfig()
plt_95_ci(cbs_agents_times, "CBS First/Optimal", 0, 4, 1200)
plt_95_ci(mstar_agents_times, "M* First/Optimal", 1, 4, 1200)
plt_95_ci(afs_agents_first_times, "AFS First", 2, 4, 1200)
plt_95_ci(xstar_agents_first_times, "X* First", 3, 4, 1200)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_vs_all_first_times")

ps.setupfig()
plt_95_ci(cbs_agents_times, "CBS Optimal", 0, 4, 1200)
plt_95_ci(mstar_agents_times, "M* Optimal", 1, 4, 1200)
plt_95_ci(afs_agents_optimal_times, "AFS Optimal", 2, 4, 1200)
plt_95_ci(xstar_agents_optimal_times, "X* Optimal", 3, 4, 1200)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_vs_all_optimal_times")


###########################################
# X* Only Agents vs Performance Breakdown #
###########################################
print("X* Only Agents vs Performance Breakdown")

ps.setupfig()
plt_cis(agents_first_times_lst, "Time to first solution", kTimeout)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_first_solution_ci")

ps.setupfig()
plt_cis(agents_optimal_times_lst, "Time to optimal solution", kTimeout)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_optimal_solution_ci")

ps.setupfig()
plt_percentiles(agents_first_times_lst, "Time to first solution", kTimeout)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_first_solution_percentile")

ps.setupfig()
plt_percentiles(agents_optimal_times_lst, "Time to optimal solution", kTimeout)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_optimal_solution_percentile")


def plt_window_agents_boxplot(num_agents_in_window_times_lst, title, timeout=None):
    # Build {agent : runtimes} dict
    num_agents_in_window_to_optimal_times_dict = \
        reduce(add_to_dict, num_agents_in_window_times_lst, dict())
    # Remove keys less than 2
    num_agents_in_window_to_optimal_times_dict = \
        {k: v for k, v in num_agents_in_window_to_optimal_times_dict.items()
         if k >= 2}
    # Fill in any missing count in the range with empty list
    max_key = max(num_agents_in_window_to_optimal_times_dict.keys())
    min_key = min(num_agents_in_window_to_optimal_times_dict.keys())
    num_agents_in_window_to_optimal_times_dict = \
        {e: num_agents_in_window_to_optimal_times_dict.get(e, []) for e in
         range(min_key, max_key + 1)}
    # Convert to list of (k, list(v)) pairs sorted by k
    keys_values = \
        [(k, sorted(v)) for k, v in
         sorted(num_agents_in_window_to_optimal_times_dict.items(),
                key=lambda kv: kv[0])]
    ks, vs = zip(*keys_values)
    flierprops = dict(marker='.', markersize=1)
    plt.boxplot(vs, notch=False, flierprops=flierprops)
    plt.gca().set_xticklabels(ks)
    plt.xlabel("Largest number of agents in window")
    plt.ylabel("Time (seconds)")

    draw_timeout(timeout, None)


def plt_window_agents_hist(num_agents_in_window_times_lst, title, plt=plt, draw_y_label=True, xlabel="Largest number of agents in window"):
    # Build {agent : runtimes} dict
    num_agents_in_window_to_optimal_times_dict = \
        reduce(add_to_dict, num_agents_in_window_times_lst, dict())
    # Remove keys less than 2
    num_agents_in_window_to_optimal_times_dict = \
        {k: v for k, v in num_agents_in_window_to_optimal_times_dict.items()
         if k >= 2}
    # Fill in any missing count in the range with empty list
    max_key = max(num_agents_in_window_to_optimal_times_dict.keys())
    min_key = min(num_agents_in_window_to_optimal_times_dict.keys())
    num_agents_in_window_to_optimal_times_dict = \
        {e: num_agents_in_window_to_optimal_times_dict.get(e, []) for e in
         range(min_key, max_key + 1)}
    # Convert to list of (k, list(v)) pairs sorted by k
    k_vs = \
        [(k, len(v)) for k, v in
         num_agents_in_window_to_optimal_times_dict.items()]
    ks, vs = zip(*k_vs)
    plt.bar(ks, vs)
    if plt is matplotlib.pyplot:
        plt.xticks(ks)
        plt.xlabel(xlabel)
        plt.ylabel("Occurrences")
    else:
        plt.set_xticks(ks)
        plt.set_xlabel(xlabel)
        if draw_y_label:
            plt.set_ylabel("Occurrences")


######################################################
# Histogram and boxplot of window dimensions vs time #
######################################################
print("Histogram and boxplot of window dimensions vs time")

ps.setupfig()
plt_window_agents_boxplot(num_agents_in_window_first_times_lst,
                          "Largest number of agents in window vs time to "
                          "first solution", kTimeout)
plt.yscale('log')
ps.grid()
ps.save_fig("window_vs_time_to_first_boxplot")

ps.setupfig()
plt_window_agents_hist(num_agents_in_window_first_times_lst,
                       "Largest number of agents in window vs occurrences in "
                       "first solution")
plt.yscale('log')
ps.grid()
ps.save_fig("window_vs_time_to_first_hist")

ps.setupfig()
plt_window_agents_boxplot(num_agents_in_window_optimal_times_lst,
                          "Largest number of agents in any window vs time to "
                          "optimal solution", kTimeout)
plt.yscale('log')
ps.grid()
ps.save_fig("window_vs_time_to_optimal_boxplot")

ps.setupfig()
plt_window_agents_hist(num_agents_in_window_optimal_times_lst,
                          "Largest number of agents in window vs occurrences in "
                          "optimal solution")
plt.yscale('log')
ps.grid()
ps.save_fig("window_vs_time_to_optimal_hist")

f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
ps.setupfig(f)
ps.grid(ax1)
plt_window_agents_hist(num_agents_in_window_first_times_lst,
                       "Largest number of agents in window vs occurrences in "
                       "first solution", plt=ax1, xlabel="Largest number of agents in window\nfor first solution")
ax1.set_yscale('log')
ps.grid(ax2)

plt_window_agents_hist(num_agents_in_window_optimal_times_lst,
                       "Largest number of agents in window vs occurrences in "
                       "optimal solution", plt=ax2, draw_y_label=False, xlabel="Largest number of agents in window\nfor optimal solution")
ps.save_fig("window_vs_time_both_hist")


def plt_radius_vs_agents(data, printylabel, plt=plt, timeout=kRadiusTimeout):
    data_dict = reduce(add_to_dict, data, dict())
    radius_to_100_bounds_lst = \
        [[k] + list(get_ci(v, 100)) for k, v in data_dict.items()]
    radius_to_95_bounds_lst = \
        [[k] + list(get_ci(v, 95)) for k, v in data_dict.items()]
    radius_to_90_bounds_lst = \
        [[k] + list(get_ci(v, 90)) for k, v in data_dict.items()]
    radius_to_75_bounds_lst = \
        [[k] + list(get_ci(v, 75)) for k, v in data_dict.items()]
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


######################
# Density vs runtime #
######################
print("Density vs runtime")
print("TODO: FIX")
exit(0)
# ps.setupfig()
# plt_radius_vs_agents(radius_40_first_times_lst)
# ps.grid()
# ps.legend('ul')
# ps.save_fig("radius_40_first_times")

# ps.setupfig()
# plt_radius_vs_agents(radius_40_optimal_times_lst)
# ps.grid()
# ps.legend('ul')
# ps.save_fig("radius_40_optimal_times")

f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
ps.setupfig(f)
ps.grid(ax1)
plt_radius_vs_agents(radius_30_first_times_lst, True, plt=ax1)
ps.legend('ul', plt=ax1)

ps.grid(ax2)
plt_radius_vs_agents(radius_30_optimal_times_lst, False, plt=ax2)
# ps.legend('ul')
ps.save_fig("radius_30_first_optimal_times")

f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
ps.setupfig()
plt.subplot(121)
plt_radius_vs_agents(radius_20_first_times_lst, True)
ps.grid()
ps.legend('ul')

plt.subplot(122)
plt_radius_vs_agents(radius_20_optimal_times_lst, False)
ps.grid()
# ps.legend('ul')
ps.save_fig("radius_20_first_optimal_times")

# =============================================================================

ps.setupfig()
plt_cis(ratio_agents_first_times_lst, "Time to first solution", kTimeout)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_first_solution_for_ratio")

ps.setupfig()
plt_cis(ratio_agents_optimal_times_lst, "Time to optimal solution", kTimeout)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_optimal_solution_for_ratio")
