#!/usr/bin/env python3
import collections
import glob
import re
import numpy as np
from scipy import stats
from functools import reduce

import plot_styling as ps
from matplotlib import pyplot as plt

FileData = collections.namedtuple('FileData', ['filename',
                                               'agents',
                                               'iter',
                                               'trial',
                                               'seed'])
kTimeout = 300


def filename_to_filedata(l):
    regex = r"[a-zA-Z_\/]*(\d\d\d|\d\d|\d)_iter_(\d\d\d|\d\d|\d)_trial_(\d\d\d|\d\d|\d)_seed_(\d\d\d\d\d\d|\d\d\d\d\d|\d\d\d\d|\d\d\d|\d\d|\d).result" # noqa
    matches = list(re.finditer(regex, l, re.MULTILINE))
    match = matches[0]
    agents = int(match.group(1))
    itr = int(match.group(2))
    trial = int(match.group(3))
    seed = int(match.group(4))
    return FileData(l, agents, itr, trial, seed)


def get_line(ls, string, t):
    fls = [l for l in ls if string in l]
    if len(fls) > 1:
        print(string, "is not unique")
    elif len(fls) < 1:
        print(string, "is not found")
    assert(len(fls) == 1)
    line = fls[0]
    line = line.replace(string, '').replace(':', '').strip()
    return t(line)


def get_total_first_plan_time(filename):
    ls = open(filename, 'r').readlines()
    if len(ls) == 0:
        return kTimeout
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
    [filename_to_filedata(f) for f in glob.glob('datasave/xstar_idv_*.result')]
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


def agents_to_timeout(agents):
    lookup = {20: 300, 30: 450, 40: 600, 60: 900, 80: 1200}
    return lookup[agents]


ratio_file_datas = \
    [filename_to_filedata(f)
     for f in glob.glob('datasave/xstar_ratio_*.result')]
ratio_agents_first_times_lst = \
    sorted([(d.agents, get_total_first_plan_time(d.filename))
            for d in ratio_file_datas])
ratio_agents_optimal_times_lst = \
    sorted([(d.agents, get_optimal_time_or_timeout(d.filename, agents_to_timeout(d.agents)))
            for d in ratio_file_datas])


def plt_cis(agents_times_lst, title, timeout=None):
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

    if timeout is not None:
        plt.axhline(timeout, color='black', lw=0.7, linestyle='--')
    else:
        plt.plot(xs, [agents_to_timeout(x) for x in xs], linestyle='--',
                 color='black')
    slope, intercept, r_value, p_value, std_err = stats.linregress(xs, medians)
    print("xs:", xs)
    print("medians:", medians)
    fitted_line = [(slope * x + intercept) for x in xs]
    print("Intercept:", intercept)

    textstr = '\n'.join((
    r'$\mathrm{slope}=%.5f$' % (slope, ),
    r'$\mathrm{intercept}=%.5f$' % (intercept, ),
    r'$r=%.5f$' % (r_value, )))

    
    # these are matplotlib.patch.Patch properties
    props = dict(boxstyle='round', facecolor='white', alpha=0.5)

    # place a text box in upper left in axes coords
    plt.text(0.73, 0.165, textstr, transform=plt.gca().transAxes,
             verticalalignment='top', horizontalalignment='left', bbox=props)

    plt.plot(xs, fitted_line, color='green')


def plt_percentiles(agents_times_lst, title):
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


ps.setupfig()
plt_cis(agents_first_times_lst, "Time to first solution", kTimeout)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_first_solution_ci")

ps.setupfig()
plt_cis(agents_optimal_times_lst, "Time to optimal solution", kTimeout)
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_optimal_solution_solution_ci")

ps.setupfig()
plt_percentiles(agents_first_times_lst, "Time to first solution")
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_first_solution_percentile")

ps.setupfig()
plt_percentiles(agents_optimal_times_lst, "Time to optimal solution")
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_optimal_solution_percentile")

# =============================================================================

ps.setupfig()
plt_cis(ratio_agents_first_times_lst, "Time to first solution")
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_first_solution_for_ratio")

ps.setupfig()
plt_cis(ratio_agents_optimal_times_lst, "Time to optimal solution")
ps.grid()
ps.legend('ul')
ps.save_fig("xstar_optimal_solution_for_ratio")


def plt_boxplot(num_agents_in_window_times_lst, title):
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
    plt.xlabel("Largest number of agents in any window")
    plt.ylabel("Time (seconds)")


ps.setupfig()
plt_boxplot(num_agents_in_window_optimal_times_lst,
            "Largest number of agents in any window vs time to optimal "
            "solution time")
plt.axhline(kTimeout, color='black', lw=0.7, linestyle='--')
plt.yscale('log')
ps.grid()
ps.save_fig("boxplot")
