import shared_helpers as sh
import argparse

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

def args_to_string(args):
    return str(args).replace('(', '').replace(')', '').replace(' ', '').replace('=', '').replace(',', '')

def args_to_title(args):
    return str(args.width) + "x" + str(args.height) + ", " + str(args.agents) + " Agents, " + str(args.obs_density * 100) + "% Obst"

fig_path = '../../Pictures/acbs_figs/'

args = get_args()
outfile_infix = "supplemental_"

xstar_data = sh.read_from_file("xstar_{}data_lst_{}".format(outfile_infix, args_to_string(args)))
cbs_data = sh.read_from_file("cbs_{}data_lst_{}".format(outfile_infix, args_to_string(args)))
acbs_data = sh.read_from_file("nrwcbs_{}data_lst_{}".format(outfile_infix, args_to_string(args)))
nwcbs_data = sh.read_from_file("nwcbs_{}data_lst_{}".format(outfile_infix, args_to_string(args)))

# cbs_c = 0
# acbs_c = 0
# xstar_c = 0
# acbs_c_f = 0
# cbs_c_f = 0
# xstar_c_f = 0


# for i in range(len(cbs_data)):
#     if xstar_data[i].runtimes[-1] < cbs_data[i].runtimes:
#         if xstar_data[i].runtimes[-1] < acbs_data[i].runtimes[-1]:
#             xstar_c+=1
#         elif xstar_data[i].runtimes[-1] == xstar_data[i].timeout:
#             pass
#         else:
#             acbs_c += 1
#     elif acbs_data[i].runtimes[-1] < cbs_data[i].runtimes:
#         acbs_c += 1
#     elif acbs_data[i].runtimes[-1] > cbs_data[i].runtimes:
#         cbs_c += 1

#     if xstar_data[i].runtimes[0] < cbs_data[i].runtimes:
#         if xstar_data[i].runtimes[0] < acbs_data[i].runtimes[0]:
#             xstar_c_f+=1
#         elif xstar_data[i].runtimes[0] > acbs_data[i].runtimes[0]:
#             acbs_c_f += 1
#     elif acbs_data[i].runtimes[0] < cbs_data[i].runtimes:
#         acbs_c_f += 1
#     elif acbs_data[i].runtimes[0] > cbs_data[i].runtimes:
#         cbs_c_f += 1
    
# print("cbs faster: " + str(cbs_c))
# print("xstar faster: " + str(xstar_c))
# print("acbs faster: " + str(acbs_c))
# print("")
# print("cbs first faster: " + str(cbs_c_f))
# print("xstar first faster: " + str(xstar_c_f))
# print("acbs first faster: " + str(acbs_c_f))

import matplotlib.pyplot as plt
import numpy as np
import math
import yaml

xstar_first_list = [i.runtimes[0] for i in xstar_data]
acbs_first_list = [i.runtimes[0] for i in acbs_data]
nwcbs_first_list = [i.runtimes[0] for i in nwcbs_data]
cbs_list = [i.runtimes for i in cbs_data]
acbs_list = [i.runtimes[-1] if i.runtimes[-1] != -1 else i.runtimes[-2] for i in acbs_data]
xstar_list = [i.runtimes[-1] for i in xstar_data]
nwcbs_list = [i.runtimes[-1] if i.runtimes[-1] != -1 else i.runtimes[-2] for i in nwcbs_data]

# acb = 0
# xst = 0
# cbb= 0
# for i in range(len(cbs_list)):
#     if acbs_list[i] < 1:
#         acb += 1
#     if xstar_list[i] < 1:
#         xst += 1
#     if cbs_list[i] < 1:
#         cbb += 1
# print('acbs less than 1: ' +str(acb))
# print('xstar less than 1: '+str(xst))
# print('cbs less than 1: ' + str(cbb))

n, bins, patches = plt.hist(cbs_list, 100, facecolor='red', alpha=0.5)
n2, bins2, patches2 = plt.hist(acbs_list, 100, facecolor='green', alpha=0.5)
n3, bins3, patches3 = plt.hist(xstar_list, 100, facecolor='blue', alpha=0.5)

n4, bins4, patches4 = plt.hist(xstar_first_list, 100, facecolor='blue', alpha=0.5)
n5, bins5, patches5 = plt.hist(acbs_first_list, 100, facecolor='blue', alpha=0.5)

n6, bins6, patches6 = plt.hist(nwcbs_first_list, 100, facecolor='purple', alpha=0.5)
n7, bins7, patches7 = plt.hist(nwcbs_list, 100, facecolor='purple', alpha=0.5)

plt.clf()

logbins = np.logspace(np.log10(bins[0]),np.log10(bins[-1]),len(bins))       # cbs
logbins2 = np.logspace(np.log10(bins2[0]),np.log10(bins2[-1]),len(bins2))   # acbs
logbins3 = np.logspace(np.log10(bins3[0]),np.log10(bins3[-1]),len(bins3))   # xstar

logbins4 = np.logspace(np.log10(bins4[0]),np.log10(bins4[-1]),len(bins4))   # xstar first
logbins5 = np.logspace(np.log10(bins5[0]),np.log10(bins5[-1]),len(bins5))   # acbs first

logbins6 = np.logspace(np.log10(bins6[0]),np.log10(bins6[-1]),len(bins6)) # nwcbs first
logbins7 = np.logspace(np.log10(bins7[0]),np.log10(bins7[-1]),len(bins7)) # nwcbs

# CUMULATIVE HISTOGRAMS FOR FULL RUNTIMES
plt.xscale('log')
plt.hist(cbs_list, logbins,  density=1, histtype='step',color='red',
         cumulative=True, label='CBS')

plt.hist(acbs_list, logbins2, density=1, histtype='step',color='green',
         cumulative=True, label='ACBS')

plt.hist(nwcbs_list, logbins7, density=1, histtype='step',color='purple',
         cumulative=True, label='Naive ACBS')

plt.hist(xstar_list, logbins3, density=1, histtype='step',color='blue',
         cumulative=True, label='X*')

plt.grid()
plt.legend(loc='upper left')
plt.title("Optimal Cumulative Histograms " + args_to_title(args)) #agents width height density trials
plt.xlabel("Runtime [s]")
# plt.show()
plt.savefig(fig_path+args_to_string(args)+"Opt.png")

plt.clf()

# CUMULATIVE HISTOGRAMS FOR FIRST RUNTIMES
plt.xscale('log')
plt.hist(cbs_list, logbins,  density=1, histtype='step',color='red',
         cumulative=True, label='CBS')

plt.hist(xstar_first_list, logbins4, density=1, histtype='step',color='blue',
         cumulative=True, label='X*')

plt.hist(acbs_first_list, logbins5, density=1, histtype='step', color='green',
         cumulative=True, label='ACBS')

plt.hist(acbs_first_list, logbins6, density=1, histtype='step', color='purple',
         cumulative=True, label='Naive ACBS')

plt.grid()
plt.legend(loc='upper left')
plt.title("First Solution Cumulative Histograms: " + args_to_title(args))
plt.xlabel("Runtime [s]")
# plt.show()
plt.savefig(fig_path+args_to_string(args)+"First.png")