#!/usr/bin/env python3
import os
import glob
import yaml
import numpy as np
import multiprocessing

def get_CPUs_to_use(max_cpus):
  return max(max_cpus - 2, 1)

pool_cpus = get_CPUs_to_use(multiprocessing.cpu_count())

print("Has {} CPUs. Pool CPUs: {}".format(multiprocessing.cpu_count(), pool_cpus))
destination_data_folder = "./fixed_building"

kNumAgents = 5

index_dict = {'CBS': 0, 'AFS': 1, 'M*': 2, 'X*': 3}
rev_index_dict = {v: k for k, v in index_dict.items()}

grid_map = None

def get_seed(file):
  return int(file.split("seed")[1].split("_")[0])

def get_map_file(label_file):
  return label_file.split("seed")[0] + "seed{}_generic.map".format(get_seed(label_file))

def make_agent_layer(agent_info, shape):
  grid_map = np.zeros(shape)
  sx, sy = agent_info['start']  
  gx, gy = agent_info['goal']  
  grid_map[sx, sy] = 1
  grid_map[gx, gy] = -1
  return grid_map

def read_agents(map_file, N):
  f = open(map_file)
  head = [next(f) for x in range(N * 3 + 1)]
  return yaml.load("\n".join(head), Loader=yaml.FullLoader)

def make_grid_map(map):
  dx, dy = map['map']['dimensions']
  shape = (dx, dy)
  grid_map = np.zeros(shape)
  for x, y in map['map']['obstacles']:
    grid_map[x, y] = 1
  assert(len(map['agents']) == kNumAgents)
  return grid_map


def map_file_to_x(map_file):
  global grid_map

  if grid_map is None:
    map = yaml.load(open(map_file, 'r'), Loader=yaml.FullLoader)
    grid_map = make_grid_map(map)
  else:
    print
    map = read_agents(map_file, kNumAgents)

  layers = [make_agent_layer(a, grid_map.shape) for a in map['agents']]
  layers.insert(0, grid_map)
  return np.concatenate([np.expand_dims(e, 0) for e in layers])
  
def times_tuple_list_to_vectors(times):
  times.sort(key=lambda e: e[0])
  winner = times[0]
  idx_times = [(index_dict[name], time) for time, name in times]
  times_vec = np.zeros(4)
  for idx, time in idx_times:
    times_vec[idx] = time
  times_vec = times_vec - times_vec.min()
  return times_vec

def label_file_to_ys(file):
  d = eval(open(file, 'r').read())  
  opt_times = times_tuple_list_to_vectors(d["optimal_times"])
  first_times = times_tuple_list_to_vectors(d["first_times"])
  return (opt_times, first_times)

def label_file_to_X_y(arguments):
  try:
    label_file, index = arguments
    print(index, ";", label_file)
    X = map_file_to_x(get_map_file(label_file))
    opt_y, first_y = label_file_to_ys(label_file)
    return X, opt_y, first_y
  except:
    return None

arguments = [(label_file, idx) for idx, label_file in enumerate(sorted(list(glob.glob(destination_data_folder + "/data/*.labels"))))]
print("Making pool of {} CPUs for {} entries".format(pool_cpus, len(arguments)))
# Initialize the globals.
label_file_to_X_y(arguments[0])
pool = multiprocessing.Pool(pool_cpus)
Xys = pool.map(label_file_to_X_y, arguments)

import joblib

lst_Xs, lst_opt_ys, lst_first_ys = zip(*[e for e in Xys if e is not None])

def stack_instances(instances):
  return np.concatenate([np.expand_dims(e, 0) for e in instances])

np_Xs = stack_instances(lst_Xs)
np_first_ys = stack_instances(lst_first_ys)
np_opt_ys = stack_instances(lst_opt_ys)

joblib.dump(np_Xs, destination_data_folder + "/np_Xs")
joblib.dump(np_first_ys, destination_data_folder + "/np_first_ys")
joblib.dump(np_opt_ys, destination_data_folder + "/np_opt_ys")

joblib.dump(Xys, destination_data_folder + "/Xys")