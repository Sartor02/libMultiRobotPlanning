#!/usr/bin/env python3
import sys
import numpy as np
import yaml
import matplotlib
import argparse
from collections import namedtuple
import time
import os
def generate_agents(num_agents, width, height, occupancy_lst):
    starts = []
    goals = []
    while len(starts) < num_agents:
      sx = np.random.randint(0, width)
      sy = np.random.randint(0, height)
      sxy = [sx, sy]
      gx = np.random.randint(0, width)
      gy = np.random.randint(0, height)
      gxy = [gx, gy]
      if sxy in starts or gxy in goals or tuple(sxy) in occupancy_lst or tuple(gxy) in occupancy_lst:
            continue
      starts.append(sxy)
      goals.append(gxy)

    Agent = namedtuple('Agent', 'name start goal')
    return [ Agent('agent{}'.format(e[0]), e[1][0], e[1][1] ) for e in enumerate(zip(starts, goals))] 

def load_map_file(map_file, occupied_char='@', valid_chars={'@', '.', 'T'}):
    if not os.path.isfile(map_file):
        print("Map file not found!")
        exit(-1)
    map_ls = open(map_file, 'r').readlines()
    height = int(map_ls[1].replace("height ", ""))
    width = int(map_ls[2].replace("width ", ""))
    map_ls = map_ls[4:]
    map_ls = [l.replace('\n', '') for l in map_ls]
    occupancy_lst = set()
    assert(len(map_ls) == height)
    for y, l in enumerate(map_ls):
        assert(len(l) == width)
        for x, c in enumerate(l):
            assert(c in valid_chars)
            if c == occupied_char:
                occupancy_lst.add((x, y))
    return width, height, occupancy_lst


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("agents", type=int, help="Number of agents")
    parser.add_argument("benchmark_map_file", type=str, help="CBS Output file")
    parser.add_argument("cbs_out_file", type=str, help="CBS Output file")
    parser.add_argument("afs_map_file", type=str, help="AFS Map file")
    parser.add_argument("afs_agents_file", type=str, help="AFS Agents file")
    parser.add_argument("--seed", type=int, default=None, help="RNG Seed")
    return parser.parse_args()

args = get_args();

if args.seed is not None:
    np.random.seed(args.seed)
else:
    seed_value = int((time.time() * 1000) % (2**32-1));
    print("Seed:", seed_value)
    np.random.seed(seed_value)

width, height, occupancy_lst = load_map_file(args.benchmark_map_file)
agents = generate_agents(args.agents, width, height, occupancy_lst)

def write_yaml(agents, obstacles, width, height, f):
    f.write("agents:\n")
    for a in agents:
        rs = \
"""-   goal: [{}, {}]
    name: {}
    start: [{}, {}]"""
        subs = rs.format(a.start[0], a.start[1], a.name, a.goal[0], a.goal[1])
        f.write(subs + '\n')

    f.write("map:\n")
    f.write("    dimensions: [{}, {}]\n".format(width, height))
    f.write("    obstacles:\n")
    for o in obstacles:
        f.write("    - !!python/tuple [{}, {}]\n".format(o[0], o[1]))
        
def write_afs_map(agents, obstacles, width, height, f):
    f.write("{},{}\n".format(width + 2, height + 2))
    f.write("{}\n".format(("1," * (width + 2))[:-1]))
    
    for x in range(width):
      f.write('1,')
      for y in range(height):
        xy = (x, y)
        if xy in obstacles:
          f.write("1,")
        else:
          f.write("0,")
      f.write('1\n')
    f.write("{}\n".format(("1," * (width + 2))[:-1]))
    
def wite_afs_agents(agents, obstacles, width, height, f):
    f.write(str(len(agents)) + '\n')
    for a in agents:
      f.write("{},{},{},{}\n".format(a.start[0] + 1,a.start[1] + 1,a.goal[0] + 1,a.goal[1] + 1))

f = open(args.cbs_out_file, "w")
write_yaml(agents, occupancy_lst, width, height, f)
f.close()

f = open(args.afs_map_file, "w")
write_afs_map(agents, occupancy_lst, width, height, f)
f.close()

f = open(args.afs_agents_file, "w")
wite_afs_agents(agents, occupancy_lst, width, height, f)
f.close()
