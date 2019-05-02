#!/usr/bin/env python3
import sys
import numpy as np
import yaml
import matplotlib
import argparse
from collections import namedtuple
import time
def generate_agents(num_agents, width, height):
    starts = []
    goals = []
    while len(starts) < num_agents:
      sx = np.random.randint(0, width)
      sy = np.random.randint(0, height)
      sxy = [sx, sy]
      gx = np.random.randint(0, width)
      gy = np.random.randint(0, height)
      gxy = [gx, gy]
      if sxy in starts or gxy in goals:
            continue
      starts.append(sxy)
      goals.append(gxy)

    Agent = namedtuple('Agent', 'name start goal')
    return [ Agent('agent{}'.format(e[0]), e[1][0], e[1][1] ) for e in enumerate(zip(starts, goals))] 

def generate_obstacles(num_obstacles, agents, width, height):
    starts = [a.start for a in agents]
    goals = [a.goal for a in agents]
    obs = []
    while len(obs) < num_obstacles:
        x = np.random.randint(0, width)
        y = np.random.randint(0, height)
        xy = [x, y]
        if xy in starts or xy in goals:
            continue
        obs.append(xy)
    return obs


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("agents", type=int, help="Number of agents")
    parser.add_argument("width", type=int, help="Map width")
    parser.add_argument("height", type=int, help="Map height")
    parser.add_argument("obs_density", type=float, help="Obstacle density")
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

agents = generate_agents(args.agents, args.width, args.height)
num_obstacles = int(args.obs_density * args.width * args.height)
obstacles = generate_obstacles(num_obstacles, agents, args.width, args.height)

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
        xy = [x, y]
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
write_yaml(agents, obstacles, args.width, args.height, f)
f.close()

f = open(args.afs_map_file, "w")
write_afs_map(agents, obstacles, args.width, args.height, f)
f.close()

f = open(args.afs_agents_file, "w")
wite_afs_agents(agents, obstacles, args.width, args.height, f)
f.close()
