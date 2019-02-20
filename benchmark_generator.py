#!/usr/bin/env python3
import sys
import numpy as np
import yaml
import matplotlib
import argparse
from collections import namedtuple

def generate_agents(num_agents, width, height):
    starts = []
    goals = []
    while len(starts) < num_agents:
      sx = np.random.randint(0, width - 1)
      sy = np.random.randint(0, height - 1)
      sxy = [sx, sy]
      gx = np.random.randint(0, width - 1)
      gy = np.random.randint(0, height - 1)
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
    parser.add_argument("out_file", type=str, help="Output file")
    parser.add_argument("--seed", type=int, default=None, help="RNG Seed")
    return parser.parse_args()

args = get_args();

if args.seed is not None:
    np.random.seed(args.seed)

agents = generate_agents(args.agents, args.width, args.height)
num_obstacles = int(args.obs_density * args.width * args.height)
obstacles = generate_obstacles(num_obstacles, agents, args.width, args.height)
print(agents)

f = open(args.out_file, "w")
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

write_yaml(agents, obstacles, args.width, args.height, f)
