#!/usr/bin/env python3
import argparse

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("yaml", type=str, help="Input YAML File")
    parser.add_argument("map", type=str, help="Output .map file")
    parser.add_argument("agents", type=str, help="Output .agents file")
    return parser.parse_args()

args = get_args()

def get_line(line, ls):
    return [l for l in ls if line in l][0].replace(line, "")

def get_lines(line, ls):
    return [l.replace(line, "") for l in ls if line in l]

ls = open(args.yaml, 'r').readlines()
width, height = tuple(eval(get_line("dimensions:", ls)))
print(width, height)

def make_map(map_file, width, height, ls):
    f = open(map_file, 'w')
    obstacle_points = set([tuple(eval(e)) for e in get_lines("- !!python/tuple ", ls)])
    # header
    f.write("{},{}\n".format(width + 2, height + 2))
    f.write('1' * (width + 2) + '\n')
    for row in range(width):
        f.write('1')
        for col in range(height):
            if (row, col) in obstacle_points:
                f.write('1')
            else:
                f.write('0')
        f.write('1\n')
    f.write('1' * (width + 2) + '\n')

def make_agents(agents_file, ls):
    f = open(agents_file, 'w')
    goals = ([tuple(eval(e)) for e in get_lines("-   goal: ", ls)])
    starts = ([tuple(eval(e)) for e in get_lines("start: ", ls)])
    for start, goal in zip(starts, goals):
        f.write("{},{},{},{}\n".format(start[0] + 1, start[1] + 1,goal[0] + 1, goal[1] + 1,))

make_map(args.map, width, height, ls)
make_agents(args.agents, ls)
