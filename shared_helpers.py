import collections
import argparse

Runtime = collections.namedtuple('Runtime', ['datalst',])
XStarData = collections.namedtuple('XStarData', ['obs_density', 'width', 'height', 'num_agents', 'timeout', 'bounds', 'runtimes',])
AFSData = collections.namedtuple('AFSData', ['obs_density', 'width', 'height', 'num_agents', 'timeout', 'bounds', 'runtimes',])
CBSData = collections.namedtuple('CBSData', ['obs_density', 'width', 'height', 'num_agents', 'timeout', 'runtimes',])
MStarData = collections.namedtuple('MStarData', ['obs_density', 'width', 'height', 'num_agents', 'timeout', 'runtimes',])

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("agents", type=int, help="Number of agents")
    parser.add_argument("width", type=int, help="Map width")
    parser.add_argument("height", type=int, help="Map height")
    parser.add_argument("obs_density", type=float, help="Obstacle density")
    parser.add_argument("iter_per_trial", type=int, help="Number of iterations to average runtime over")
    parser.add_argument("num_trials", type=int, help="Number of trials")
    parser.add_argument("timeout", type=int, help="Run timeout")
    return parser.parse_args()

def save_to_file(name, data):
  f = open("datasave/{}.datasave".format(name), "w")
  f.write(str(data))
  f.close()
  
def read_from_file(name):  
  f = open("datasave/{}.datasave".format(name), 'r')
  data = eval(f.read())
  f.close()
  return data
