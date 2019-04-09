#!/usr/bin/env python3
import subprocess
from matplotlib import pyplot as plt
import shlex
import hashlib
import signal
import sys
import os
import argparse
import glob
from shutil import copyfile

current_proc = None
def signal_handler(sig, frame):
  print(sys.argv[0], 'Signal caught')
  if current_proc is not None:
    current_proc.terminate()
  sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def get_args():
  parser = argparse.ArgumentParser()
  parser.add_argument("iterations", type=int, help="X* iterations")
  parser.add_argument("trials", type=int, help="X* trials")
  return parser.parse_args()

args = get_args();

kMapBaseName = "map"
kNumAgents = [10, 20]
kDensity = 0.05
kWidth = 100
kHeight = 100
kTimeout = 300 # 5 minutes

def std_map_name(map_base_name):
  return map_base_name + ".stdmap"

def afs_map_name(map_base_name):
  return map_base_name + ".afsmap"

def afs_agents_name(map_base_name):
  return map_base_name + ".afsagents"

def generate_new_scenario(agents, width, height, obs_density, seed, map_base_name):
  cmd = "./benchmark_generator.py {} {} {} {} {} {} {} --seed {}"\
    .format(agents, 
            width, 
            height, 
            obs_density, 
            std_map_name(map_base_name), 
            afs_map_name(map_base_name), 
            afs_agents_name(map_base_name), 
            seed)
  if subprocess.call(cmd, shell=True) != 0:
    print("Genrate failed")
    exit(-1)
    
def select_seed(idx, num_agents, density, width, height):
  return int((idx + 1) * 3 + num_agents * 7 + 1000 * density + width + height * 2)

def run():
  global current_proc
  for num_agents in kNumAgents:
    for idx in range(args.iterations):
      seed = select_seed(idx, num_agents, kDensity, kWidth, kHeight)
      generate_new_scenario(num_agents, kWidth, kHeight, kDensity, seed, kMapBaseName)
      print("Agents:", num_agents, "Iter:", idx, "Seed:", seed)
      cmd = "./collect_data_xstar.py {} {} {} agents_{}_iteration_{}_trial_ _seed_{}.result".format(std_map_name(kMapBaseName), kTimeout, args.trials, num_agents, idx, seed)
      current_proc = subprocess.Popen(shlex.split(cmd))
      current_proc.wait()
      
run()

#def is_file_complete(file_name):
  #file_shibboleth = "Complete!"
  #f = open(file_name, 'r')
  #return (file_shibboleth in f.readlines()[-1])

#def clear_output_files():
  #for f in glob.glob(args.output_file_prefix + "*" + args.output_file_postfix):
    #os.remove(f)

#def clear_timing_files():
  #for f in glob.glob(our_timing_files):
    #os.remove(f)

#def get_complete_file():
  #file_lst = [str(e) for e in list(glob.glob(our_timing_files))]
  #file_lst.sort(key = lambda f: int(f.replace('iteration_{}_'.format(timing_file_custom_infix), '').replace('.timing', '')))
  #if len(file_lst) <= 0:
    #print("No timing files!")
    #return None
  
  #complete_file = file_lst[-1]
  #if not is_file_complete(complete_file):
    #if len(file_lst) == 1:
      ## Only known file is incomplete
      #print("Only file incomplete")
      #return None
    #else:
      #complete_file = file_lst[-2]
      #assert(is_file_complete(complete_file))
  #return complete_file
    

#def run_xstar(input_file, timeout):
  #global current_proc
  #output_file = "delete_me.out"
  #cmd = "timeout {} release/xstar -i {} -o {} -t {}".format(timeout, input_file, output_file, timing_file_custom_infix)
  #current_proc = subprocess.Popen(shlex.split(cmd))
  #current_proc.wait()
  #if current_proc.returncode != 0:
    #print("X* timeout")
  #current_proc = None
  #complete_file = get_complete_file()
  #if os.path.isfile(output_file):
    #os.remove(output_file)
  #return complete_file

#def save_complete_file(complete_file, idx):
  #dest_name = args.output_file_prefix + str(idx) + args.output_file_postfix
  #if complete_file is None:
    ## Write empty file
    #open(dest_name, 'w').close()
    #return
  #copyfile(complete_file, dest_name)


#def run():
  #clear_output_files();
  #for idx in range(args.iterations):
    #clear_timing_files()
    #complete_file = run_xstar(args.input_file, args.timeout)
    #save_complete_file(complete_file, idx)
    #clear_timing_files()
    
#run()


    
    
    
