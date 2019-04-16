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

def get_xstar_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_file", type=str, help="Input file for X*")
    parser.add_argument("timeout", type=float, help="X* timeout")
    parser.add_argument("trials", type=int, help="X* trials")
    parser.add_argument("memory_limit", type=int, help="Memory limit (GB)")
    parser.add_argument("output_file_prefix", type=str, help="Output file prefix")
    parser.add_argument("output_file_postfix", type=str, help="Output file postfix")
    parser.add_argument("--binary", type=str, default=None, help="Output file postfix")
    return parser.parse_args()

args = get_xstar_args();
timing_file_custom_infix = str(hashlib.md5(open(args.input_file,'rb').read()).hexdigest())
our_timing_files = "iteration_{}_*.timing".format(timing_file_custom_infix)

current_proc = None

def signal_handler(sig, frame):
  print(sys.argv[0], 'Signal caught')
  if current_proc is not None:
    current_proc.terminate()
  clear_timing_files()
  sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def is_file_complete(file_name):
  file_shibboleth = "Complete!"
  f = open(file_name, 'r')
  return (file_shibboleth in f.readlines()[-1])

def clear_output_files():
  for f in glob.glob(args.output_file_prefix + "*" + args.output_file_postfix):
    os.remove(f)

def clear_timing_files():
  for f in glob.glob(our_timing_files):
    os.remove(f)

def get_complete_file():
  file_lst = [str(e) for e in list(glob.glob(our_timing_files))]
  file_lst.sort(key = lambda f: int(f.replace('iteration_{}_'.format(timing_file_custom_infix), '').replace('.timing', '')))
  if len(file_lst) <= 0:
    print("No timing files!")
    return None
  
  complete_file = file_lst[-1]
  if not is_file_complete(complete_file):
    if len(file_lst) == 1:
      # Only known file is incomplete
      print("Only file incomplete")
      return None
    else:
      complete_file = file_lst[-2]
      assert(is_file_complete(complete_file))
  return complete_file

def get_memory_limit_kb(limit_gb):
    limit_gb = int(limit_gb)
    limit_mb = limit_gb * 1024
    limit_kb = limit_mb * 1024
    return limit_kb

def run_xstar(input_file, timeout, memory_limit, binary):
  global current_proc
  output_file = "delete_me.out"
  cmd = "bash -c '(ulimit -v {}; timeout {} {} -i {} -o {} -t {})'".format(get_memory_limit_kb(memory_limit),
                                                                           timeout,
                                                                           binary,
                                                                           input_file,
                                                                           output_file,
                                                                           timing_file_custom_infix)
  current_proc = subprocess.Popen(shlex.split(cmd))
  current_proc.wait()
  if current_proc.returncode != 0:
    print("X* timeout")
  current_proc = None
  complete_file = get_complete_file()
  if os.path.isfile(output_file):
    os.remove(output_file)
  return complete_file

def save_complete_file(complete_file, idx):
  dest_name = args.output_file_prefix + str(idx) + args.output_file_postfix
  if complete_file is None:
    # Write empty file
    open(dest_name, 'w').close()
    return
  copyfile(complete_file, dest_name)


def run():
  clear_output_files();
  for idx in range(args.trials):
    clear_timing_files()
    binary = "release/xstar" if args.binary is None else args.binary
    complete_file = run_xstar(args.input_file, args.timeout, args.memory_limit, binary)
    save_complete_file(complete_file, idx)
    clear_timing_files()
    
run()


    
    
    
