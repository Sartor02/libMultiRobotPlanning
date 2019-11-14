import subprocess
import signal
import shlex
import sys
import os
import glob
import argparse
current_proc = None

def run_with_current_proc(cmd):
    global current_proc
    FNULL = open(os.devnull, 'w')
    current_proc = subprocess.Popen(shlex.split(cmd), stdout=FNULL, stderr=subprocess.STDOUT)
    current_proc.wait()
    retcode = current_proc.returncode
    current_proc = None
    return retcode