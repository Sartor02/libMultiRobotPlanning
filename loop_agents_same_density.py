#!/usr/bin/env python3
import subprocess
import math

agents = 20
wh = 100
for i in range(5):
    agents = agents * 2
    wh =  wh * 1.414213562
    #./experiments_compare_runtime_data.py "$i" 100 100 0.1 30 1200 60;
    #./xstardone.sh;
    subprocess.run("./xstarstarted.sh; echo 'Agents: {0} WH: {1}'; ./experiments_compare_runtime_data.py {0} {1} {1} 0.1 30 1200 60; ./xstardone.sh;".format(agents, int(math.ceil(wh))), shell=True)
