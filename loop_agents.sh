#!/bin/bash
agents=("10" "20" "30" "40" "50" "60" "70")
for i in "${agents[@]}"; do
    ./xstarstarted.sh;
    echo "Agents: $i"
    ./experiments_compare_runtime_data.py "$i" 100 100 0.1 30 1200 60;
    ./xstardone.sh;
done
