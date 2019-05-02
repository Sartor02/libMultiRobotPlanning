#!/bin/bash
trap ctrl_c INT

function ctrl_c() {
    echo "** Trapped CTRL-C"
    exit -1
}
while true; do
./benchmark_generator.py 3 5 5 0.2 cbs_file afs_map afs_agents
rm afs_agents afs_map
timeout 10 ./release/cbs -i cbs_file -o cbs_result.out
rc=$?;
echo "Ret code: $rc"
if [[ $rc != 0 ]]; then continue $rc; fi
echo "CBS"
cbs_res=$(head -7 cbs_result.out | grep "cost:")
timeout --preserve-status 10 ./build/xstar -i cbs_file -o xstar_result.out -t foo > /dev/null
rc=$?;
if [ $rc -ne 0 -a $rc -ne 124 ]; then
    echo "Binary not clean exit!"
    exit -1
fi
if [[ $rc != 0 ]]; then continue $rc; fi
echo "XSTAR"
xstar_res=$(head -7 xstar_result.out | grep "cost:")

if [ "$xstar_res" == "$cbs_res" ]; then
    echo "Strings are equal"
else
    echo "Strings are not equal"
    exit -1
fi
#exit 0
done
#rm *.timing
#./example/visualize.py cbs_file xstar_result.out
#./example/visualize.py cbs_file xstar_result.out --video out.mp4 --speed 3

