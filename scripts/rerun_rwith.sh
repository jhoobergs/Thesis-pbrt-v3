#!/bin/bash
ALGOS=(bsprandomwithkd-3 bsprandomwithkd-4 bsprandomwithkd-5 bsprandomwithkd-6 bsprandomwithkd-7 bsprandomwithkd-8 bsprandomwithkd-9 bsprandomwithkd-10)


for((K=1;K<=7;K+=1));
do
  echo $K
  for algo in ${ALGOS[*]}
  do
    $(dirname "$0")/run_simple.sh test_feet_local_k $algo
    $(dirname "$0")/run_simple.sh test_sponza_local_k $algo
    rm /tmp/pbrt.JHO*
  done
done
