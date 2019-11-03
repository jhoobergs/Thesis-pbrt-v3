#!/bin/bash
ALGOS=(bspclusterfastkd-5 bsprandomfastkd-5 bsparbitraryfastkd-5 bsppaper kdtree rbspkd-13 rbsp-13 bsppaperkd bspclusterfastkd-10 bsprandomfastkd-10 bsparbitraryfastkd-10)

for((K=7;K<=7;K+=1));
do
  echo $K
  for algo in ${ALGOS[*]}
  do
    $(dirname "$0")/run_simple.sh test_museum_local $algo
    rm /tmp/pbrt.JHO*
  done
done
