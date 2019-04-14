#!/bin/bash
ALGOS=(kdtree rbspkd-13 rbsp-13 bsppaperkd bspclusterfastkd-5 bsprandomfastkd-5 bsparbitraryfastkd-5 bsppaper)

for((K=1;K<=10;K+=1));
do
  echo $K
  for algo in ${ALGOS[*]}
  do
    $(dirname "$0")/run_simple.sh test_museum_local $algo
  done
done
