#!/bin/bash
#ALGOS=(bspclusterfastkd-6 bsprandomfastkd-6 bsparbitraryfastkd-6 bspclusterfastkd-7 bsprandomfastkd-7 bsparbitraryfastkd-7 bspclusterfastkd-8 bsprandomfastkd-8 bsparbitraryfastkd-8 bspclusterfastkd-9 bsprandomfastkd-9 bsparbitraryfastkd-9)
#ALGOS=(bspclusterfastkd-3 bsprandomfastkd-3 bsparbitraryfastkd-3 bspclusterwithkd-3 bsprandomwithkd-3 bsparbitrarywithkd-3 bspclusterwithkd-4 bsprandomwithkd-4 bsparbitrarywithkd-4 bspclusterwithkd-5 bsprandomwithkd-5 bsparbitrarywithkd-5 bspclusterwithkd-6 bsprandomwithkd-6 bsparbitrarywithkd-6 bspclusterwithkd-7 bsprandomwithkd-7 bsparbitrarywithkd-7 bspclusterwithkd-8 bsprandomwithkd-8 bsparbitrarywithkd-8 bspclusterwithkd-9 bsprandomwithkd-9 bsparbitrarywithkd-9 bspclusterwithkd-10 bsprandomwithkd-10 bsparbitrarywithkd-10)
ALGOS=(bspcluster-2 bsprandom-2 bsparbitrary-2 bspcluster-3 bsprandom-3 bsparbitrary-3 bspcluster-4 bsprandom-4 bsparbitrary-4 bspcluster-5 bsprandom-5 bsparbitrary-5 bspcluster-6 bsprandom-6 bsparbitrary-6 bspcluster-7 bsprandom-7 bsparbitrary-7 bspcluster-8 bsprandom-8 bsparbitrary-8 bspcluster-9 bsprandom-9 bsparbitrary-9 bspcluster-10 bsprandom-10 bsparbitrary-10)

for((K=1;K<=7;K+=1));
do
  echo $K
  for algo in ${ALGOS[*]}
  do
    $(dirname "$0")/run_simple.sh test_feet_local_k $algo
    rm /tmp/pbrt.JHO*
  done
done
