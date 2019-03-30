#!/bin/bash
ALGOS=(kdtree rbsp-3 rbsp-7 rbsp-9 rbsp-13 rbspkd-3 rbspkd-7 rbspkd-9 rbspkd-13 bsppaper bsppaperkd bspcluster-2 bspcluster-3 bspcluster-4 bspcluster-5 bspclusterwithkd-3 bspclusterwithkd-4 bspclusterwithkd-5 bspclusterfastkd-3 bspclusterfastkd-4 bspclusterfastkd-5 bsprandom-2 bsprandom-3 bsprandom-4 bsprandom-5 bsprandomwithkd-3 bsprandomwithkd-4 bsprandomwithkd-5 bsprandomfastkd-3 bsprandomfastkd-4 bsprandomfastkd-5 bsparbitrary-2 bsparbitrary-3 bsparbitrary-4 bsparbitrary-5 bsparbitrarywithkd-3 bsparbitrarywithkd-4 bsparbitrarywithkd-5 bsparbitraryfastkd-3 bsparbitraryfastkd-4 bsparbitraryfastkd-5)

for algo in ${ALGOS[*]}
do
  $(dirname "$0")/run_simple.sh test_feet $algo
done

