# Run bspcluster for k from 2 to 40 on killeroo
for((K=2;K<=10;K+=1));
do
     $(dirname "$0")/run_simple.sh test_feet_k "bspcluster-"$K
     $(dirname "$0")/run_simple.sh test_feet_k "bspclusterwithkd-"$K
     $(dirname "$0")/run_simple.sh test_feet_k "bspclusterfastkd-"$K
done

