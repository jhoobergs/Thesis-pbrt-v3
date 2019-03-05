# Run bspcluster for k from 2 to 40 on killeroo
for((K=41;K<=40;K+=1));
do
     $(dirname "$0")/run_simple.sh killeroo-test5 "bspcluster-"$K
done
$(dirname "$0")/run_rbsps.sh killeroo-test5
