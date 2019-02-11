for((type=0;type<=4;type+=1));
do
  for ((amount=1;amount<=13;amount+=1));
  do
     AXISSELECTIONTYPE=$type AXISSELECTIONAMOUNT=$amount $(dirname "$0")/run_simple.sh killeroo-test4 rbsp-13
  done
done
