for((type=4;type<=4;type+=1));
do
  for ((amount=1;amount<=3;amount+=1));
  do
     AXISSELECTIONTYPE=$type AXISSELECTIONAMOUNT=$amount $(dirname "$0")/run_simple.sh killeroo-test3 rbsp-3
  done
  for ((amount=1;amount<=7;amount+=1));
  do
     AXISSELECTIONTYPE=$type AXISSELECTIONAMOUNT=$amount $(dirname "$0")/run_simple.sh killeroo-test3 rbsp-7
  done
  for ((amount=1;amount<=9;amount+=1));
  do
     AXISSELECTIONTYPE=$type AXISSELECTIONAMOUNT=$amount $(dirname "$0")/run_simple.sh killeroo-test3 rbsp-9
  done
  for ((amount=1;amount<=13;amount+=1));
  do
     AXISSELECTIONTYPE=$type AXISSELECTIONAMOUNT=$amount $(dirname "$0")/run_simple.sh killeroo-test3 rbsp-13
  done
done
