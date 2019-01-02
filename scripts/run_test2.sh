for ((j=1;j<=2;j++));
do
  export ALPHATYPE=$j
  for ((i=90;i>0;i-=10));
  do
    export SPLITALPHA=$i
    $(dirname "$0")/run_simple.sh killeroo-test2 kdtree
    #$(dirname "$0")/run_rbsps.sh killeroo-test2
  done
done
export ALPHATYPE=0
for ((i=45;i>0;i-=5));
do
  export SPLITALPHA=$i
  $(dirname "$0")/run_simple.sh killeroo-test2 kdtree
  #$(dirname "$0")/run_rbsps.sh killeroo-test2
done

