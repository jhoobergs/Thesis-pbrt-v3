for ((i=90;i>=0;i-=10));
do
   export SPLITALPHA=$i
   $(dirname "$0")/run_rbsps.sh killeroo-test1
done
