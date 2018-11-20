#!/bin/sh
NAME=$1
ACC=$2
BASE="/Programming/Thesis/pbrt-v3"
PBRT_PATH=$BASE"/build/"
SCENE_PATH=$BASE/"scenes/"$NAME
RESULTS_DIR=$BASE"/results/"$NAME
mkdir $RESULTS_DIR > /dev/null 2>&1 || true
NR=`ls -vl $RESULTS_DIR | cut -d' ' -f9 | cut -d'-' -f1 | tail -n 1`
RESULTS_DIR=$RESULTS_DIR"/"$(($NR + 1))"-"$ACC
FILE_NAME='output.png'
echo $RESULTS_DIR

mkdir -p $RESULTS_DIR
cp $SCENE_PATH $RESULTS_DIR"/input.pbrt"
sed -i 's/$acc/"'$ACC'"/' $RESULTS_DIR"/input.pbrt"
#sed -i 's/Include "/Include "'$BASE'/scenes/' $RESULTS_DIR"/input.pbrt"
$PBRT_PATH"pbrt" --outfile $RESULTS_DIR"/"$FILE_NAME $RESULTS_DIR"/input.pbrt" 2>&1 | tee $RESULTS_DIR/"renderOutput"
tail --lines=+$(cat $RESULTS_DIR/"renderOutput" | grep -n "Statistics" | cut -f1 -d:) $RESULTS_DIR/"renderOutput" > $RESULTS_DIR/"statistics"
diff $SCENE_PATH".png" $RESULTS_DIR/$FILE_NAME
xdg-open $RESULTS_DIR/$FILE_NAME
$BASE"/scripts/show.sh" $NAME
