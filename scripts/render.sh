#!/bin/sh
BASE="/Programming/Thesis/pbrt-v3"
PBRT_PATH=$BASE"/build/"
SCENE_PATH=$BASE/"scenes/"$1".pbrt"
DATE=`date '+%Y-%m-%d-%H:%M:%S'`
RESULTS_DIR=$BASE"/results/"$1"/"$DATE"/"
FILE_NAME='output.png'

mkdir -p $RESULTS_DIR
cp $SCENE_PATH $RESULTS_DIR"input.pbrt"
$PBRT_PATH"pbrt" --outfile $RESULTS_DIR$FILE_NAME $SCENE_PATH 2>&1 | tee $RESULTS_DIR/"renderOutput"
tail --lines=+$(cat $RESULTS_DIR/"renderOutput" | grep -n "Statistics" | cut -f1 -d:) $RESULTS_DIR/"renderOutput" > $RESULTS_DIR/"statistics"
xdg-open $RESULTS_DIR$FILE_NAME
