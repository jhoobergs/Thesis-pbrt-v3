#!/bin/sh
BASE="/Programming/Thesis/pbrt-v3"
PBRT_PATH=$BASE"/build/"
SCENE_PATH=$BASE/"scenes/"$1".pbrt"
DATE=`date '+%Y-%m-%d-%H:%M:%S'`
RESULTS_DIR=$BASE"/results/"$1"/"$DATE"/"
FILE_NAME='output.png'

mkdir -p $RESULTS_DIR
cp $SCENE_PATH $RESULTS_DIR"input.pbrt"
$PBRT_PATH"pbrt" --outfile $RESULTS_DIR$FILE_NAME $SCENE_PATH
