#!/bin/sh
PBRT_PATH="../build/"
SCENE_PATH="../scenes/"
RESULTS_PATH="../results/"
echo $1
$PBRT_PATH"pbrt" --outfile $RESULTS_PATH$1".png" $SCENE_PATH$1".pbrt"
