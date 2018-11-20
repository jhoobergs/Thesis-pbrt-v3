#!/bin/bash
NAME=$1
BASE="/Programming/Thesis/pbrt-v3"
PBRT_PATH=$BASE"/build/"
SCENE_PATH=$BASE/"scenes/"$NAME
RESULTS_DIR=$BASE"/results/"$NAME
NR=`ls -vl $RESULTS_DIR | cut -d' ' -f9 | tail -n 1`
RESULTS_DIR=$RESULTS_DIR"/"$NR
FILE_NAME='output.png'
echo $RESULTS_DIR

while true
do
echo prim kd bvh or rbsp
read t
echo $t
if [ "$t" == "prim" ]
then
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-primitiveIntersections.txt" &
fi
if [ "$t" == "kd" ]
then
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-kdTreeNodeTraversals.txt" &
fi
if [ "$t" == "bvh" ]
then
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-bvhTreeNodeTraversals.txt" &
fi
if [ "$t" == "rbsp" ]
then
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-rBSPTreeNodeTraversals.txt" &
fi
done
