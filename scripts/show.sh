#!/bin/bash
NAME=$1
BASE="/Programming/Thesis/pbrt-v3"
PBRT_PATH=$BASE"/build/"
SCENE_PATH=$BASE/"scenes/"$NAME
RESULTS_DIR=$BASE"/results/"$NAME
NR=`ls -vl $RESULTS_DIR | sed 's/\s\s*/ /g' | cut -d' ' -f9 | tail -n 1`
RESULTS_DIR=$RESULTS_DIR"/"$NR
FILE_NAME='output.png'
echo $RESULTS_DIR


for VAR in 1 2 3 4 5 6 7 8 9 10
do
echo prim kd bvh or rbsp
read ty 
echo $ty
if [ "$ty" == "prim" ]
then
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-primitiveIntersections.txt" #& 
fi
if [ "$ty" == "kd" ]
then
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-kdTreeNodeTraversals.txt" #&
fi
if [ "$ty" == "bvh" ]
then
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-bvhTreeNodeTraversals.txt" #&
fi
if [ "$ty" == "rbsp" ]
then
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-rBSPTreeNodeTraversals.txt" #&
fi
done
