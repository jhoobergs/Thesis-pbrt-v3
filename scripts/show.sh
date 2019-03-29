#!/bin/bash
NAME=$1
PBRT_PATH=$BASE"/build/"
SCENE_PATH=$BASE/"scenes/"$NAME
RESULTS_DIR=$BASE"/results/"$NAME
NR=`ls -vl $RESULTS_DIR | sed 's/\s\s*/ /g' | cut -d' ' -f9 | tail -n 1`
RESULTS_DIR=$RESULTS_DIR"/"$NR
FILE_NAME='output.png'
echo $RESULTS_DIR


for VAR in 1 2 3 4 5 6 7 8 9 10
do
echo prim kd bvh bsp or leaf
read ty 
echo $ty
if [ "$ty" == "prim" ]
then
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-primitiveIntersections.txt" #& 
fi
if [ "$ty" == "primp" ]
then
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-primitiveIntersectionsP.txt" #&
fi
if [ "$ty" == "kd" ]
then
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-kdTreeNodeTraversals.txt" #&
fi
if [ "$ty" == "kdp" ]
then
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-kdTreeNodeTraversalsP.txt" #&
fi
if [ "$ty" == "bvh" ]
then
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-bvhTreeNodeTraversals.txt" #&
fi
if [ "$ty" == "bvhp" ]
then
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-bvhTreeNodeTraversalsP.txt" #&
fi
if [ "$ty" == "bsp" ]
then
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-bspTreeNodeTraversals.txt" #&
fi
if [ "$ty" == "bspp" ]
then
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-bspTreeNodeTraversalsP.txt" #&
fi
if [ "$ty" == "leaf" ]
then
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-leafNodeTraversals.txt" #&
fi
if [ "$ty" == "leafp" ]
then
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-leafNodeTraversalsP.txt" #&
fi
done
