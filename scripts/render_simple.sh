#!/bin/bash
export NAME=$1
IFS='-' read -ra ACCINFO <<< "$2"

export ACC=${ACCINFO[0]}
export ACCNR=${ACCINFO[1]}
echo $ACCNR
case $HOSTNAME in
  (JHO) export BASE="/Programming/Thesis-pbrt-v3";;
  (*)   export BASE="/home/r0588750/Thesis/pbrt-v3";;
esac

export PBRT_PATH=$BASE"/"$BUILD_FOLDER"/"
export SCENE_PATH=$BASE/"scenes/"$NAME
export RESULTS_DIR=$BASE"/results/"$NAME"/"$HOSTNAME
mkdir -p $RESULTS_DIR > /dev/null 2>&1 || true
export NR=`ls -vl $RESULTS_DIR | sed 's/\s\s*/ /g' | cut -d' ' -f9 | cut -d'-' -f1 | tail -n 1`
export RESULTS_DIR=$RESULTS_DIR"/"$(($NR + 1))"-"$ACC
export FILE_NAME='output.png'
echo $RESULTS_DIR

mkdir -p $RESULTS_DIR
cp $SCENE_PATH $RESULTS_DIR"/input.pbrt"
sed -i 's/$acc/"'$ACC'"/' $RESULTS_DIR"/input.pbrt"
sed -i 's/$accnr/'$ACCNR'/' $RESULTS_DIR"/input.pbrt"
sed -i 's/$splitalpha/'$SPLITALPHA'/' $RESULTS_DIR"/input.pbrt"
sed -i 's/$alphatype/'$ALPHATYPE'/' $RESULTS_DIR"/input.pbrt"
sed -i 's/$axisselectiontype/'$AXISSELECTIONTYPE'/' $RESULTS_DIR"/input.pbrt"
sed -i 's/$axisselectionamount/'$AXISSELECTIONAMOUNT'/' $RESULTS_DIR"/input.pbrt"
#sed -i 's/Include "/Include "'$BASE'/scenes/' $RESULTS_DIR"/input.pbrt"
time $PBRT_PATH"pbrt" --quiet --outfile $RESULTS_DIR"/"$FILE_NAME $RESULTS_DIR"/input.pbrt" 2>&1 | tee $RESULTS_DIR/"renderOutput"
tail --lines=+$(cat $RESULTS_DIR/"renderOutput" | grep -n "Statistics" | cut -f1 -d:) $RESULTS_DIR/"renderOutput" > $RESULTS_DIR/"statistics"
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-primitiveIntersectionsP.txt" $RESULTS_DIR"/heatmap-primitiveIntersectionsP.png"
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-primitiveIntersections.txt" $RESULTS_DIR"/heatmap-primitiveIntersections.png"
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-kdTreeNodeTraversalsP.txt" $RESULTS_DIR"/heatmap-kdTreeNodeTraversalsP.png"
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-kdTreeNodeTraversals.txt" $RESULTS_DIR"/heatmap-kdTreeNodeTraversals.png"
#python ../../plots/heatmap_single.py $RESULTS_DIR"/output-bvhTreeNodeTraversalsP.txt" $RESULTS_DIR"/heatmap-bvhTreeNodeTraversalsP.png"
#python ../../plots/heatmap_single.py $RESULTS_DIR"/output-bvhTreeNodeTraversals.txt" $RESULTS_DIR"/heatmap-bvhTreeNodeTraversals.png"
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-bspTreeNodeTraversalsP.txt" $RESULTS_DIR"/heatmap-bspTreeNodeTraversalsP.png"
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-bspTreeNodeTraversals.txt" $RESULTS_DIR"/heatmap-bspTreeNodeTraversals.png"
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-leafNodeTraversalsP.txt" $RESULTS_DIR"/heatmap-leafNodeTraversalsP.png"
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-leafNodeTraversals.txt" $RESULTS_DIR"/heatmap-leafNodeTraversals.png"
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-leafNodeIntersectionPMeanAmount.txt" $RESULTS_DIR"/heatmap-leafNodeIntersectionPMeanAmount.png"
python ../../plots/heatmap_single.py $RESULTS_DIR"/output-leafNodeIntersectionMeanAmount.txt" $RESULTS_DIR"/heatmap-leafNodeIntersectionMeanAmount.png"

tar --transform 's/.*\///g' -czvf $RESULTS_DIR".tar.gz" $RESULTS_DIR
rm -r $RESULTS_DIR
