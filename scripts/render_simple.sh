#!/bin/bash
export NAME=$1
IFS='-' read -ra ACCINFO <<< "$2"

export ACC=${ACCINFO[0]}
export ACCNR=${ACCINFO[1]}
echo $ACCNR
export BASE="/Programming/Thesis/pbrt-v3"
export PBRT_PATH=$BASE"/"$BUILD_FOLDER"/"
export SCENE_PATH=$BASE/"scenes/"$NAME
export RESULTS_DIR=$BASE"/results/"$NAME
mkdir $RESULTS_DIR > /dev/null 2>&1 || true
export NR=`ls -vl $RESULTS_DIR | sed 's/\s\s*/ /g' | cut -d' ' -f9 | cut -d'-' -f1 | tail -n 1`
export RESULTS_DIR=$RESULTS_DIR"/"$(($NR + 1))"-"$ACC
export FILE_NAME='output.png'
echo $RESULTS_DIR

mkdir -p $RESULTS_DIR
cp $SCENE_PATH $RESULTS_DIR"/input.pbrt"
sed -i 's/$acc/"'$ACC'"/' $RESULTS_DIR"/input.pbrt"
sed -i 's/$accnr/'$ACCNR'/' $RESULTS_DIR"/input.pbrt"
#sed -i 's/Include "/Include "'$BASE'/scenes/' $RESULTS_DIR"/input.pbrt"
$PBRT_PATH"pbrt" --outfile $RESULTS_DIR"/"$FILE_NAME $RESULTS_DIR"/input.pbrt" 2>&1 | tee $RESULTS_DIR/"renderOutput"
tail --lines=+$(cat $RESULTS_DIR/"renderOutput" | grep -n "Statistics" | cut -f1 -d:) $RESULTS_DIR/"renderOutput" > $RESULTS_DIR/"statistics"
