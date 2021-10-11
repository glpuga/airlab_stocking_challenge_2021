#!/bin/bash

DATAFILE="net/cfg/dataset.data"
CFGFILE="net/cfg/yolov4-airlab.cfg"
WEIGHTSFILE="/home/user/data/yolo/yolov4.conv.137"

LASTWEIGHTS="net/backup/yolov4-airlab_last.weights"

if [[ -e $LASTWEIGHTS ]]; then
    echo "Found a previous session weights file, resuming from there..."
    sleep 10
    WEIGHTSFILE=`echo $LASTWEIGHTS`
fi

# create the list of files based on the ones in the training and evaluation folders

pushd dataset

rm dataset_training.txt
for n in `ls  training/*.png`; do
    echo `pwd`/$n >> dataset_training.txt;
done

rm dataset_evaluation.txt
for n in `ls  evaluation/*.png`; do
    echo `pwd`/$n >> dataset_evaluation.txt;
done

popd

# Do the actual training
darknet detector train $DATAFILE $CFGFILE $WEIGHTSFILE -dont_show -mjpeg_port 8090 -map -iou_thresh 0.4 -clear
