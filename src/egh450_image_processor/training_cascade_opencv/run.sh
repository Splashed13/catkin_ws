#!/bin/sh

#cd training_cascade_opencv

echo "---"
echo "Preparing workspace"
mkdir pos
mkdir output

echo "---"
echo "Creating samples"
echo "---"

./opencv_createsamples -img target.png -bg bg.txt -info pos/info.txt -num 180 -maxxangle 0.0 -maxyangle 0.0 -maxzangle 0.3 -bgcolor 255 -bgthresh 8 -w 48 -h 48

#Sometimes the previous command crashes, so double check we got 128 results
num_pos_check=$(cat ./pos/info.txt | wc -l)

echo "---"
echo "Samples created: $num_pos_check"

if [ $num_pos_check -ne 180 ]
then
	echo "Error: Failed to create all samples!"
	exit 1
fi

echo "---"
echo "Compositing samples"
echo "---"

./opencv_createsamples -info pos/info.txt -bg neg.txt -vec samples.vec -num 180 -w 48 -h 48

echo "---"
echo "Training Classifier"
echo "---"

./opencv_traincascade -data output -vec samples.vec -bg bg.txt -numPos 180 -numNeg 100 -numStages 20 -precalcValBufSize 1024 -precalcIdxBufSize 1024 -featureType HAAR -minHitRate 0.8 -maxFalseAlarmRate 0.5 -w 48 -h 48
