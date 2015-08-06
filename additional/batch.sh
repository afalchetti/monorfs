#!/bin/bash
outdir="$1"
n="$2"
args="${@:3}"

mkdir -p "${outdir}"

echo "Solving SLAM, ${outdir}"
for i in $(seq 1 $n); do
	echo "solving ${i}"
	mono mono-rfs.exe $args -s -x -r=$outdir/$i.zip
done

echo "Aggregating data and plotting"
python plot.py "${outdir}/pose.pdf" "${outdir}/map.pdf" $outdir/*.zip
