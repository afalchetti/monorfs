#!/bin/bash

from="$1"
to="$2"

outdir="${3:-../output}"

for i in $( seq $from $to ); do
	echo "Run #${i}"
	echo "==================="
	.runall.sh "${outdir}/${i}"
	echo ""
done
