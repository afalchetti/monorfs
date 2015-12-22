#!/bin/bash

outdir="${1:-../output}"
assetdir="${2:-../../assets}"
solver="${3:-../../mono-rfs.exe}"

mkdir -p "${outdir}"

echo "output folder: ${outdir}"
echo "assets folder: ${assetdir}"
echo "solver executable: ${solver}"
echo ""

for script in {chap3,chap4}/*.sh; do
	echo "executing ${script}"
	$script "${outdir}" "${assetdir}"  "${solver}"
done
