#!/bin/bash

outdir="${1:-plots/output}"
assetdir="${2:-assets}"
solverdir="${3:-../..}"

solverdir=$( realpath "${solverdir}" )
assetdir="${solverdir}/${assetdir}"
outdir="${solverdir}/${outdir}"

mkdir -p "${outdir}"

echo "output folder: ${outdir}"
echo "assets folder: ${assetdir}"
echo "solver folder: ${solverdir}"
echo ""

trap ' ' INT

for script in chap3/S*.sh; do
	echo "executing ${script}"
	$script "${outdir}/chap3" "${assetdir}" "${solverdir}"
done

for script in chap4/S*.sh; do
	echo "executing ${script}"
	$script "${outdir}/chap4" "${assetdir}" "${solverdir}"
done
