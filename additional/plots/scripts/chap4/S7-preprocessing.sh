#!/bin/bash

outdir="$1"
suboutdir="${outdir}/7-preprocessing"
assetdir="$2"
solverdir="$3"
solver="mono-rfs.exe"
plotter="postanalysis.exe"
scriptdir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

mkdir -p "${suboutdir}"

particlecount=800

phdfile="${suboutdir}/phd.zip"
odofile="${suboutdir}/odometry.zip"
isamfile="${suboutdir}/isam2.zip"
isamnofile="${suboutdir}/isam2nopre.zip"
cfgfile="${scriptdir}/default.cfg"
cfgfileno="${scriptdir}/nopreprocessing.cfg"


pushd "${solverdir}" >/dev/null

echo "outputting to ${suboutdir}"
echo "from ${solverdir}"

echo "Solving with PHD"
/opt/monodevelop/bin/mono "${solver}" -x -i=simulation -f="${assetdir}/sim.world" -c="${assetdir}/movsim.in" -a=phd -p=${particlecount} -g="${cfgfile}" -r="${phdfile}"

echo ""
echo "Solving with Odometry"
/opt/monodevelop/bin/mono "${solver}" -x -i=record -f="${phdfile}" -a=odometry -g="${cfgfile}" -r="${odofile}"

echo ""
echo "Solving with iSAM2, with preprocessing"
/opt/monodevelop/bin/mono "${solver}" -x -i=record -f="${phdfile}" -a=isam2 -g="${cfgfile}" -r="${isamfile}"

echo ""
echo "Solving with iSAM2, no preprocessing"
/opt/monodevelop/bin/mono "${solver}" -x -i=record -f="${phdfile}" -a=isam2 -g="${cfgfileno}" -r="${isamnofile}"

/opt/monodevelop/bin/mono "${plotter}" -f="${phdfile}" -c=1 -p=2 -H='timed'
/opt/monodevelop/bin/mono "${plotter}" -f="${odofile}" -c=1 -p=2 -H='timed'
/opt/monodevelop/bin/mono "${plotter}" -f="${isamfile}" -c=1 -p=2 -H='timed'

popd >/dev/null
