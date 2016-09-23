#!/bin/bash

outdir="$1"
suboutdir="${outdir}/8-sandwich"
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
isamknownfile="${suboutdir}/isam2known.zip"
cfgfile="${scriptdir}/default.cfg"
cfgknownfile="${scriptdir}/known.cfg"


pushd "${solverdir}" >/dev/null

echo "outputting to ${suboutdir}"
echo "from ${solverdir}"

echo "Solving with iSAM2, known association"
/opt/monodevelop/bin/mono "${solver}" -x -i=simulation -f="${assetdir}/sim.world" -c="${assetdir}/movsim.in" -a=isam2 -g="${cfgknownfile}" -r="${isamknownfile}"

echo ""
echo "Solving with Odometry"
/opt/monodevelop/bin/mono "${solver}" -x -i=record -f="${isamknownfile}" -a=odometry -g="${cfgfile}" -r="${odofile}"

echo ""
echo "Solving with PHD"
/opt/monodevelop/bin/mono "${solver}" -x -i=record -f="${isamknownfile}" -a=phd -p=${particlecount} -g="${cfgfile}" -r="${phdfile}"

echo ""
echo "Solving with iSAM2, Mahalanobis association"
/opt/monodevelop/bin/mono "${solver}" -x -i=record -f="${isamknownfile}" -a=isam2 -g="${cfgfile}" -r="${isamfile}"

/opt/monodevelop/bin/mono "${plotter}" -f="${phdfile}" -c=1 -p=1 -H='timed'
/opt/monodevelop/bin/mono "${plotter}" -f="${odofile}" -c=1 -p=1 -H='timed'
/opt/monodevelop/bin/mono "${plotter}" -f="${isamfile}" -c=1 -p=1 -H='timed'
/opt/monodevelop/bin/mono "${plotter}" -f="${isamknownfile}" -c=1 -p=1 -H='timed'

popd >/dev/null
