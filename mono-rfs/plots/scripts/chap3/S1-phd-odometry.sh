#!/bin/bash

outdir="$1"
suboutdir="${outdir}/1-phd-odometry"
assetdir="$2"
solverdir="$3"
solver="mono-rfs.exe"
plotter="postanalysis.exe"
scriptdir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

mkdir -p "${suboutdir}"

particlecount=800

phdfile="${suboutdir}/phd.zip"
odofile="${suboutdir}/odometry.zip"
cfgfile="${scriptdir}/default.cfg"


pushd "${solverdir}" >/dev/null

echo "outputting to ${suboutdir}"
echo "from ${solverdir}"

echo "Solving with PHD"
/opt/monodevelop/bin/mono "${solver}" -x -i=simulation -f="${assetdir}/sim.world" -c="${assetdir}/movsim.in" -a=phd -p=${particlecount} -g="${cfgfile}" -r="${phdfile}"

echo ""
echo "Solving with Odometry"
/opt/monodevelop/bin/mono "${solver}" -x -i=record -f="${phdfile}" -a=odometry -g="${cfgfile}" -r="${odofile}"

/opt/monodevelop/bin/mono "${plotter}" -f="${phdfile}" -c=1 -p=2 -H='timed'
/opt/monodevelop/bin/mono "${plotter}" -f="${odofile}" -c=1 -p=2 -H='timed'

popd >/dev/null
