#!/bin/bash

outdir="$1"
suboutdir="${outdir}/2-standard"
assetdir="$2"
solverdir="$3"
solver="mono-rfs.exe"
plotter="postanalysis.exe"
scriptdir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

mkdir -p "${suboutdir}"

particlecount=800

phdfile="${suboutdir}/phd.zip"
loopyfile="${suboutdir}/loopy.zip"
odofile="${suboutdir}/odometry.zip"
cfgfile="${scriptdir}/default2d.cfg"


pushd "${solverdir}" >/dev/null

echo "outputting to ${suboutdir}"
echo "from ${solverdir}"

echo "Solving with Odometry"
/opt/monodevelop/bin/mono "${solver}" -x -i=simulation -f="${assetdir}/linear2dcomplex.world" -c="${assetdir}/linear2dcomplex.in" -a=odometry -g="${cfgfile}" -r="${odofile}"

echo ""
echo "Solving with PHD"
/opt/monodevelop/bin/mono "${solver}" -x -i=record -f="${odofile}" -a=phd -p=${particlecount} -g="${cfgfile}" -r="${phdfile}"

echo ""
echo "Solving with Loopy PHD"
/opt/monodevelop/bin/mono "${solver}" -i=record -f="${odofile}" -a=loopy -g="${cfgfile}" -r="${loopyfile}"

/opt/monodevelop/bin/mono "${plotter}" -f="${odofile}" -c=1 -p=1 -H='timed' -t 0.0333
/opt/monodevelop/bin/mono "${plotter}" -f="${phdfile}" -c=1 -p=1 -H='timed' -t 0.0333
/opt/monodevelop/bin/mono "${plotter}" -f="${loopyfile}" -c=1 -p=1 -H='timed' -t 0.0333

popd >/dev/null
