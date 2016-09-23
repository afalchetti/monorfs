#!/bin/bash

outdir="$1"
suboutdir="${outdir}/9-realsensor"
assetdir="$2"
solverdir="$3"
solver="mono-rfs.exe"
plotter="postanalysis.exe"
scriptdir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

mkdir -p "${suboutdir}"

particlecount=500

phdfile="${suboutdir}/phd.zip"
odofile="${suboutdir}/odometry.zip"
isamfile="${suboutdir}/isam2.zip"
cfgfile="${scriptdir}/kinect.cfg"

pushd "${solverdir}" >/dev/null

echo "outputting to ${suboutdir}"
echo "from ${solverdir}"

echo "Solving with PHD in real sensor"
/opt/monodevelop/bin/mono "${solver}" -x -i=kinect -f="${assetdir}/room.oni" -c="${assetdir}/movroom.in" -a=phd -p=${particlecount} -g="${cfgfile}" -r="${phdfile}"

echo ""
echo "Solving with Odometry in real sensor"
/opt/monodevelop/bin/mono "${solver}" -x -i=record -f="${phdfile}" -a=odometry -g="${cfgfile}" -r="${odofile}"

echo ""
echo "Solving with iSAM2 in real sensor"
/opt/monodevelop/bin/mono "${solver}" -x -i=record -f="${phdfile}" -a=isam2 -g="${cfgfile}" -r="${isamfile}"

wait
/opt/monodevelop/bin/mono "${plotter}" -f="${phdfile}" -c=1 -p=1 -H='timed'
/opt/monodevelop/bin/mono "${plotter}" -f="${odofile}" -c=1 -p=1 -H='timed'
/opt/monodevelop/bin/mono "${plotter}" -f="${isamfile}" -c=1 -p=1 -H='timed'

popd >/dev/null
