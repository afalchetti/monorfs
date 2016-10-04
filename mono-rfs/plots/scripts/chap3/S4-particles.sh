#!/bin/bash

outdir="$1"
suboutdir="${outdir}/4-particles"
assetdir="$2"
solverdir="$3"
solver="mono-rfs.exe"
plotter="postanalysis.exe"
scriptdir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

mkdir -p "${suboutdir}"

particlecount0="20"
particlecount="100 800 2000"

outprefix="${suboutdir}/p"
cfgfile="${scriptdir}/default.cfg"

pushd "${solverdir}" >/dev/null

echo "outputting to ${suboutdir}"
echo "from ${solverdir}"

echo "Solving with PHD, ${particlecount0} particles"
/opt/monodevelop/bin/mono "${solver}" -x -i=simulation -f="${assetdir}/sim.world" -c="${assetdir}/movsim.in" -a=phd -p=${particlecount0} -g="${cfgfile}" -r="${outprefix}${particlecount0}.zip"
/opt/monodevelop/bin/mono "${plotter}" -f="${outprefix}${particlecount0}.zip" -c=1 -p=1 -H='timed'

for p in ${particlecount}; do
	echo "Solving with PHD, ${p} particles"
	/opt/monodevelop/bin/mono "${solver}" -x -i=record -f="${outprefix}${particlecount0}.zip" -a=phd -p=${p} -g="${cfgfile}" -r="${outprefix}${p}.zip"
done

for p in ${particlecount}; do
	/opt/monodevelop/bin/mono "${plotter}" -f="${outprefix}${p}.zip" -c=1 -p=1 -H='timed'
done

echo ""
echo "Solving with Odometry"
/opt/monodevelop/bin/mono "${solver}" -x -i=record -f="${outprefix}${particlecount0}.zip" -a=odometry -g="${cfgfile}" -r="${suboutdir}/odometry.zip"
/opt/monodevelop/bin/mono "${plotter}" -f="${suboutdir}/odometry.zip" -c=1 -p=1 -H='timed'

popd >/dev/null
