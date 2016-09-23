#!/bin/bash

outdir="$1"
suboutdir="${outdir}/5-imprecisestatistics"
assetdir="$2"
solverdir="$3"
solver="mono-rfs.exe"
plotter="postanalysis.exe"
scriptdir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

mkdir -p "${suboutdir}"

particlecount=800

cfgfile0="stat1"
cfgfiles="stat2 stat3"
outprefix="${suboutdir}/"

pushd "${solverdir}" >/dev/null

echo "outputting to ${suboutdir}"
echo "from ${solverdir}"

echo "Solving with PHD, ${cfgfile0} configuration"
/opt/monodevelop/bin/mono "${solver}" -x -i=simulation -f="${assetdir}/sim.world" -c="${assetdir}/movsim.in" -a=phd -p=${particlecount} -g="${scriptdir}/${cfgfile0}.cfg" -r="${outprefix}${cfgfile0}.zip"
/opt/monodevelop/bin/mono "${plotter}" -f="${outprefix}${cfgfile0}.zip" -c=1 -p=2 -H='timed'

for cfgfile in ${cfgfiles}; do
	echo "Solving with PHD, ${cfgfile} configuration"
	/opt/monodevelop/bin/mono "${solver}" -x -i=record -f="${outprefix}${cfgfile0}.zip" -a=phd -p=${particlecount} -g="${scriptdir}/${cfgfile}.cfg" -r="${outprefix}${cfgfile}.zip"
done

for cfgfile in ${cfgfiles}; do
	/opt/monodevelop/bin/mono "${plotter}" -f="${outprefix}${cfgfile}.zip" -c=1 -p=1 -H='timed'
done

popd >/dev/null
