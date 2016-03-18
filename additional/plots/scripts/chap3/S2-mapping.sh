#!/bin/bash

outdir="$1"
suboutdir="${outdir}/2-mapping"
assetdir="$2"
solverdir="$3"
solver="mono-rfs.exe"
plotter="postanalysis.exe"
scriptdir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

mkdir -p "${suboutdir}"

particlecount=1

phdfile="${suboutdir}/phd.zip"
cfgfile="${scriptdir}/default.cfg"

pushd "${solverdir}" >/dev/null

echo "outputting to ${suboutdir}"
echo "from ${solverdir}"

echo "Solving with PHD, only mapping"
/opt/monodevelop/bin/mono "${solver}" -x -i=simulation -f="${assetdir}/sim.world" -c="${assetdir}/movsim-map.in" -a=phd -p=${particlecount} -g="${cfgfile}" -r="${phdfile}"

/opt/monodevelop/bin/mono "${plotter}" -f="${phdfile}" -c=1 -p=2 -H='timed'

popd >/dev/null
