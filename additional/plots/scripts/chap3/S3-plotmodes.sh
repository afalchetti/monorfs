#!/bin/bash

outdir="$1"
suboutdir="${outdir}/3-plotmodes"
assetdir="$2"
solverdir="$3"
solver="mono-rfs.exe"
plotter="postanalysis.exe"
scriptdir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

mkdir -p "${suboutdir}"

particlecount=800

phdfile="${suboutdir}/phd.zip"
cfgfile="${scriptdir}/default.cfg"

pushd "${solverdir}" >/dev/null

echo "outputting to ${suboutdir}"
echo "from ${solverdir}"

echo "Solving with PHD, different plot modes"
/opt/monodevelop/bin/mono "${solver}" -x -i=simulation -f="${assetdir}/sim.world" -c="${assetdir}/movsim.in" -a=phd -p=${particlecount} -g="${cfgfile}" -r="${phdfile}"

/opt/monodevelop/bin/mono "${plotter}" -f="${phdfile}" -c=1 -p=2 -H='filter'
mv "${phdfile}.loc.data" "${phdfile}.filter.loc.data"
mv "${phdfile}.rot.data" "${phdfile}.filter.rot.data"
mv "${phdfile}.map.data" "${phdfile}.filter.map.data"

/opt/monodevelop/bin/mono "${plotter}" -f="${phdfile}" -c=1 -p=2 -H='smooth'
mv "${phdfile}.loc.data" "${phdfile}.smooth.loc.data"
mv "${phdfile}.rot.data" "${phdfile}.smooth.rot.data"
mv "${phdfile}.map.data" "${phdfile}.smooth.map.data"

/opt/monodevelop/bin/mono "${plotter}" -f="${phdfile}" -c=1 -p=2 -H='timed'
mv "${phdfile}.loc.data" "${phdfile}.timed.loc.data"
mv "${phdfile}.rot.data" "${phdfile}.timed.rot.data"
mv "${phdfile}.map.data" "${phdfile}.timed.map.data"

popd >/dev/null
