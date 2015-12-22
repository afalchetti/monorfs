#!/bin/bash

outdir="$1"
suboutdir="${outdir}/1-phd-odometry"
assetdir="$2"
solver="$3"
scriptdir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

mkdir -p "${suboutdir}"

particlecount=25

phdfile="${suboutdir}/phd.zip"
odometryfile="${suboutdir}/odometry.zip"
cfgfile="${scriptdir}/1-phd-odometry.cfg"

/opt/monodevelop/bin/mono "${solver}" -x -i=simulation -f="${assetdir}/map.world" -c="${assetdir}/movroom.in" -a=phd -p=${particlecount} -g="${cfgfile}" -r="${phdfile}"
/opt/monodevelop/bin/mono "${solver}" -x -i=record -f="${phdfile}" -a=odometry -g="${cfgfile}" -r="${odometryfile}"
