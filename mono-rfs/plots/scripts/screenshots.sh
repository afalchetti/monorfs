#!/bin/bash

root="${1:-plots/output}"
solverdir="${3:-../..}"
solver="mono-rfs.exe"

pushd "${solverdir}" >/dev/null

find "${root}" -type f -iname '*.zip' | while read file; do
	echo "taking screenshots of ${file}"
	/opt/monodevelop/bin/mono "${solver}" -v -s -r "${file}"
done

popd >/dev/null