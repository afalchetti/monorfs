#!/bin/bash

root="${1:-../output}"

find "${root}" -type d -links 2 | while read dir; do
	echo "plotting ${dir}"
	python topdf.py "${dir}"
done
