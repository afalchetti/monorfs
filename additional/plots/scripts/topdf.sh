#!/bin/bash

root="${1:-../output}"

find "${root}" -type d -links 2 | while read dir; do
	python topdf.py "${dir}"
done
