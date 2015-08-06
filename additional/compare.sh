#!/bin/bash
n="${1:-3}"

./batch.sh phd $n -f=map.world -c=movroom.in -p=20 -a=phd
./batch.sh isam2 $n -f=map.world -c=movroom.in -a=isam2
