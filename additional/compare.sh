#!/bin/bash
n="${1:-3}"

./batch.sh phd $n -f=map.world -c=movroom.in -p=200 -a=phd
./batch.sh isam2 $n -f=map.world -c=movroom.in -a=isam2

./batch.sh phd-noise1 $n -f=map.world -c=movroom.in -p=200 -a=phd -g=noise1.cfg
./batch.sh isam2-noise1 $n -f=map.world -c=movroom.in -a=isam2 -g=noise1.cfg

./batch.sh phd-noise2 $n -f=map.world -c=movroom.in -p=200 -a=phd -g=noise2.cfg
./batch.sh isam2-noise2 $n -f=map.world -c=movroom.in -a=isam2 -g=noise2.cfg

./batch.sh phd-clutter1 $n -f=map.world -c=movroom.in -p=200 -a=phd -g=clutter1.cfg
./batch.sh isam2-clutter1 $n -f=map.world -c=movroom.in -a=isam2 -g=clutter1.cfg

./batch.sh phd-clutter2 $n -f=map.world -c=movroom.in -p=200 -a=phd -g=clutter2.cfg
./batch.sh isam2-clutter2 $n -f=map.world -c=movroom.in -a=isam2 -g=clutter2.cfg
