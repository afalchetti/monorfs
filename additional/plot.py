#!/usr/bin/env python
# plot.py
# Post-processing plots
# Part of MonoRFS
#
# Copyright (c) 2015, Angelo Falchetti
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * The names of its contributors may not be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL ANGELO FALCHETTI BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import re
from matplotlib import pyplot as mp
from matplotlib import rcParams
import numpy as np
import munkres
import multiprocessing
import os, sys, shutil, tempfile, zipfile

rcParams['axes.labelsize']  = 9
rcParams['xtick.labelsize'] = 9
rcParams['ytick.labelsize'] = 9
rcParams['legend.fontsize'] = 9
rcParams['axes.titlesize']  = 9
rcParams['axes.labelsize']  = 9
rcParams['font.family']     = 'serif'
rcParams['font.serif']      = ['Computer Modern Roman']
rcParams['text.usetex']     = True
rcParams['figure.figsize']  = 4, 2.5

def readscene(descriptor):
	descriptor = normalizelinefeeds(descriptor)
	dictionary = parsedict(descriptor)
	
	vehicle    = tuple(float(n) for n in dictionary["vehicle"][0].split())
	world      = tuple(float(n) for n in dictionary["world"][0].split())
	landmarks  = [np.array(tuple(float(n) for n in line.split())) for line in dictionary["landmarks"]]
	
	return {"vehicle": {"pos": np.array((vehicle[0], vehicle[1], vehicle[2])),
	                    "theta": vehicle[3],
	                    "axis": np.array((vehicle[4], vehicle[5], vehicle[6]))},
	        "worldclip": {"left": world[0], "right": world[1], "top": world[2], "bottom": world[3]},
	        "map": landmarks}

def readtrajectory(descriptor):
	descriptor = normalizelinefeeds(descriptor)
	points     = (tuple(float(n) for n in point.split()) for point in descriptor.split("\n"))
	
	return [{"time": p[0], "pos": np.array((p[1], p[2], p[3]))} for p in points]

def readfinaltrajectory(descriptor):
	# get last block and remove first line (timestamp)
	return readtrajectory(descriptor.rsplit("\n|\n", 1)[1].split("\n", 1)[1])

def readmaps(descriptor):
	descriptor  = normalizelinefeeds(descriptor)
	descriptors = (block.split("\n", 1) for block in descriptor.split("\n|\n"))
	
	return [tuple((float(head), [parsegaussian(line) for line in tail.split("\n") if line != ""])) for (head, tail) in descriptors]

def parsegaussian(descriptor):
	descriptors = descriptor.split(";")
	weight      = float(descriptor[0])
	mean        = tuple((float(n) for n in descriptors[1].split()))
	cov         = tuple((float(n) for n in descriptors[2].split()))
	
	return {"weight": weight,
	        "mean": np.array(mean),
	        "covariance": np.mat(((cov[0], cov[1], cov[2]), (cov[3], cov[4], cov[5]), (cov[6], cov[7], cov[8])))}

def parsedict(descriptor):
	dictionary = {}
	lines      = descriptor.split("\n")
	key        = ""
	
	empty      = re.compile(r"^\s*$")
	
	for line in lines:
		if not empty.match(line):
			if line[0] != '\t':
				key = line
				dictionary[key] = []
			else:
				dictionary[key].append(line[1:])
	
	return dictionary

def normalizelinefeeds(text):
	return text.replace("\r\n", "\n").replace("\r", "\n")

def poseerror(real, estimate):
	treal   = [p["time"] for p in real]
	posreal = [p["pos"]  for p in real]
	
	testimate   = [p["time"] for p in estimate]
	posestimate = [p["pos"]  for p in estimate]
	
	time = sorted(set(treal) | set(testimate))
	
	interreal     = [np.array(p) for p in zip(*(np.interp(time, treal, xi) for xi in zip(*posreal)))]
	interestimate = [np.array(p) for p in zip(*(np.interp(time, testimate, xi) for xi in zip(*posestimate)))]
	
	error = [np.linalg.norm(interreal[i] - interestimate[i]) for i in xrange(len(time))]
	
	return (time, error)

def maperror(real, estimate):
	time     = [estmap[0] for estmap in estimate]
	real     = [np.array(p) for p in real]
	estimate = [[gaussian["mean"] for gaussian in estmap[1] if gaussian["weight"] > 0.8] for estmap in estimate]
	c        = 1
	p        = 2
	
	error = multiprocessing.Pool(4).map(errorentry, [(real, estimate[i], c, p) for i in xrange(len(estimate))])
	
	return (time, error)

def errorentry(entry):
	return mapdistance(*entry)

def mapdistance(a, b, c, p):
	
	if len(a) > len(b):
		temp = a
		a    = b
		b    = temp
	
	distances = [[landmarkdistance(ai, bk, c)**p for ai in a] + [c**p for i in xrange(len(b) - len(a))] for bk in b]
	indices   = munkres.Munkres().compute(distances)
	
	return sum(distances[i][k] for i, k in indices) / max(len(a), len(b))

def landmarkdistance(x, y, c):
	return min(c, np.linalg.norm(x - y))

def addgraphs(a, b):
	(ta, fa) = a
	(tb, fb) = b

	fbproj = np.interp(ta, tb, fb)
	fadd   = [fa[i] + fb[i] for i in xrange(len(ta))]

	return (ta, fadd)

def plot(poseerror, maperror, posefile, mapfile):	
	pplot = mp.figure(1)
	mp.plot(*poseerror)
	mp.xlabel("Time [s]")
	mp.ylabel("Pose error [m]")
	mp.autoscale(tight=True)
	pplot.subplots_adjust(bottom=0.15)
	mp.savefig(posefile)

	mplot = mp.figure(2)
	mp.plot(*maperror)
	mp.xlabel("Time [s]")
	mp.ylabel("OSPA Map error")
	mp.autoscale(tight=True)
	mplot.subplots_adjust(bottom=0.15)
	mp.savefig(mapfile)

def processdir(directory):
	print "  -- reading scene"
	with open(os.path.join(directory, "scene.world")) as sfile:
		scene = readscene(sfile.read())
	
	print "  -- reading trajectory"
	with open(os.path.join(directory, "trajectory.out")) as tfile:
		trajectory = readtrajectory(tfile.read())
	
	print "  -- reading estimate"
	with open(os.path.join(directory, "estimate.out")) as efile:
		estimate   = readfinaltrajectory(efile.read())
	
	print "  -- reading maps"
	with open(os.path.join(directory, "maps.out")) as mfile:
		maps = readmaps(mfile.read())
	
	print "  -- calculating pose error"
	perror = poseerror(trajectory, estimate)
	
	print "  -- calculating map error"
	merror = maperror(scene["map"], maps)

	return (perror, merror)

def processfile(datafile):
	tmp     = tempfile.mkdtemp()
	datadir = os.path.join(tmp, "data")

	os.mkdir(datadir)

	with zipfile.ZipFile(datafile, "r") as datazip:
		datazip.extractall(datadir)

	print "processing", datafile
	(perror, merror) = processdir(datadir)

	shutil.rmtree(tmp)

	return (perror, merror)

def main():
	if len(sys.argv) < 2:
		print "usage: python plot.py pose.pdf map.pdf data1.zip [data2.zip [data3.zip ...]]"
		return

	n = len(sys.argv) - 3
	
	(totalp, totalm) = processfile(sys.argv[3])
	
	for i in xrange(4, len(sys.argv)):
		(perror, merror) = processfile(sys.argv[i])
		totalp = addgraphs(totalp, perror)
		totalm = addgraphs(totalm, merror)

	totalp = (np.array(totalp[0]), np.array(totalp[1]) / n)
	totalm = (np.array(totalm[0]), np.array(totalm[1]) / n)

	print "plotting"
	plot(totalp, totalm, sys.argv[1], sys.argv[2])

if __name__ == '__main__':
	main()