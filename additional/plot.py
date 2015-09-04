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
	
	vehicle    = tuple(float(n) for n in dictionary["pose"][0].split())
	landmarks  = [np.array(tuple(float(n) for n in line.split())) for line in dictionary["landmarks"]]
	
	return {"vehicle": {"pos": np.array((vehicle[0], vehicle[1], vehicle[2])),
	                    "theta": vehicle[3],
	                    "axis": np.array((vehicle[4], vehicle[5], vehicle[6]))},
	        "map": landmarks}

def readtrajectory(descriptor):
	descriptor = normalizelinefeeds(descriptor)
	points     = (tuple(float(n) for n in point.split()) for point in descriptor.split("\n") if point)
	
	return [{"time": p[0], "pos": np.array((p[1], p[2], p[3]))} for p in points]

# histfilter -> for every time, use the best estimate at that time, not at the end
def readfinaltrajectory(descriptor, histfilter):
	if histfilter:
		history  = descriptor.split("\n|\n")
		filtered = ""
		
		for i in xrange(len(history)):
			filtered = filtered + history[i].rsplit("\n", 1)[1] + "\n"
		
		return readtrajectory(filtered)
	else:
		# get last block and remove first line (timestamp)
		return readtrajectory(descriptor.rsplit("\n|\n", 1)[1].split("\n", 1)[1])

def readmaps(descriptor):
	descriptor  = normalizelinefeeds(descriptor)
	descriptors = (block.split("\n", 1) for block in descriptor.split("\n|\n") if block)
	
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
	
	error = map(errorentry, [(real, estimate[i], c, p) for i in xrange(len(estimate))])
	
	return (time, error)

def errorentry(entry):
	return mapdistance(*entry)

def mapdistance(a, b, c, p):
	
	if len(a) > len(b):
		temp = a
		a    = b
		b    = temp

	if len(a) == 0:
		return c**p
	
	distances = [[landmarkdistance(ai, bk, c)**p for ai in a] + [c**p for i in xrange(len(b) - len(a))] for bk in b]
	indices   = munkres.Munkres().compute(distances)
	
	return sum(distances[i][k] for i, k in indices) / max(len(a), len(b))

def landmarkdistance(x, y, c):
	return min(c, np.linalg.norm(x - y))

def addgraphs(a, b):
	(ta, fa) = a
	(tb, fb) = b
	
	t = ta if (max(ta) > max(tb)) else tb

	faproj = np.interp(t, ta, fa, right=float("inf"))
	fbproj = np.interp(t, tb, fb, right=float("inf"))
	
	fadd   = [faproj[i] + fbproj[i] for i in xrange(len(t))]

	return (t, fadd)

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

def processdir(directory, histfilter = False, verbose = False):
	if verbose:
		print "  -- reading scene"
		
	with open(os.path.join(directory, "scene.world")) as sfile:
		scene = readscene(sfile.read())
	
	if verbose:
		print "  -- reading trajectory"
		
	with open(os.path.join(directory, "trajectory.out")) as tfile:
		trajectory = readtrajectory(tfile.read())
	
	if verbose:
		print "  -- reading estimate"
		
	with open(os.path.join(directory, "estimate.out")) as efile:
		estimate   = readfinaltrajectory(efile.read(), histfilter)
	
	if verbose:
		print "  -- reading maps"
		
	with open(os.path.join(directory, "maps.out")) as mfile:
		maps = readmaps(mfile.read())
	
	if verbose:
		print "  -- calculating pose error"
		
	perror = poseerror(trajectory, estimate)
	
	if verbose:
		print "  -- calculating map error"
		
	merror = maperror(scene["map"], maps)

	return (perror, merror)

def processfile(datafile, histfilter = False, verbose = False):
	tmp     = tempfile.mkdtemp()
	datadir = os.path.join(tmp, "data")

	os.mkdir(datadir)

	with zipfile.ZipFile(datafile, "r") as datazip:
		datazip.extractall(datadir)

	print "processing", datafile
	(perror, merror) = processdir(datadir, histfilter, verbose)

	shutil.rmtree(tmp)
	
	np.savetxt(datafile + ".pose.data", perror)
	np.savetxt(datafile  + ".map.data", merror)

	return (perror, merror)

def processfile2(args):
	return processfile(*args)
	
def parsearg(argument):
	parts = argument.split('=', 1)
	if len(parts) < 2:
		return [parts[0].lstrip('-'), ""]
	else:
		return [parts[0].lstrip('-'), parts[1].strip('\'"')]

def print_usage():
	print "usage: python plot.py [-history=('filter'|'smooth')] pose.pdf map.pdf data1.zip [data2.zip [data3.zip ...]]"

def main():
	if len(sys.argv) < 3:
		print_usage()
		return
		
	arg1 = parsearg(sys.argv[1])
	
	if arg1[0] == "history":
		if len(sys.argv) < 4:
			print_usage()
			return
		
		posefile   = sys.argv[2]
		mapfile    = sys.argv[3]
		offset     = 4
		histfilter = (arg1[1] == "filter")
	else:
		posefile = sys.argv[1]
		mapfile  = sys.argv[2]
		offset   = 3
		histfilter = False
	
	verbose = False
	
	pool   = multiprocessing.Pool()
	errors = pool.map(processfile2, [(f, histfilter, verbose) for f in sys.argv[offset:]])
	n      = len(errors)
	pool.close()
	pool.join()
	
	(totalp, totalm) = errors[0]
	
	for i in xrange(1, len(errors)):
		perror, merror = errors[i]
		totalp = addgraphs(totalp, perror)
		totalm = addgraphs(totalm, merror)
	
	totalp = (np.array(totalp[0]), np.array(totalp[1]) / n)
	totalm = (np.array(totalm[0]), np.array(totalm[1]) / n)
	
	print "plotting"
	plot(totalp, totalm, posefile, mapfile)
	
	np.savetxt(posefile + ".data", totalp)
	np.savetxt(mapfile  + ".data", totalm)

if __name__ == '__main__':
	main()
