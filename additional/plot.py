#!/usr/bin/env python

import re
from matplotlib import pyplot as mp
from matplotlib import rcParams
import numpy as np
import munkres
import multiprocessing

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

def readtrajectories(descriptor):
	descriptor     = normalizelinefeeds(descriptor)
	descriptors    = descriptor.split("\n|\n")
	realpoints     = (tuple(float(n) for n in point.split()) for point in descriptors[0].split("\n"))
	estimatepoints = (tuple(float(n) for n in point.split()) for point in descriptors[1].split("\n"))
	
	return { "real":     [{"time": t, "pos": np.array((x, y, z))} for (t, x, y, z) in realpoints],
	         "estimate": [{"time": t, "pos": np.array((x, y, z))} for (t, x, y, z) in estimatepoints] }

def readmaps(descriptor):
	descriptor  = normalizelinefeeds(descriptor)
	descriptors = (block.split("\n", 1) for block in descriptor.split("\n|\n"))
	
	return [tuple((float(head), [parsegaussian(line) for line in tail.split("\n") if line != ""])) for (head, tail) in descriptors]
	
def parsegaussian(descriptor):
	descriptors = descriptor.split("; ")
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

def plottrajectories(real, estimate):
	treal   = [p["time"] for p in real]
	posreal = [p["pos"]  for p in real]
	
	testimate   = [p["time"] for p in estimate]
	posestimate = [p["pos"]  for p in estimate]
	
	time = sorted(set(treal) | set(testimate))
	
	interreal     = [np.array(p) for p in zip(*(np.interp(time, treal, xi) for xi in zip(*posreal)))]
	interestimate = [np.array(p) for p in zip(*(np.interp(time, testimate, xi) for xi in zip(*posestimate)))]
	
	error = [np.linalg.norm(interreal[i] - interestimate[i]) for i in xrange(len(time))]
	
	mp.plot(time, error)

def plotmaps(real, estimate):
	time     = [estmap[0] for estmap in estimate]
	real     = [np.array(p) for p in real]
	estimate = [[gaussian["mean"] for gaussian in estmap[1] if gaussian["weight"] > 0.8] for estmap in estimate]
	c        = 1
	p        = 2
	
	error = multiprocessing.Pool(4).map(errorentry, [(str(i) + " / " + str(len(estimate)), (real, estimate[i], c, p)) for i in xrange(len(estimate))])
	
	mp.plot(time, error)

def errorentry(entry):
	print entry[0]
	return mapdistance(*entry[1])

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

if __name__ == '__main__':
	print "reading scene"
	with open("map.world") as sfile:
		scene = readscene(sfile.read())
	
	print "reading trajectories"
	with open("trajectories.out") as tfile:
		trajectories = readtrajectories(tfile.read())
	
	print "reading maps"
	with open("maps.out") as mfile:
		maps = readmaps(mfile.read())
	
	print "plotting trajectories"
	trajplot = mp.figure(1)
	plottrajectories(trajectories["real"], trajectories["estimate"])
	
	mp.xlabel("Time [s]")
	mp.ylabel("Pose error [m]")
	mp.autoscale(tight=True)
	trajplot.subplots_adjust(bottom=0.15)
	mp.savefig("pose-error.pdf")
	
	print "plotting maps"
	mapplot  = mp.figure(2)
	plotmaps(scene["map"], maps)
	
	mp.xlabel("Time [s]")
	mp.ylabel("OSPA Map error")
	mp.autoscale(tight=True)
	mapplot.subplots_adjust(bottom=0.15)
	mp.savefig("map-error.pdf")