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
import traceback

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

# utility function to define an enumeration
def enum(*values):
	return type('Enum', (), dict(zip(values, range(len(values)))))

HistMode = enum("Smooth", "Filter", "TimedAverage")

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

def readtrajectoryhistory(descriptor):
	descriptor = normalizelinefeeds(descriptor)
	history = descriptor.split("\n|\n")
	return [readtrajectory(snapshot.split("\n", 1)[1]) for snapshot in history]

# histfilter -> for every time, use the best estimate at that time, not at the end
def readfinaltrajectory(descriptor, histfilter):
	if histfilter:
		descriptor = normalizelinefeeds(descriptor)
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

def readtags(descriptor):
	descriptor = normalizelinefeeds(descriptor)
	tags       = (line.split(" ", 1) for line in descriptor.split("\n") if line)
	
	return [(float(entry[0]), entry[1]) for entry in tags if len(entry) > 1]
	
	
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

def poseerror(real, estimate, histmode = HistMode.Smooth):
	if histmode == HistMode.Smooth:
		return poseerror0(real, estimate[-1])
		
	elif histmode == HistMode.Filter:
		return poseerror0(real, [snapshot[-1] for snapshot in estimate])
		
	elif histmode == HistMode.TimedAverage:
		chopreal = []
		
		k = 0
		for i in xrange(len(estimate)):
			# for each estimate[i], chopreal[i] will be the real array
			# in the window where the estimate exists, i.e. the future
			# tail is removed (so an average error makes sense)
			
			# to do so, compare against the time of the last estimate pose [-1]
			while k < len(real) and real[k]["time"] <= estimate[i][-1]["time"]:
				# as the estimate grows over time, the chop point does as well
				k = k + 1
			
			chopreal.append(real[0:k])
		
		time  = [p["time"] for p in estimate[-1]]
		error = [np.mean(poseerror0(chopreal[i], estimate[i])[1]) for i in xrange(len(estimate))]
		
		return (time, error)
		
	else:
		raise NameError("History mode not recognized")

def poseerror0(real, estimate):
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
	if a is None or b is None:
		return None
	
	(ta, fa) = a
	(tb, fb) = b
	
	t = ta if (max(ta) > max(tb)) else tb
	
	faproj = np.interp(t, ta, fa, right=float("inf"))
	fbproj = np.interp(t, tb, fb, right=float("inf"))
	
	fadd   = [faproj[i] + fbproj[i] for i in xrange(len(t))]
	
	return (t, fadd)

def addtags(a, b):
	if a is None:
		return b
	elif b is None:
		return a
	
	tags = a[:]
	
	# though it has bad complexity, there should be very few tags
	# so this shouldn't be too bad
	for entry in b:
		close = [x[1] for x in a if abs(x[0] - entry[0]) < 1e-5]
		
		if (b[1] not in close):
			tags.append(entry)
	
	return tags

def plot(poseerror, maperror, tags, posefile, mapfile):
	if poseerror is not None:
		pplot = mp.figure(1)
		
		ymax = max(poseerror[1])
		
		if tags is not None:
			for tag in tags:
				color = "#ff2222" if tag[1].startswith("!") else "#bbbbbb"
				mp.axvline(tag[0], color=color, linewidth=0.5)
				mp.annotate(xy=(tag[0], ymax), s=tag[1], color=color, fontsize=5, family="sans-serif",
					xycoords='data', xytext=(1, -2), textcoords='offset points', rotation="vertical", verticalalignment="top")
		
		mp.plot(*poseerror)
		mp.xlabel("Time [s]")
		mp.ylabel("Pose error [m]")
		
		mp.autoscale(tight=True)
		pplot.subplots_adjust(bottom=0.15)
		mp.savefig(posefile)
	
	if maperror is not None:
		mplot = mp.figure(2)
		
		ymax = max(maperror[1])
		
		if tags is not None:
			for tag in tags:
				color = "#ff2222" if tag[1].startswith("!") else "#bbbbbb"
				mp.axvline(tag[0], color=color, linewidth=0.5)
				mp.annotate(xy=(tag[0], ymax), s=tag[1], color=color, fontsize=5, family="sans-serif",
					xycoords='data', xytext=(1, -2), textcoords='offset points', rotation="vertical", verticalalignment="top")
				
		mp.plot(*maperror)
		mp.xlabel("Time [s]")
		mp.ylabel("OSPA Map error")
		
		mp.autoscale(tight=True)
		mplot.subplots_adjust(bottom=0.15)
		mp.savefig(mapfile)

def processdir(directory, histmode = HistMode.Smooth, verbose = False):
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
		
	try:
		with open(os.path.join(directory, "estimate.out")) as efile:
			estimate = readtrajectoryhistory(efile.read())
	except:
		print "     [!] no pose estimate found, skipping pose error calculation"
		estimate = None
	
	if verbose:
		print "  -- reading maps"
		
	try:
		with open(os.path.join(directory, "maps.out")) as mfile:
			maps = readmaps(mfile.read())
	except:
		print "     [!] no map estimate found, skipping map error calculation"
		maps = None
	
	if verbose:
		print "  -- reading tags"
	
	try:
		with open(os.path.join(directory, "tags.out")) as tfile:
			tags = readtags(tfile.read())
	except:
		print "     [!] no tags found, skipping"
		tags = None
	
	perror = None
	merror = None
	
	if estimate != None:
		if verbose:
			print "  -- calculating pose error"
			
		perror = poseerror(trajectory, estimate, histmode)
	
	if maps != None:
		if verbose:
			print "  -- calculating map error"
			
		merror = maperror(scene["map"], maps)

	return (perror, merror, tags)

def processfile(datafile, histmode = HistMode.Smooth, verbose = False):
	tmp     = tempfile.mkdtemp()
	datadir = os.path.join(tmp, "data")

	os.mkdir(datadir)

	with zipfile.ZipFile(datafile, "r") as datazip:
		datazip.extractall(datadir)

	print "processing", datafile
	(perror, merror, tags) = processdir(datadir, histmode, verbose)

	shutil.rmtree(tmp)
	
	if perror is not None:
		np.savetxt(datafile + ".pose.data", perror)
	
	if merror is not None:
		np.savetxt(datafile  + ".map.data", merror)

	return (perror, merror, tags)

def processfile2(args):
	try:
		return processfile(*args)
	except:
		traceback.print_exc(file=sys.stdout)
	
def parsearg(argument):
	parts = argument.split('=', 1)
	if len(parts) < 2:
		return [parts[0].lstrip('-'), ""]
	else:
		return [parts[0].lstrip('-'), parts[1].strip('\'"')]

def print_usage():
	print "usage: python plot.py [-history=('filter'|'smooth'|'timed')] pose.pdf map.pdf data1.zip [data2.zip [data3.zip ...]]"

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
		histmode = (HistMode.Filter       if (arg1[1] == "filter")
		       else HistMode.TimedAverage if (arg1[1] == "timed")
		       else HistMode.Smooth)
	else:
		posefile = sys.argv[1]
		mapfile  = sys.argv[2]
		offset   = 3
		histmode = HistMode.Smooth
	
	verbose = False
	
	pool   = multiprocessing.Pool()
	errors = pool.map(processfile2, [(f, histmode, verbose) for f in sys.argv[offset:]])
	n      = len(errors)
	pool.close()
	pool.join()
	
	(totalp, totalm, totalt) = errors[0]
	
	for i in xrange(1, len(errors)):
		perror, merror, tags = errors[i]
		totalp = addgraphs(totalp, perror)
		totalm = addgraphs(totalm, merror)
		totalt = addtags(totalt, tags)
	
	if totalp is not None:
		totalp = (np.array(totalp[0]), np.array(totalp[1]) / n)
		np.savetxt(posefile + ".data", totalp)
	
	if totalm is not None:
		totalm = (np.array(totalm[0]), np.array(totalm[1]) / n)
		np.savetxt(mapfile  + ".data", totalm)
	
	print "plotting"
	plot(totalp, totalm, totalt, posefile, mapfile)
	

if __name__ == '__main__':
	main()
