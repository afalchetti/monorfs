#!/usr/bin/env python
# topdf.py
# Convert simple dlm data files into pdf plots
# Part of MonoRFS
#
# Copyright (c) 2016, Angelo Falchetti
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

from matplotlib import pyplot as mp
from matplotlib import rcParams
import numpy as np
import math
import os, sys
import re
import bisect

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

labels = {"map":     "OSPA map error [m]",
          "loc":     "Pose location error [m]",
          "rot":     "Pose rotation error [rads]",
          "odoloc":  "Pose delta location error [m]",
          "odorot":  "Pose delta rotation error [rads]",
          "size":    "Number of landmarks",
          "pathlen": "Pose location error [m]"}

def formatlegend(legend):
	legend = legend.replace("phd", "PHD")
	legend = legend.replace("loopy", "Loopy PHD")
	legend = legend.replace("isam", "iSAM")
	legend = legend.replace("nopre", " (no preprocessing)")
	legend = legend.replace("known", " (known association)")
	legend = legend.replace("odometry", "Odometry")
	legend = legend.replace("stat", "PHD, Config ")
	legend = legend.replace("-spatial", " (spatial)")
	legend = re.sub(r"p(?P<count>[0-9]+)", r"PHD, p = \1", legend)
	return legend

def getstyle(legend):
	
	style = "-k"
	
	if "phd" in legend:
		style = "-b"
	elif "loopy" in legend:
		style = "-g"
	elif "odometry" in legend:
		style = "-k"
	elif "isam" in legend:
		style = "-r"
		
		if "nopre" in legend:
			style = "-c"
		elif "known" in legend:
			style = "-g"
	elif "p2000" in legend:
		style = "-b"
	elif "p800" in legend:
		style = "-r"
	elif "p100" in legend:
		style = "-g"
	elif "p20" in legend:
		style = "-c"
	elif "stat1" in legend:
		style = "-b"
	elif "stat2" in legend:
		style = "-r"
	elif "stat3" in legend:
		style = "-g"
	
	if "spatial" in legend:
		style = "-" + style
	
	return style

def readtags(tagfile):
	with open(tagfile) as file:
		descriptor = normalizelinefeeds(file.read())
		tags       = (line.split(" ", 1) for line in descriptor.split("\n") if line)
	
	return [(float(entry[0]), entry[1]) for entry in tags if len(entry) > 1]

def normalizelinefeeds(text):
	return text.replace("\r\n", "\n").replace("\r", "\n")

def addtags(a, b):
	if a is None:
		return b
	elif b is None:
		return a
	
	tags = a[:]
	
	for entry in b:
		close = [x[1] for x in a if abs(x[0] - entry[0]) < 1e-5]
		
		if (entry[1] not in close):
			tags.append(entry)
	
	return tags

def parsefiles(directory):
	filedata = {}
	tagdata  = None
	
	for fn in os.listdir(directory):
		basename = os.path.split(fn)[1]
		name, ext = os.path.splitext(basename)
		if ext == ".data":
			hierarchy = name.replace(".zip", "").split(".")[::-1]
			highlevel = hierarchy[0]
			sublevel  = ""
			legend    = ""
			
			if highlevel == "tags":
				tagdata = addtags(tagdata, readtags(os.path.join(directory, fn)))
				continue
			
			if len(hierarchy) > 2:
				sublevel = hierarchy[1]
				legend   = " - ".join(hierarchy[2:])
			elif len(hierarchy) > 1:
				legend = hierarchy[1]
			
			if not filedata.has_key(highlevel):
				filedata[highlevel] = {}
			
			if not filedata[highlevel].has_key(sublevel):
				filedata[highlevel][sublevel] = []
			
			filedata[highlevel][sublevel].append({"legend": formatlegend(legend), "style": getstyle(legend),
			                                      "data": np.loadtxt(os.path.join(directory, fn))})
	
	return (filedata, tagdata)

def filterdata(filedata):
	filtered = filedata
	
	# remove realsize as an independent plot and add it as a layer in the size plot
	if filtered.has_key("realsize") and len(filtered["realsize"]) > 0:
		if not filtered.has_key("size"):
			filtered["size"] = {"": []}
		
		if not filtered["size"].has_key(""):
			filtered["size"][""] = []
		
		# choose the non-zero realsize if any (all of them are either equal or zero-valued if the groundtruth was not available)
		rsdata = filtered["realsize"].values()[0][0]["data"]
		for realsize in filtered["realsize"].values()[0]:
			if realsize["data"][-1, 1] > 0:
				rsdata = realsize["data"]
		
		filtered["size"][""].append({"legend": "Real size", "style": getstyle("real"), "data": rsdata})
		
		filtered.pop("realsize", None)
		
	# if there is a pathlen file, plot every location error twice:
	# one with the pathlen layer and one without it
	if filtered.has_key("pathlen") and filtered.has_key("loc") and len(filtered["pathlen"]) > 0:
		# choose any of the pathlen files (values()[0][0]), since they should be exactly the same
		pldata = filtered["pathlen"].values()[0][0]["data"]
		
		filtered["pathlen"] = {}
		
		for sublayer in filtered["loc"]:
			filtered["pathlen"][sublayer] = filtered["loc"][sublayer][:]
			filtered["pathlen"][sublayer].append({"legend": "Real trajectory arclength", "style": getstyle("arclength"),
			                                      "data": pldata})
	
	return filtered

def plot(layers, tags, axes, label, outfile):
	pplot = mp.figure()
	
	ymax = axes[3]
	
	online = not any(layer["legend"] == "Loopy PHD" for layer in layers)
	
	if online:
		if tags is not None:
			for tag in tags:
				if tag[1].startswith("screenshot"):  # these tags are not interesting
					continue
				
				color = "#ff2222" if tag[1].startswith("!") else "#bbbbbb"
				mp.axvline(tag[0], color=color, linewidth=0.5)
				mp.annotate(xy=(tag[0], ymax), s=tag[1], color=color, fontsize=5, family="sans-serif",
					xycoords='data', xytext=(1, -2), textcoords='offset points', rotation="vertical", verticalalignment="top")
		
		for layer in layers:
			mp.plot(layer["data"][:, 0], layer["data"][:, 1], layer["style"], label = layer["legend"])
		
		mp.xlabel("Time [s]")
		
	else:
		for layer in layers:
			if "Loopy" not in layer["legend"]:
				finalvalue = layer["data"][-1, 1]
				mp.plot([axes[0], axes[1]], [finalvalue, finalvalue], layer["style"], label = layer["legend"])
			else:
				mp.plot(layer["data"][:, 0], layer["data"][:, 1], layer["style"], label = layer["legend"])
		
		mp.xlabel("Time [iterations]")
	
	mp.ylabel(label)
	mp.axis(axes)
	
	pplot.subplots_adjust(bottom = 0.16)
	
	if len(layers) > 1:
		leg = mp.legend(loc = "best", fancybox=True, framealpha=0.5)
		mp.savefig(outfile, bbox_extra_artists=(leg,), bbox_inches='tight')
	else:
		mp.savefig(outfile)

def plotfiles(filedata, tags, directory):
	for highlevel in filedata:
		ymin = float("inf")
		ymax = float("-inf")
		
		sortedlevels = {}
		
		numregex = re.compile(r"([0-9]+)")
		
		for sublevel in filedata[highlevel]:
			# use a natural sort on the legend, i.e. A = a, a < b, p20 < p100 < p800 < p2000
			sortedlayers = sorted(filedata[highlevel][sublevel], key= lambda x: [int(text) if text.isdigit() else text.lower()
			                                                                         for text in re.split(numregex, x["legend"])])
			sortedlevels[sublevel] = sortedlayers
			for layer in sortedlayers:
				data = layer["data"]
				ymin = min(ymin, min(data[:, 1]))
				ymax = max(ymax, max(data[:, 1]))
		
		ymin = min(ymin, 0)
		
		levels = [1e-4, 2e-4, 5e-4, 1e-3, 2e-3, 3e-3, 5e-3, 1e-2, 2e-2, 3e-2, 5e-2, 1e-1, 2e-1, 3e-1, 5e-1, 1, 2, 3, 5, 10, 20, 30, 50]
		index = bisect.bisect(levels, ymax + 1e-5)
		ymax = levels[index] if index < len(levels) else (50 * (math.floor(ymax / 50) + 1))
		
		for sublevel in sortedlevels:
			label = labels.get(highlevel, "Value")
			
			xmin = float("inf")
			xmax = float("-inf")
			for layer in sortedlevels[sublevel]:
				data = layer["data"]
				xmin = min(xmin, min(data[:, 0]))
				xmax = max(xmax, max(data[:, 0]))
			
			plot(sortedlevels[sublevel], tags, [xmin, xmax, ymin, ymax], label,
			     os.path.join(directory, ".".join(filter(None, (highlevel, sublevel))) + ".pdf"))

def print_usage():
	print "usage: python topdf.py path/to/data/root"

def main():
	if len(sys.argv) != 2:
		print_usage()
		return
		
	processdir = sys.argv[1]
	
	filedata, tags = parsefiles(processdir)
	
	filterdata(filedata)
	
	plotfiles(filedata, tags, processdir)

if __name__ == '__main__':
	main()
