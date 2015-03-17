#!/usr/bin/env python
# quick groundtruth -> perfect odometry converter

import numpy as np
import sys
from math import sqrt, exp, cos, sin, asin, atan2

import matplotlib.pyplot as plot
from mpl_toolkits.mplot3d import axes3d

def readgroundtruth(filename):
	time = []
	tx   = []
	ty   = []
	tz   = []
	qx   = []
	qy   = []
	qz   = []
	qw   = []
	
	with open(filename) as gtfile:
		for line in gtfile:
			if line[0] == '#':
				continue
			framedata = line.split()
			time.append(float(framedata[0]))
			tx  .append(float(framedata[1]))
			ty  .append(float(framedata[2]))
			tz  .append(float(framedata[3]))
			qx  .append(float(framedata[4]))
			qy  .append(float(framedata[5]))
			qz  .append(float(framedata[6]))
			qw  .append(float(framedata[7]))
	
	return time, tx, ty, tz, qw, qx, qy, qz

def readdepth(filename):
	time = []
	
	with open(filename) as gtfile:
		for line in gtfile:
			if line[0] == '#':
				continue
			framedata = line.split()
			time.append(float(framedata[0]))
	
	return time

def quatmag(q):
	w, x, y, z = q
	return sqrt(w**2 + x**2 + y**2 + z**2)

def quatmul(q, p):
	w1, x1, y1, z1 = q
	w2, x2, y2, z2 = p
	w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
	x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
	y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
	z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
	return (w, x, y, z)

def quatconj(q):
	w, x, y, z = q
	return (w, -x, -y, -z)

def quatconjby(q, p):
	return quatmul(quatmul(p, q), quatconj(p))

def quatpow(q, n):
	w, x, y, z = q
	mag    = quatmag(q)
	ct     = w / mag
	st     = sqrt(x**2 + y**2 + z**2) / mag
	theta  = atan2(st, ct)
	expmag = mag ** n
	alpha  = expmag * sin(n * theta) / st
	
	return (expmag * cos(n * theta), alpha * x, alpha * y, alpha * z);

def quatslerp(q, p, alpha):
	return quatmul(quatpow(quatmul(p, quatconj(q)), alpha), q)

def quatdiff(q, p):
	return quatmul(quatconj(q), p)

def quat2ypr(q):
	w, x, y, z = q
	yaw   = asin (2 * x * y + 2 * z * w)
	pitch = atan2(2 * x * w - 2 * y * z, 1 - 2 * x * x - 2 * z * z)
	roll  = atan2(2 * y * w - 2 * x * z, 1 - 2 * y * y - 2 * z * z)
	
	return (yaw, pitch, roll)

def tolocalcoords(x, y, z, qx, qy, qz, qw):
	dx = np.diff(x)
	dy = np.diff(y)
	dz = np.diff(z)
	dunused = [0 for i in xrange(len(dx))]
	
	dpos = zip(dx, dy, dz, dunused)
	
	quats     = zip(qw, qx, qy, qz)
	halfquat  = [quatslerp(quats[i], quats[i + 1], 0.5) for i in xrange(len(quats) - 1)]
	dquat     = [quatdiff (quats[i], quats[i + 1])      for i in xrange(len(quats) - 1)]
	dlocal    = [quatconjby(dpos[i], halfquat[i])       for i in xrange(len(dpos))]
	
	dlocx, dlocy, dlocz, dunused = zip(*dlocal)
	
	dangle = [quat2ypr(dquat[i]) for i in xrange(len(dquat))]
	
	dyaw, dpitch, droll = zip(*dangle)
	
	return dlocx, dlocy, dlocz, dyaw, dpitch, droll

def writemovements(filename, dx, dy, dz, dyaw, dpitch, droll):
	with open(filename, 'w') as odofile:
		for i in xrange(len(dx)):
			odofile.write(str(dx[i]) + " " +
			              str(dy[i]) + " " +
			              str(dz[i]) + " " +
			              str(dyaw[i]) + " " +
			              str(dpitch[i]) + " " +
			              str(droll[i]) + " " +
			              "0" + "\n")  # last one is dcamera

if __name__ == '__main__':
	if len(sys.argv) != 4:
		print "usage: python", __file__, "groundtruth.txt depth.txt output.in"
		sys.exit(1)
	
	print "reading groundtruth"
	timegt, tx, ty, tz, qw, qx, qy, qz = readgroundtruth(sys.argv[1])
	
	fig1 = plot.figure(1)
	ax1  = fig1.add_subplot(111, projection="3d")
	ax1.plot(xs=tx, ys=ty, zs=tz)
	
	print "reading depth timestamps"
	timedepth = readdepth(sys.argv[2])
	
	print "interpolating data"
	tx = np.interp(timedepth, timegt, tx)
	ty = np.interp(timedepth, timegt, ty)
	tz = np.interp(timedepth, timegt, tz)
	qw = np.interp(timedepth, timegt, qw)
	qx = np.interp(timedepth, timegt, qx)
	qy = np.interp(timedepth, timegt, qy)
	qz = np.interp(timedepth, timegt, qz)
	
	fig2 = plot.figure(2)
	ax2  = fig2.add_subplot(111, projection="3d")
	ax2.plot(xs=tx, ys=ty, zs=tz)
	
	for i in xrange(20):
		print "{: .4f}".format(tx[i] - tx[0]),
		print "{: .4f}".format(ty[i] - ty[0]),
		print "{: .4f}".format(tz[i] - tz[0]),
		print "{: .4f}".format(qw[i] - qw[0]),
		print "{: .4f}".format(qx[i] - qx[0]),
		print "{: .4f}".format(qy[i] - qy[0]),
		print "{: .4f}".format(qz[i] - qz[0])
	
	print "calculating local differential forms"
	dx, dy, dz, dyaw, dpitch, droll = tolocalcoords(tx, ty, tz, qx, qy, qz, qw)
	
	fig3 = plot.figure(3)
	ax3  = fig3.add_subplot(111, projection="3d")
	ax3.plot(xs=dx, ys=dy, zs=dz)
	
	print "writing to file"
	writemovements(sys.argv[3], dx, dy, dz, dyaw, dpitch, droll)
	
	plot.show()
