#!/usr/bin/env python
# quick groundtruth -> perfect odometry converter

import numpy as np
import sys
from math import sqrt, exp, cos, sin, asin, acos, atan2

import matplotlib.pyplot as plot
from mpl_toolkits.mplot3d import axes3d

class Quaternion:
	w = 0
	x = 0
	y = 0
	z = 0
	
	def __init__(self, w, x, y, z):
		self.w = w
		self.x = x
		self.y = y
		self.z = z
		
		#assert w == 0 or abs(abs(self) - 1) < 1e-3, abs(self)
	
	def __add__(self, other):
		return Quaternion(self.w + other.w, self.x + other.x, self.y + other.y, self.z + other.z)
	
	def __mul__(self, other):
		w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
		x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
		y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
		z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
		return Quaternion(w, x, y, z)
	
	def __abs__(self):
		return sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
	
	def normalize(self):
		mag = abs(self)
		
		if mag > 0:
			w = self.w / mag
			x = self.x / mag
			y = self.y / mag
			z = self.z / mag
		else:
			w = 0
			x = 0
			y = 0
			z = 0
		
		return Quaternion(w, x, y, z)
	
	def conjugate(self):
		return Quaternion(self.w, -self.x, -self.y, -self.z)
	
	def conjugateby(self, other):
		#return other.conjugate() * self * other
		return other * self * other.conjugate()
	
	#def __pow__(self, n):
	#	magnitude = abs(self)
	#	ct        = self.w / magnitude
	#	st        = sqrt(self.x**2 + self.y**2 + self.z**2) / magnitude
	#	theta     = atan2(st, ct)
	#	expmag    = magnitude**n
	#	alpha     = expmag * cos(n * theta) #/ ct
	#	beta      = expmag * sin(n * theta) / st
	#	
	#	return Quaternion(alpha, beta * self.x, beta * self.y, beta * self.z)
	
	def slerp(self, other, amount):
		inner = self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z
		sign  = 1 if inner >= 0 else -1
		inner = abs(inner)
		
		if inner > 0.999999:
			alpha = 1 - amount
			beta  = sign * amount
		else:
			acin = acos(inner)
			isin = 1 / sin(acin)
			
			alpha = sin((1 - amount) * acin) * isin
			beta  = sign * sin(amount * acin) * isin
		
		return Quaternion(alpha * self.w + beta * other.w,
		                  alpha * self.x + beta * other.x,
		                  alpha * self.y + beta * other.y,
		                  alpha * self.z + beta * other.z)
	
	#def slerp(self, other, alpha):
	#	return (other * self.conjugate())**alpha * self
	
	def diff(self, other):
		#return self * other.conjugate()
		return self.conjugate() * other
	
	def ypr(self):
		w = self.w
		x = self.x
		y = self.y
		z = self.z
		
		yaw   = atan2(2 * (w * y + x * z), 1 - 2 * (y * y + x * x))
		pitch = asin (2 * (w * x - z * y))
		roll  = atan2(2 * (w * z + y * x), 1 - 2 * (x * x + z * z))
		
		#yaw   = asin (2 * x * y + 2 * z * w)
		#pitch = atan2(2 * x * w - 2 * y * z, 1 - 2 * x * x - 2 * z * z)
		#roll  = atan2(2 * y * w - 2 * x * z, 1 - 2 * y * y - 2 * z * z)
		return (yaw, pitch, roll)
	
	@staticmethod
	def FromYPR(yaw, pitch, roll):
		r  = 0.5 * roll
		sr = sin(r)
		cr = cos(r)
		
		p  = 0.5 * pitch
		sp = sin(p)
		cp = cos(p)
		
		y  = 0.5 * yaw
		sy = sin(y)
		cy = cos(y)
		
		return Quaternion(cy * cp * cr + sy * sp * sr,
		                  cy * sp * cr + sy * cp * sr,
		                  sy * cp * cr - cy * sp * sr,
		                  cy * cp * sr - sy * sp * cr)
	
	def __str__(self):
		return "(" + str(self.w) + ", " + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + ")"

class Point:
	x = 0
	y = 0
	z = 0
	
	def __init__(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z
	
	def __str__(self):
		return "(" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + ")"

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

def tolocalcoords(x, y, z, qw, qx, qy, qz):
	dx = np.diff(x)
	dy = np.diff(y)
	dz = np.diff(z)
	
	#print "\ntolocal"
	
	dpos     = [Quaternion(0, px, py, pz)                     for (px, py, pz)     in zip(dx, dy, dz)]
	quats    = [Quaternion(pw, px, py, pz).normalize()        for (pw, px, py, pz) in zip(qw, qx, qy, qz)]
	halfquat = [quats[i].slerp(quats[i + 1], 0.5)             for i                in xrange(len(quats) - 1)]
	dquat    = [quats[i].conjugate() * quats[i + 1]           for i                in xrange(len(quats) - 1)]
	dlocal   = [dpos [i].conjugateby(halfquat[i].conjugate()) for i                in xrange(len(dpos))]
	
	dlocw = [q.w for q in dlocal]
	dlocx = [q.x for q in dlocal]
	dlocy = [q.y for q in dlocal]
	dlocz = [q.z for q in dlocal]
	
	dangle = [dquat[i].ypr() for i in xrange(len(dquat))]
	
	dyaw, dpitch, droll = zip(*dangle)
	
	#for i in xrange(len(dpos)):
	#	print i, "=>"
	#	print "input... location=", Point(x[i], y[i], z[i]), "orientation=", Quaternion(qw[i], qx[i], qy[i], qz[i])
	#	print "dpos:", dpos[i]
	#	print "halfquat:", halfquat[i]
	#	print "dquat:", dquat[i]
	#	print "dlocal:", dlocal[i]
	#	print "dangle:", dangle[i]
	
	return dlocx, dlocy, dlocz, dyaw, dpitch, droll

def toglobalcoords(dlocx, dlocy, dlocz, dyaw, dpitch, droll):
	location    = Point(0, 0, 0)
	orientation = Quaternion(0, 1, 0, 0)
	
	x  = [location.x]
	y  = [location.y]
	z  = [location.z]
	qw = [orientation.w]
	qx = [orientation.x]
	qy = [orientation.y]
	qz = [orientation.z]
	
	#print "\ntoglobal"
	
	for i in xrange(len(dlocx)):
		#print i, "=>"
		location, orientation = update(location, orientation, dlocx[i], dlocy[i], dlocz[i], dyaw[i], dpitch[i], droll[i])
		x .append(location.x)
		y .append(location.y)
		z .append(location.z)
		qw.append(orientation.w)
		qx.append(orientation.x)
		qy.append(orientation.y)
		qz.append(orientation.z)
		
	return x, y, z, qw, qx, qy, qz

def update(location, orientation, dlocx, dlocy, dlocz, dyaw, dpitch, droll):
	dorientation   = Quaternion.FromYPR(dyaw, dpitch, droll)
	neworientation = orientation * dorientation
	midrotation    = orientation.slerp(neworientation, 0.5)
	dlocation      = midrotation * Quaternion(0, dlocx, dlocy, dlocz) * midrotation.conjugate()
	
	location       = Point(location.x + dlocation.x, location.y + dlocation.y, location.z + dlocation.z)
	orientation    = neworientation
	orientation    = orientation.normalize()
	
	#print "input... dlocx=", dlocx, "dlocy=", dlocy, "dlocz=", dlocz, "dyaw=", dyaw, "dpitch=", dpitch, "droll=", droll
	#print "dorientation:", dorientation
	#print "neworientation:", neworientation
	#print "midrotation:", midrotation
	#print "dlocation:", dlocation
	#print "location:", location
	#print "orientation:", orientation
	
	return location, orientation

def newplot(fignum, x, y, z):
	fig  = plot.figure(fignum)
	axis = fig.add_subplot(111, projection="3d")
	axis.plot(xs=x, ys=y, zs=z)
	
	# Create cubic bounding box to simulate equal aspect ratio
	max_range = np.array([max(x)-min(x), max(y)-min(y), max(z)-min(z)]).max()
	Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(max(x)+min(x))
	Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(max(y)+min(y))
	Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(max(z)+min(z))
	# Comment or uncomment following both lines to test the fake bounding box:
	for xb, yb, zb in zip(Xb, Yb, Zb):
	   axis.plot([xb], [yb], [zb], 'w')
	
	axis.grid()

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
	
	# debug
	#print Quaternion.FromYPR(0.3, 0.1, 0.4).ypr()          #should be (0.3, 0.1, 0.4)
	#print Quaternion(1, 0, 0, 0) * Quaternion(1, 0, 0, 0)  # should be  1
	#print Quaternion(0, 1, 0, 0) * Quaternion(0, 1, 0, 0)  # should be -1
	#print Quaternion(0, 0, 1, 0) * Quaternion(0, 0, 1, 0)  # should be -1
	#print Quaternion(0, 0, 0, 1) * Quaternion(0, 0, 0, 1)  # should be -1
	
	print "reading groundtruth"
	timegt, tx, ty, tz, qw, qx, qy, qz = readgroundtruth(sys.argv[1])
	
	newplot(1, tx, ty, tz)
	#ax1.scatter(xs=tx[0],  ys=ty[0],  zs=tz[0],  c='b', marker='o')
	#ax1.scatter(xs=tx[-1], ys=ty[-1], zs=tz[-1], c='r', marker='s')
	
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
	
	#timedepth =  timedepth[0:3]
	#tx = tx[0:3]
	#ty = ty[0:3]
	#tz = tz[0:3]
	#qw = qw[0:3]
	#qx = qx[0:3]
	#qy = qy[0:3]
	#qz = qz[0:3]
	
	#tx[0] = ty[0] = tz[0] = qw[0] = qx[0] = qy[0] = qz[0] = 0
	#qx[0] = 1
	
	newplot(2, tx, ty, tz)
	
	#for i in xrange(min(20, len(tx))):
	#	print "{: .4f}".format(tx[i] - tx[0]),
	#	print "{: .4f}".format(ty[i] - ty[0]),
	#	print "{: .4f}".format(tz[i] - tz[0]),
	#	print "{: .4f}".format(qw[i] - qw[0]),
	#	print "{: .4f}".format(qx[i] - qx[0]),
	#	print "{: .4f}".format(qy[i] - qy[0]),
	#	print "{: .4f}".format(qz[i] - qz[0])
	
	print "calculating local differential forms"
	dx, dy, dz, dyaw, dpitch, droll = tolocalcoords(tx, ty, tz, qw, qx, qy, qz)
	x2, y2, z2, qw2, qx2, qy2, qz2  = toglobalcoords(dx, dy, dz, dyaw, dpitch, droll)
	
	newplot(3, dx, dy, dz)
	newplot(4, x2, y2, z2)
	
	print "writing to file"
	writemovements(sys.argv[3], dx, dy, dz, dyaw, dpitch, droll)
	
	#plot.show()