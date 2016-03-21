#!/usr/bin/env python
# gtodometry.py
# Groundtruth to odometry readings converter
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
		return other * self * other.conjugate()
	
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
	
	def diff(self, other):
		return self.conjugate() * other
	
	def ypr(self):
		w = self.w
		x = self.x
		y = self.y
		z = self.z
		
		yaw   = atan2(2 * (w * y + x * z), 1 - 2 * (y * y + x * x))
		pitch = asin (2 * (w * x - z * y))
		roll  = atan2(2 * (w * z + y * x), 1 - 2 * (x * x + z * z))
		
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
			qw  .append(float(framedata[4]))
			qx  .append(float(framedata[5]))
			qy  .append(float(framedata[6]))
			qz  .append(float(framedata[7]))
	
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

def rotate(qw, qx, qy, qz, ryaw, rpitch, rroll):
	rot     = Quaternion.FromYPR(ryaw, rpitch, rroll)
	rotated = [Quaternion(q[0], q[1], q[2], q[3]) * rot for q in zip(qw, qx, qy, qz)]
	qw      = [q.w for q in rotated]
	qx      = [q.x for q in rotated]
	qy      = [q.y for q in rotated]
	qz      = [q.z for q in rotated]
	
	return (qw, qx, qy, qz)

def tolocalcoords(x, y, z, qw, qx, qy, qz):
	dx = np.diff(x)
	dy = np.diff(y)
	dz = np.diff(z)
	
	dpos     = [Quaternion(0, px, py, pz)                     for (px, py, pz)     in zip(dx, dy, dz)]
	quats    = [Quaternion(pw, px, py, pz).normalize()        for (pw, px, py, pz) in zip(qw, qx, qy, qz)]
	halfquat = [quats[i].slerp(quats[i + 1], 0.5)             for i                in xrange(len(quats) - 1)]
	dquat    = [quats[i].conjugate() * quats[i + 1]           for i                in xrange(len(quats) - 1)]
	dlocal   = [dpos [i].conjugateby(halfquat[i].conjugate()) for i                in xrange(len(dpos))]
	
	dlocw = [q.w for q in dlocal]
	dlocx = [q.x for q in dlocal]
	dlocy = [q.y for q in dlocal]
	dlocz = [q.z for q in dlocal]
	
	dangle = [dq.ypr() for dq in dquat]
	
	dyaw, dpitch, droll = zip(*dangle)
	
	return dlocx, dlocy, dlocz, dyaw, dpitch, droll

def toglobalcoords(dlocx, dlocy, dlocz, dyaw, dpitch, droll, x0=0, y0=0, z0=0, qw0=1, qx0=0, qy0=0, qz0=0):
	location    = Point(x0, y0, z0)
	orientation = Quaternion(qw0, qx0, qy0, qz0)
	
	x  = [location.x]
	y  = [location.y]
	z  = [location.z]
	qw = [orientation.w]
	qx = [orientation.x]
	qy = [orientation.y]
	qz = [orientation.z]
	
	for i in xrange(len(dlocx)):
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
	
	return location, orientation

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
	if len(sys.argv) > 4 or len(sys.argv) < 3:
		print "usage: python", __file__, "output.in groundtruth.txt [depth.txt]"
		sys.exit(1)
	
	print "reading groundtruth"
	timegt, tx, ty, tz, qw, qx, qy, qz = readgroundtruth(sys.argv[2])
	
	if len(sys.argv) > 3:
		print "reading depth timestamps"
		timedepth = readdepth(sys.argv[3])
		
		print "interpolating data"
		tx = np.interp(timedepth, timegt, tx)
		ty = np.interp(timedepth, timegt, ty)
		tz = np.interp(timedepth, timegt, tz)
		qw = np.interp(timedepth, timegt, qw)
		qx = np.interp(timedepth, timegt, qx)
		qy = np.interp(timedepth, timegt, qy)
		qz = np.interp(timedepth, timegt, qz)
	
	print "calculating local differential forms"
	
	qw, qx, qy, qz = rotate(qw, qx, qy, qz, 0, np.pi, 0)
	#qw, qx, qy, qz = rotate(qw, qx, qy, qz, 0, -np.pi/2, 0)
	
	print qw[0], qx[0], qy[0], qz[0]
	
	with open("assets/sim.world") as sf:
		lines = sf.readlines()
	
	lines[1] = "\t{:.16f} {:.16f} {:.16f} {:.16f} {:.16f} {:.16f} {:.16f}\n".format(tx[0], ty[0], tz[0], qw[0], qx[0], qy[0], qz[0])
	
	with open("assets/sim.world", "w") as sf:
		sf.writelines(lines)
	
	dx, dy, dz, dyaw, dpitch, droll = tolocalcoords(tx, ty, tz, qw, qx, qy, qz)
	x2, y2, z2, qw2, qx2, qy2, qz2  = toglobalcoords(dx, dy, dz, dyaw, dpitch, droll, tx[0], ty[0], tz[0], qw[0], qx[0], qy[0], qz[0])
	
	#fig = plot.figure()
	#ax = fig.gca(projection='3d')
	#ax.plot(tx, ty, tz)
	#fig2 = plot.figure()
	#ax2 = fig2.gca(projection='3d')
	#ax2.plot(x2, y2, z2)
	#plot.show()
	
	#print sum(np.array(x2) - np.array(tx)), sum(np.array(y2) - np.array(ty)), sum(np.array(z2) - np.array(tz))
	
	#dz = [-z for z in dz]
	
	print "writing to file"
	writemovements(sys.argv[1], dx, dy, dz, dyaw, dpitch, droll)
