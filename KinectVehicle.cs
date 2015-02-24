// KinectVehicle.cs
// Vehicle motion and measurement model for a Kinect sensor
// Part of MonoRFS
//
// Copyright (C) 2015 Angelo Falchetti
// All rights reserved.
// Any use, reproduction, distribution or copy of this
// work via any medium is strictly forbidden without
// express written consent from the author.

using System;
using System.Collections.Generic;
using AForge;
using Microsoft.Xna.Framework;

namespace monorfs
{
/// <summary>
/// Kinect vehicle model.
/// Extracts measurements from an online Kinect sensor.
/// </summary>
public class KinectVehicle : Vehicle
{
	//public VideoStream videostream;

	public KinectVehicle()
	{
	}

	public override void Update(GameTime time, double dx, double dy, double dz, double dyaw, double dpitch, double droll)
	{
		throw new NotImplementedException();
	}

	public override List<double[]> Measure()
	{
		throw new NotImplementedException();
	}
}
}
