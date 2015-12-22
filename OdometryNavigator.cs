// OdometryNavigator.cs
// Navigator that only follows the odometry data
// Part of MonoRFS
//
// Copyright (c) 2015, Angelo Falchetti
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * The names of its contributors may not be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ANGELO FALCHETTI BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

using System;
using System.Collections.Generic;

using Accord.Math;

using Microsoft.Xna.Framework;

using TimedState    = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedMapModel = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<monorfs.Gaussian>>>;

namespace monorfs
{
/// <summary>
/// "SLAM" solver. It uses the odometry data
/// directly with no major intelligence.
/// </summary>
public class OdometryNavigator : Navigator
{
	/// <summary>
	/// Maximum euclidean distance for which two landmarks are considered the same and merged.
	/// </summary>
	public static double MergeThreshold { get { return Config.OdometryMergeThreshold; } }

	/// <summary>
	/// Most accurate estimate of the current vehicle pose.
	/// </summary>
	public override TrackVehicle BestEstimate { get; set; }

	/// <summary>
	/// Most accurate estimate model of the map.
	/// </summary>
	public override Map BestMapModel { get; set; }

	/// <summary>
	/// Construct a OdometryNavigator using the indicated vehicle as a reference.
	/// </summary>
	/// <param name="vehicle">Vehicle to track.</param>
	public OdometryNavigator(Vehicle vehicle)
		: base(vehicle)
	{
		BestEstimate = new TrackVehicle(vehicle, 1, 1, 1, 0);
		BestMapModel = new Map();
	}
	
	/// <summary>
	/// Reset the map model to an empty map.
	/// </summary>
	public override void ResetMapModel()
	{
		BestMapModel.Clear();
	}

	/// <summary>
	/// Update the vehicle particles.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="reading">Odometry reading (dx, dy, dz, dpitch, dyaw, droll).</param>
	public override void Update(GameTime time, double[] reading)
	{
		if (OnlyMapping) {
			BestEstimate.Pose      = new Pose3D(RefVehicle.Pose);
			BestEstimate.WayPoints = new TimedState(RefVehicle.WayPoints);
		}
		else {
			BestEstimate.Update(time, reading);
		}

		UpdateTrajectory(time);
	}

	/// <summary>
	/// Update both the estimated map and the localization.
	/// This means doing a model prediction and a measurement update.
	/// This method is the core of the whole program.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="measurements">Sensor measurements in pixel-range form.</param>
	public override void SlamUpdate(GameTime time, List<double[]> measurements)
	{
		List<double[]> landmarks = measurements.ConvertAll(m => BestEstimate.MeasureToMap(m));
		double MT2 = MergeThreshold * MergeThreshold;
		double[][] dummycov = 0.001.Multiply(Accord.Math.Matrix.Identity(3).ToArray());

		List<Gaussian> maplist = BestMapModel.ToList();

		foreach (double[] landmark in landmarks) {
			int    nearest   = int.MinValue;
			double neardist2 = double.MaxValue;

			for (int i = 0; i < maplist.Count; i++) {
				double dist2 = landmark.SquareEuclidean(maplist[i].Mean);
				if (neardist2 > dist2) {
					neardist2 = dist2;
					nearest   = i;
				}
			}

			if (neardist2 > MT2) {
				maplist.Add(new Gaussian(landmark, dummycov, 1.0));
			}
			else {
				double   newweight = maplist[nearest].Weight + 1;
				double[] average   = ((newweight - 1).Multiply(maplist[nearest].Mean)
				                         .Add(landmark)).Divide(newweight);
				maplist[nearest] = new Gaussian(average, dummycov, newweight);
			}
		}

		BestMapModel.Clear();

		foreach (Gaussian landmark in maplist) {
			BestMapModel.Add(landmark);
		}

		UpdateMapHistory(time);
	}
}
}
