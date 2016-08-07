// FakeNavigator.cs
// Navigator that does nothing but may be intervened
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

using Microsoft.Xna.Framework;

using TimedState    = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedMapModel = System.Collections.Generic.List<System.Tuple<double, monorfs.Map>>;

namespace monorfs
{
/// <summary>
/// SLAM solver. It does nothing.
/// </summary>
public class FakeNavigator<MeasurerT, PoseT, MeasurementT> : Navigator<MeasurerT, PoseT, MeasurementT>
	where PoseT        : IPose<PoseT>, new()
	where MeasurementT : IMeasurement<MeasurementT>, new()
	where MeasurerT    : IMeasurer<MeasurerT, PoseT, MeasurementT>, new()
{
	/// <summary>
	/// Vehicle pose.
	/// </summary>
	public TrackVehicle<MeasurerT, PoseT, MeasurementT> Vehicle { get; set; }

	/// <summary>
	/// Map representation as a mixture-of-gaussians.
	/// </summary>
	public Map MapModel { get; set; }

	/// <summary>
	/// Most accurate estimate of the current vehicle pose.
	/// </summary>
	public override TrackVehicle<MeasurerT, PoseT, MeasurementT> BestEstimate { get { return Vehicle; } }

	/// <summary>
	/// Most accurate estimate model of the map.
	/// </summary>
	public override Map BestMapModel { get { return MapModel; } }

	/// <summary>
	/// Construct a PHDNavigator using the indicated vehicle as a reference.
	/// </summary>
	/// <param name="vehicle">Vehicle to track.</param>
	public FakeNavigator(Vehicle<MeasurerT, PoseT, MeasurementT> vehicle)
		: base(vehicle, true)
	{
		Vehicle  = vehicle.TrackClone(1, 1, 1, 0, false);
		MapModel = new Map();
	}

	/// <summary>
	/// Reset the map to an empty map.
	/// </summary>
	public override void ResetMapModel()
	{
		MapModel.Clear();
	}

	/// <summary>
	/// Remove all the localization and mapping history and start it again from the current position.
	/// </summary>
	public override void ResetHistory()
	{
		Vehicle.ResetHistory();
		base.ResetHistory();
	}

	/// <summary>
	/// Update the vehicle particles (does nothing).
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="reading">Odometry reading (dx, dy, dz, dpitch, dyaw, droll).</param>
	public override void Update(GameTime time, double[] reading) {}

	/// <summary>
	/// Update both the estimated map and the localization (does nothing).
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="measurements">Sensor measurements in pixel-range form.</param>
	public override void SlamUpdate(GameTime time, List<MeasurementT> measurements) {}

	/// <summary>
	/// Render the navigation HUD and the trajectory on the graphics device.
	/// The graphics device must be ready, otherwise
	/// the method will throw an exception.
	/// </summary>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public override void Render(double[][] camera)
	{
		RefVehicle.Render(camera);
		RenderTrajectory(camera);
		RenderMap(camera);
	}
}
}
