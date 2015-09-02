// RecordVehicle.cs
// Vehicle motion and measurement from prerecorded data
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

using U  = monorfs.Util;
using ME = monorfs.MatrixExtensions;

using TimedState        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedTrajectory   = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<System.Tuple<double, double[]>>>>;
using TimedMapModel     = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<monorfs.Gaussian>>>;
using TimedMeasurements = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<double[]>>>;

namespace monorfs
{
/// <summary>
/// Vehicle model using prerecorded data.
/// It uses a 3d odometry motion model (yaw-pitch-roll) and
/// a pixel-range measurement model.
/// </summary>
public class RecordVehicle : Vehicle
{
	/// <summary>
	/// Prerecorded trajectory.
	/// </summary>
	public TimedState Trajectory { get; private set; }

	/// <summary>
	/// Prerecorded measurements.
	/// </summary>
	public TimedMeasurements Measurements { get; private set; }

	/// <summary>
	/// Current frame index.
	/// </summary>
	public int FrameIndex { get; private set; }

	/// <summary>
	/// Number of frames in the recording.
	/// </summary>
	public int RecLength { get; private set; }

	/// <summary>
	/// Construct a RecordVehicle from prerecorded data.
	/// </summary>
	/// <param name="trajectory">Prerecorded trajectory.</param>
	/// <param name="measurements">Prerecorded measurements.</param>
	/// <param name="landmarks">Landmark 3d locations against which the measurements were performed.</param>
	public RecordVehicle(TimedState trajectory, TimedMeasurements measurements,
	                     List<double[]> landmarks)
	{
		Trajectory   = trajectory;
		Measurements = measurements;

		if (Trajectory.Count != Measurements.Count) {
			throw new ArgumentException("Trajectory and measurement data must have the same size");
		}

		if (Trajectory.Count < 1) {
			throw new ArgumentException("Trajectory length must be nonzero");
		}

		// duplicate last frame, with no measurements (freeze frame)
		Trajectory.Add(Tuple.Create(double.MaxValue, Trajectory[Trajectory.Count - 1].Item2));
		Measurements.Add(Tuple.Create(double.MaxValue, new List<double[]>()));

		RecLength  = Trajectory.Count;
		FrameIndex = 0;

		Landmarks = landmarks;
		State     = Trajectory[0].Item2;
	}

	/// <summary>
	/// Apply the motion model to the vehicle.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="dx">Moved distance from odometry in the local vertical movement-perpendicular direction since last timestep.</param>
	/// <param name="dy">Moved distance from odometry in the local horizontal movement-perpendicular direction since last timestep.</param>
	/// <param name="dz">Moved distance from odometry in the local depth movement-parallel direction since last timestep.</param>
	/// <param name="dyaw">Angle variation from odometry in the yaw coordinate since last timestep.</param>
	/// <param name="dpitch">Angle variation from odometry in the pitch coordinate since last timestep.</param>
	/// <param name="droll">Angle variation from odometry in the roll coordinate since last timestep.</param>
	public override void Update(GameTime time, double dx, double dy, double dz, double dyaw, double dpitch, double droll)
	{
		State = Trajectory[FrameIndex].Item2;
		WayPoints.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, Util.SClone(State)));
	}

	/// <summary>
	/// Obtain several measurements from the hidden state.
	/// Ladmarks may be misdetected.
	/// </summary>
	/// <returns>Pixel-range measurements.</returns>
	public override List<double[]> Measure()
	{
		List<double[]> measurements = Measurements[FrameIndex].Item2;

		FrameIndex = Math.Min(FrameIndex + 1, RecLength - 1);

		MappedMeasurements.Clear();
		foreach (double[] z in measurements) {
			MappedMeasurements.Add(MeasureToMap(z));
		}

		return measurements;
	}
}
}
