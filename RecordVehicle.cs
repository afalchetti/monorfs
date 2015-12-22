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
using TimedArray        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedTrajectory   = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<System.Tuple<double, double[]>>>>;
using TimedMapModel     = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<monorfs.Gaussian>>>;
using TimedMeasurements = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<double[]>>>;
using TimedMessage      = System.Collections.Generic.List<System.Tuple<double, string>>;

namespace monorfs
{
/// <summary>
/// Vehicle model using prerecorded data.
/// It uses a 3d odometry motion model (yaw-pitch-roll) and
/// a pixel-range measurement model.
/// </summary>
/// <remarks>This class is not completely conformant
/// with the rest of the hierarchy, since it requires that
/// methods are called in the precise order that were used
/// when they were recorded and this order should alternate
/// between updating, odometry readings and measuring
/// (otherwise the behavior is undefined).
/// This is to simplify its design and diminish the required
/// recorded data size and shouldn't be a problem anyway, since
/// comparing different algorithms should always be performed
/// on exactly the same input anyway if any bias is to be avoided.</remarks>
public class RecordVehicle : Vehicle
{
	/// <summary>
	/// Prerecorded trajectory.
	/// </summary>
	public TimedState Trajectory { get; private set; }

	/// <summary>
	/// Prerecorded odometry.
	/// </summary>
	/// <value>The odometry.</value>
	public TimedArray Odometry { get; private set; }

	/// <summary>
	/// Prerecorded measurements.
	/// </summary>
	public TimedMeasurements Measurements { get; private set; }

	/// <summary>
	/// Prerecorded tags. Used for the SLAM/Mapping events.
	/// </summary>
	public TimedMessage Tags { get; private set; }

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
	/// <param name="odometry">Prerecorded odometry.</param>
	/// <param name="measurements">Prerecorded measurements.</param>
	/// <param name="tags">Prerecorded tags.</param>
	/// <param name="landmarks">Landmark 3d locations against which the measurements were performed.</param>
	public RecordVehicle(TimedState trajectory, TimedArray odometry,
	                     TimedMeasurements measurements, TimedMessage tags,
	                     List<double[]> landmarks)
	{
		Trajectory   = trajectory;
		Odometry     = odometry;
		Measurements = measurements;
		Tags         = tags;

		if (Trajectory.Count != Measurements.Count) {
			throw new ArgumentException("Trajectory and measurement data must be have the same length.");
		}

		if (Trajectory.Count != Odometry.Count) {
			throw new ArgumentException("Trajectory and odometry data must be have the same length.");
		}

		if (Trajectory.Count < 1) {
			throw new ArgumentException("Trajectory length must be nonzero");
		}

		RecLength  = Trajectory.Count;
		FrameIndex = 1;

		Landmarks = landmarks;
		State     = Trajectory[0].Item2;

		WayPoints.Clear();
		WayPoints.Add(Tuple.Create(0.0, Util.SClone(State)));
	}

	/// <summary>
	/// Apply the motion model to the vehicle.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="reading">Odometry reading (dx, dy, dz, dpitch, dyaw, droll).</param>
	public override void Update(GameTime time, double[] reading)
	{
		State = Trajectory[FrameIndex].Item2;
		WayPoints.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, Util.SClone(State)));

		// Updates SLAM/Mapping suggestions from tags
		WantsSLAM    = false;
		WantsMapping = false;

		double viewtime = Trajectory[FrameIndex].Item1;
		int    tagindex = Tags.BinarySearch(Tuple.Create(viewtime, ""));

		if (tagindex != ~Tags.Count) {
			if (tagindex < 0) {
				tagindex = ~tagindex;
			}

			if (Math.Abs(Tags[tagindex].Item1 - viewtime) < 1e-5) {
				if (Tags[tagindex].Item2 == "SLAM mode on") {
					WantsSLAM = true;
				}
				else if (Tags[tagindex].Item2 == "Mapping mode on") {
					WantsMapping = true;
				}
			}
		}
	}

	/// <summary>
	/// Obtain the cumulative odometry reading since the last call to this function.
	/// Note that this version doesn't actually "reset" the odometry
	/// and gives the discrete recorded points in the odometry file.
	/// Calling multiple times this function would give the same result, even
	/// if the base behavior would be to return zero after the first time.
	/// </summary>
	/// <returns>State diff.</returns>
	public override double[] ReadOdometry(GameTime time)
	{
		double[] odometry = Odometry[FrameIndex].Item2;

		WayOdometry.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, Util.SClone(odometry)));

		return odometry;
	}

	/// <summary>
	/// Obtain several measurements from the hidden state.
	/// Ladmarks may be misdetected.
	/// </summary>
	/// <returns>Pixel-range measurements.</returns>
	public override List<double[]> Measure()
	{
		List<double[]> measurements = Measurements[FrameIndex].Item2;

		if (FrameIndex < RecLength - 1) {
			FrameIndex++;
		}
		else {
			WantsToStop = true;
		}

		MappedMeasurements.Clear();
		foreach (double[] z in measurements) {
			MappedMeasurements.Add(MeasureToMap(z));
		}

		return measurements;
	}
}
}
