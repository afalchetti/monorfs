// SimulatedVehicle.cs
// Vehicle motion and measurement model using a simulated environment
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

using Accord;
using Accord.Math;
using Accord.Statistics.Distributions.Univariate;
using AForge.Math.Random;
using AForge;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework;

using U  = monorfs.Util;
using ME = monorfs.MatrixExtensions;

namespace monorfs
{
/// <summary>
/// Vehicle model.
/// It uses a 3d odometry motion model (yaw-pitch-roll) and
/// a pixel-range measurement model.
/// </summary>
public class SimulatedVehicle : Vehicle
{
	/// <summary>
	/// Probability of detection.
	/// </summary>
	private readonly double detectionProbability = 0.9;

	/// <summary>
	/// Amount of expected clutter (spuriousness) on the measurement process.
	/// </summary>
	public readonly double ClutterDensity = 3e-7;

	/// <summary>
	/// Clutter density integral over the whole measurement space, i.e.
	/// the expected number of clutter measurements.
	/// </summary>
	public readonly double ClutterCount;

	/// <summary>
	/// Defines if this vehicle remains perfectly still when no input commands are given.
	/// If the robot is well located on the ground this makes sense, but if it is a handheld
	/// device, it is not so easy to maintain it steady.
	/// </summary>
	public const bool PerfectStill = false;

	/// <summary>
	/// Landmark 3d locations against which the measurements are performed.
	/// </summary>
	public List<double[]> Landmarks { get; set; }

	/// <summary>
	/// Poisson distributed random generator with parameter lambda = ClutterCount.
	/// </summary>
	private PoissonDistribution clutterGen;

	/// <summary>
	/// Construct a new Vehicle from a "zero initial state", i.e. location = {0, 0, 0},
	/// theta = 0 and rotation axis = {1, 0, 0} with no landmarks.
	/// </summary>
	public SimulatedVehicle()
		: this(new double[3] {0, 0, 0}, 0, new double[3] {1, 0, 0}, new List<double[]>()) {}

	/// <summary>
	/// Construct a new Vehicle object from its initial state.
	/// </summary>
	/// <param name="location">Spatial coordinates.</param>
	/// <param name="theta">Orientation angle.</param>
	/// <param name="axis">Orientation rotation axis.</param>
	/// <param name="landmarks">Landmark 3d locations against which the measurements are performed.</param>
	public SimulatedVehicle(double[] location, double theta, double[] axis, List<double[]> landmarks)
		: base(location, theta, axis)
	{
		ClutterCount       = ClutterDensity * FilmArea.Height * FilmArea.Width * RangeClip.Length;
		clutterGen         = new PoissonDistribution(ClutterCount);
		Landmarks          = landmarks;
		SidebarWidth       = 1;
		SidebarHeight      = 1;
	}

	/// <summary>
	/// Copy constructor. Perform a deep copy of another simulated vehicle.
	/// </summary>
	/// <param name="that">Copied simulated vehicle.</param>
	/// <param name="copytrajectory">If true, the vehicle historic trajectory is copied. Relatively heavy operation.</param>
	public SimulatedVehicle(SimulatedVehicle that, bool copytrajectory = false)
		: this((Vehicle) that, copytrajectory)
	{
		this.Landmarks            = that.Landmarks;
		this.detectionProbability = that.detectionProbability;
		this.ClutterDensity       = that.ClutterDensity;
		this.ClutterCount         = that.ClutterCount;
		this.clutterGen           = new PoissonDistribution(that.clutterGen.Mean);
	}

	/// <summary>
	/// Perform a deep copy of another general vehicle.
	/// </summary>
	/// <param name="that">Copied general vehicle.</param>
	/// <param name="copytrajectory">If true, the vehicle historic trajectory is copied. Relatively heavy operation.</param>
	public SimulatedVehicle(Vehicle that, bool copytrajectory = false)
		: base(that, copytrajectory) {}

	/// <summary>
	/// Perform a deep copy of another general vehicle,
	/// scaling the covariance matrices and setting a new detection probability.
	/// </summary>
	/// <param name="that">Copied vehicle.</param>
	/// <param name="motioncovmultiplier">Scalar multiplier for the motion covariance matrix.</param>
	/// <param name="measurecovmultiplier">Scalar multiplier for the measurement covariance matrix.</param>
	/// <param name="pdetection">Probability of detection.</param>
	/// <param name="clutter">Clutter density.</param>
	/// <param name="copytrajectory">If true, the vehicle historic trajectory is copied. Relatively heavy operation.</param>
	public SimulatedVehicle(Vehicle that, double motioncovmultiplier, double measurecovmultiplier, double pdetection, double clutter, bool copytrajectory = false)
	           : this(that, copytrajectory)
	{
		this.detectionProbability = pdetection;
		this.ClutterDensity       = clutter;
		this.ClutterCount         = this.ClutterDensity * this.FilmArea.Height * this.FilmArea.Width * this.RangeClip.Length;
		this.clutterGen           = new PoissonDistribution(this.ClutterCount);
	}

	/// <summary>
	/// Apply the motion model to the vehicle. It corresponds to a
	/// 3D odometry model following the equation:
	/// 
	/// x = x + q dx q* + N(0, Q)
	/// o = dq o dq* + N(0, Q')
	/// 
	/// where q is the midrotation quaternion (halfway between the old and new orientations) and N(a, b) is a normal function
	/// with mean 'a' and covariance matrix 'b'.
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
		// no input, static friction makes the robot stay put (if there is any static friction)
		if (PerfectStill && dx == 0 && dy == 0 && dz == 0 && dyaw == 0 && dpitch == 0 && droll == 0) {
			return;
		}

		// note that the framework uses Yaw = Y, Pitch = X, Roll = Z => YXZ Tait-Bryan parametrization
		// this is equivalent to a plane pointing upwards with its wings on the X direction
		Quaternion dorientation   = Quaternion.CreateFromYawPitchRoll((float) dyaw, (float) dpitch, (float) droll);
		Quaternion neworientation = Orientation * dorientation;
		Quaternion midrotation    = Quaternion.Slerp(Orientation, neworientation, 0.5f);
		Quaternion dlocation      = midrotation * new Quaternion((float) dx, (float) dy, (float) dz, 0) * Quaternion.Conjugate(midrotation);

		Location    = new double[3] {X + dlocation.X, Y + dlocation.Y, Z + dlocation.Z};
		Orientation = neworientation;

		// Do a better noise (radial), though it seems to work just fine
		State       = State.Add(time.ElapsedGameTime.TotalSeconds.Multiply(
		                             U.RandomGaussianVector(new double[7] {0, 0, 0, 0, 0, 0, 0}, MotionCovarianceQ)));
		Orientation = Quaternion.Normalize(Orientation);
		WayPoints.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, State));
	}
	
	/// <summary>
	/// Obtain a measurement from the hidden state.
	/// It does not use any randomness or misdetection, 
	/// as it is designed to be used with UKF filters.
	/// </summary>
	/// <param name="landmark">Landmark 3d location against which the measurement is performed.</param>
	/// <returns>Pixel-range measurements.</returns>
	
	public double[] MeasurePerfect(double[] landmark)
	{
		double[]   diff  = landmark.Subtract(Location);
		Quaternion local = Quaternion.Conjugate(Orientation) *
			                    new Quaternion((float) diff[0], (float) diff[1], (float) diff[2], 0) * Orientation;

		double range  = diff.Euclidean();
		double px     = VisionFocal * local.X / local.Z;
		double py     = VisionFocal * local.Y / local.Z;

		return new double[3] {px, py, range};
	}

	/// <summary>
	/// Obtain a measurement from the hidden state. It follows a
	/// pixel-range model where the range and pixel index from a
	/// map landmark m are:
	/// 
	/// [r, px, py] = [|m-x|, (f x/z)L, (f y/z)L] + N(0, R)
	/// 
	/// where |a-b| is the euclidean distance between a and b and
	/// (.)L is performed on the local axis
	/// This method always detects the landmark (misdetection
	/// probability is ignored).
	/// </summary>
	/// <param name="landmark">Landmark 3d location against which the measurement is performed.</param>
	/// <returns>Pixel-range measurements.</returns>
	public double[] MeasureDetected(double[] landmark)
	{
		return MeasurePerfect(landmark).Add(U.RandomGaussianVector(new double[3] {0, 0, 0}, MeasurementCovariance));
	}

	/// <summary>
	/// Obtain several measurements from the hidden state.
	/// This method always detects the landmark (misdetection
	/// probability is ignored).
	/// </summary>
	/// <returns>Pixel-range measurements.</returns>
	public double[][] MeasureDetected()
	{
		double[][] measurements = new double[Landmarks.Count][];

		for (int i = 0; i < Landmarks.Count; i++) {
			measurements[i] = MeasureDetected(Landmarks[i]);
		}

		return measurements;
	}

	/// <summary>
	/// Obtain several measurements from the hidden state.
	/// Ladmarks may be misdetected.
	/// </summary>
	/// <returns>Pixel-range measurements.</returns>
	public override List<double[]> Measure()
	{
		List<double[]> measurements = new List<double[]>();

		// add every measurement with probability = DetectionProbbility
		for (int i = 0; i < Landmarks.Count; i++) {
			if (Visible(Landmarks[i])) {
				if (U.uniform.Next() < detectionProbability) {
					measurements.Add(MeasureDetected(Landmarks[i]));
				}
			}
		}

		// poisson distributed clutter measurement count
		// a cap of 10 lambda is enforced because technically
		// the number is unbounded so it could freeze the system
		int nclutter;
		try {
			// the poisson generator may underflow if lambda is too small
			nclutter = Math.Min(clutterGen.Generate(), (int)(ClutterCount * 10));
		}
		catch (ArithmeticException)
		{
			nclutter = 0;
		}

		for (int i = 0; i < nclutter; i++) {
			double px    = U.uniform.Next() * FilmArea.Width + FilmArea.Left;
			double py    = U.uniform.Next() * FilmArea.Height + FilmArea.Top;
			double range = U.uniform.Next() * RangeClip.Length + RangeClip.Min;
			measurements.Add(new double[3] {px, py, range});
		}

		MappedMeasurements.Clear();
		foreach (double[] z in measurements) {
			MappedMeasurements.Add(MeasureToMap(z));
		}

		return measurements;
	}

	/// <summary>
	/// Transform a measurement vector in measurement space (pixel-range)
	/// into a map-space vector  (x-y plane).
	/// </summary>
	/// <param name="measurement">Measurement expressed as pixel-range.</param>
	/// <returns>Measurement expressed in x-y plane.</returns>
	public override double[] MeasureToMap(double[] measurement)
	{
		double   px    = measurement[0];
		double   py    = measurement[1];
		double   range = measurement[2];

		double   alpha = range / Math.Sqrt(VisionFocal * VisionFocal + px * px + py * py);
		double[] diff  = new double[3] {alpha * px, alpha * py, alpha * VisionFocal};

		Quaternion rotated = Orientation *
			                     new Quaternion((float) diff[0], (float) diff[1], (float) diff[2], 0) * Quaternion.Conjugate(Orientation);

		return new double[3] {X + rotated.X, Y + rotated.Y, Z + rotated.Z};
	}

	/// <summary>
	/// Obtain the jacobian of the measurement model.
	/// Designed to be used with EKF filters.
	/// </summary>
	/// <param name="landmark">Landmark 3d location against which the measurement is performed.</param>
	/// <returns>Measurement model linearization jacobian.</returns>
	public double[][] MeasurementJacobian(double[] landmark)
	{
		double[]   diff  = landmark.Subtract(Location);
		Quaternion local = Quaternion.Conjugate(Orientation) *
			                    new Quaternion((float) diff[0], (float) diff[1], (float) diff[2], 0) * Orientation;

		// the jacobian of the homography projection part is given by
		// f/z I 0
		//  X/|X|
		double mag = Math.Sqrt(local.X * local.X + local.Y * local.Y + local.Z * local.Z);
		double[][] jprojection = {new double[] {VisionFocal / local.Z, 0,                     -VisionFocal * local.X / (local.Z * local.Z)},
		                          new double[] {0,                     VisionFocal / local.Z, -VisionFocal * local.Y / (local.Z * local.Z)},
		                          new double[] {local.X / mag,         local.Y / mag,         local.Z /mag}};

		// the jacobian of the change of coordinates part of the measurement process is the rotation matrix 
		double[][] jrotation = ME.MatrixFromQuaternion(Orientation);

		return jprojection.Multiply(jrotation);
	}

	/// <summary>
	/// Find if a given ladmark is visible from the current pose of the vehicle.
	/// </summary>
	/// <param name="landmark">Queried landmark.</param>
	/// <returns>True if the landmark is visible; false otherwise.</returns>
	public bool Visible(double[] landmark)
	{
		double[]   diff  = landmark.Subtract(Location);
		Quaternion local = Quaternion.Conjugate(Orientation) *
			                    new Quaternion((float) diff[0], (float) diff[1], (float) diff[2], 0) * Orientation;

		double px   = VisionFocal * local.X / local.Z;
		double py   = VisionFocal * local.Y / local.Z;

		double range2 = local.LengthSquared();

		return local.Z > 0 && RangeClip.Min * RangeClip.Min < range2 && range2 < RangeClip.Max * RangeClip.Max &&
		       FilmArea.Left < px && px < FilmArea.Right && FilmArea.Top < py && py < FilmArea.Bottom;
	}

	/// <summary>
	/// Find if a given ladmark is visible from the current pose of the vehicle
	/// using pixel-range coordinates to express the landmark.
	/// </summary>
	/// <param name="measurement">Queried landmark in pixel-range coordinates.</param>
	/// <returns>True if the landmark is visible; false otherwise.</returns>
	public bool VisibleM(double[] measurement)
	{
		return FilmArea.Left < measurement[0] && measurement[0] < FilmArea.Right &&
		       FilmArea.Top  < measurement[1] && measurement[1] < FilmArea.Bottom &&
		       RangeClip.Min < measurement[2] && measurement[2] < RangeClip.Max;
	}

	/// <summary>
	/// Get the probability of detection of a particular landmark.
	/// It is modelled as a constant if it's on the FOV and zero if not. 
	/// </summary>
	/// <param name="landmark">Queried landmark.</param>
	/// <returns></returns>
	public double DetectionProbability(double[] landmark)
	{
		return Visible(landmark) ? detectionProbability : 0;
	}

	/// <summary>
	/// Get the probability of detection of a particular landmark
	/// using pixel-range coordinates to express the landmark.
	/// It is modelled as a constant if it's on the FOV and zero if not.
	/// </summary>
	/// <param name="measurement">Queried landmark in pixel-range coordinates.</param>
	/// <returns></returns>
	public double DetectionProbabilityM(double[] measurement)
	{
		return VisibleM(measurement) ? detectionProbability : 0;
	}

	/// <summary>
	/// Render the vehicle on the graphics device.
	/// The graphics device must be ready, otherwise
	/// the method will throw an exception.
	/// <param name="camera">Camera rotation matrix.</param>
	/// </summary>
	public override void Render(double[][] camera)
	{
		base.Render(camera);

		foreach (double[] landmark in Landmarks) {
			RenderLandmark(landmark, camera);
		}
	}

	/// <summary>
	/// Simple point landmark rendering.
	/// </summary>
	/// <param name="landmark">Point landmark position.</param>
	/// <param name="camera">Camera rotation matrix.</param>
	private void RenderLandmark(double[] landmark, double[][] camera)
	{
		const float halflen = 0.024f;
		
		Color innercolor =  Color.LightGray;
		Color outercolor =  Color.Black;

		landmark = camera.Multiply(landmark);
		
		VertexPositionColor[] invertices  = new VertexPositionColor[4];
		double[][]            outvertices = new double[4][];

		outvertices[0] = new double[] {landmark[0] - halflen, landmark[1] - halflen, landmark[2]};
		outvertices[1] = new double[] {landmark[0] - halflen, landmark[1] + halflen, landmark[2]};
		outvertices[2] = new double[] {landmark[0] + halflen, landmark[1] + halflen, landmark[2]};
		outvertices[3] = new double[] {landmark[0] + halflen, landmark[1] - halflen, landmark[2]};

		invertices[0] = new VertexPositionColor(outvertices[0].ToVector3(), innercolor);
		invertices[1] = new VertexPositionColor(outvertices[1].ToVector3(), innercolor);
		invertices[2] = new VertexPositionColor(outvertices[3].ToVector3(), innercolor);
		invertices[3] = new VertexPositionColor(outvertices[2].ToVector3(), innercolor);

		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, invertices, 0, invertices.Length - 2);
		Graphics.DrawUser2DPolygon(outvertices, 0.02f, outercolor, true);
	}
}
}
