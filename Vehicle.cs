// Vehicle.cs
// Vehicle motion and measurement model
// Part of MonoRFS
//
// Copyright (C) 2015 Angelo Falchetti
// All rights reserved.
// Any use, reproduction, distribution or copy of this
// work via any medium is strictly forbidden without
// express written consent from the author.

using System;
using System.Collections.Generic;

using Accord;
using Accord.Math;
using AForge.Math.Random;
using AForge;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework;

using U  = monorfs.Util;
using ME = monorfs.MatrixExtensions;
using Accord.Statistics.Distributions.Univariate;

namespace monorfs
{
/// <summary>
/// Vehicle model.
/// It uses a 3d odometry motion model (yaw-pitch-roll) and
/// a pixel-range measurement model.
/// </summary>
public class Vehicle
{
	/// <summary>
	/// Motion model covariance matrix.
	/// </summary>
	public readonly double[,] MotionCovariance = new double[7, 7] {{5e-3, 0, 0, 0, 0, 0, 0},
	                                                               {0, 5e-3, 0, 0, 0, 0, 0},
	                                                               {0, 0, 5e-3, 0, 0, 0, 0},
	                                                               {0, 0, 0, 5e-3, 0, 0, 0},
	                                                               {0, 0, 0, 0, 5e-3, 0, 0},
	                                                               {0, 0, 0, 0, 0, 5e-3, 0},
	                                                               {0, 0, 0, 0, 0, 0, 5e-3}};

	/// <summary>
	/// Measurement model covariance matrix.
	/// </summary>
	public readonly double[,] MeasurementCovariance = new double[3, 3] {{2e0,  0, 0},
	                                                                    {0,  2e0, 0},
	                                                                    {0, 0, 2e-3}};

	/// <summary>
	/// Probability of detection.
	/// </summary>
	private readonly double detectionProbability = 0.85;

	/// <summary>
	/// Amount of expected clutter (spuriousness) on the measurement process.
	/// </summary>
	public readonly double ClutterDensity = 3e-7;

	/// <summary>
	/// Clutter density integral over the whole measurement space, i.e.
	/// the expected number of clutter measurements.
	/// </summary>
	private readonly double ClutterCount;

	/// <summary>
	/// Get the internal state as a vector.
	/// </summary>
	public double[] State { get; set; }

	/// <summary>
	/// Get the location x-axis coordinate.
	/// </summary>
	public double X
	{
		get { return  State[0]; }
		private set { State[0] = value; }
	}

	/// <summary>
	/// Get the location y-axis coordinate.
	/// </summary>
	public double Y
	{
		get { return  State[1]; }
		private set { State[1] = value; }
	}

	/// <summary>
	/// Get the location z-axis coordinate.
	/// </summary>
	public double Z
	{
		get { return  State[2]; }
		private set { State[2] = value; }
	}

	/// <summary>
	/// Get the space location of the vehicle, i.e. its coordinates.
	/// </summary>
	public double[] Location
	{
		get { return new double[3] {State[0], State[1], State[2]}; }
		private set { State[0] = value[0]; State[1] = value[1]; State[2] = value[2]; }
	}

	/// <summary>
	/// Get the rotation quaternion scalar component.
	/// </summary>
	public double W
	{
		get { return  State[3]; }
		private set { State[3] = value; }
	}

	/// <summary>
	/// Get the rotation quaternion vector component.
	/// </summary>
	public double[] K
	{
		get { return new double[3] {State[4], State[5], State[6]}; }
		private set { State[4] = value[0]; State[5] = value[1]; State[6] = value[2]; }
	}
	

	/// <summary>
	/// Get the orientation of the vehicle.
	/// </summary>
	public Quaternion Orientation
	{
		get { return new Quaternion((float) State[4], (float) State[5], (float) State[6], (float) State[3]); }
		private set { State[3] = value.W; State[4] = value.X; State[5] = value.Y; State[6] = value.Z; }
	}

	/// <summary>
	/// Landmark 3d locations against which the measurements are performed.
	/// </summary>
	public List<double[]> Landmarks;

	/// <summary>
	/// Vision camera focal length.
	/// </summary>
	public const double VisionFocal = 480;

	/// <summary>
	/// 2D film clipping area (i.e. sensor size and offset) in pixel units.
	/// </summary>
	public readonly Rectangle FilmArea = new Rectangle(-320, -240, 640, 480); 

	/// <summary>
	/// Range clipping planes in meters.
	/// </summary>
	/// <remarks>For now, the range is twice the Kinect's, to mae it easier to debug.</remarks>
	public readonly Range RangeClip = new Range(0.8f, 2*4f);

	/// <summary>
	/// Render output.
	/// </summary>
	public GraphicsDevice Graphics { get; set; }

	/// <summary>
	/// Trajectory through which the vehicle has moved.
	/// </summary>
	public List<double[]> Waypoints { get; private set; }

	/// <summary>
	/// Poisson distributed random generator with parameter lambda = ClutterCount.
	/// </summary>
	private PoissonDistribution clutterGen;

	/// <summary>
	/// Construct a new Vehicle object from its initial state.
	/// </summary>
	/// <param name="location">Spatial coordinates.</param>
	/// <param name="theta">Orientation angle.</param>
	/// <param name="axis">Orientation rotation axis.</param>
	/// <param name="landmarks">Landmark 3d locations against which the measurements are performed.</param>
	public Vehicle(double[] location, double theta, double[] axis, List<double[]> landmarks)
	{
		double w = Math.Cos(theta / 2);
		double d = Math.Sin(theta / 2);

		axis.Divide(axis.Euclidean(), true);

		this.State        = new double[7] {location[0], location[1], location[2], w, d * axis[0], d * axis[1], d * axis[2]};
		this.ClutterCount = this.ClutterDensity * this.FilmArea.Height * this.FilmArea.Width * this.RangeClip.Length;
		this.clutterGen   = new PoissonDistribution(this.ClutterCount);
		this.Landmarks    = landmarks;

		this.Waypoints    = new List<double[]>();
		this.Waypoints.Add(location);
	}

	/// <summary>
	/// Copy constructor. Perform a deep copy of another vehicle.
	/// </summary>
	/// <param name="that">Copied vehicle.</param>
	/// <param name="copytrajectory">If true, the vehicle historic trajectory is copied. Relatively heavy operation.</param>
	public Vehicle(Vehicle that, bool copytrajectory = false)
	{
		this.State                 = new double[7];
		that.State.CopyTo(this.State, 0);
		
		this.Landmarks             = that.Landmarks;
		this.detectionProbability  = that.detectionProbability;
		this.ClutterDensity        = that.ClutterDensity;
		this.ClutterCount          = that.ClutterCount;
		this.clutterGen            = new PoissonDistribution(that.clutterGen.Mean);
		this.MotionCovariance      = that.MotionCovariance.MemberwiseClone();
		this.MeasurementCovariance = that.MeasurementCovariance.MemberwiseClone();
		this.Graphics              = that.Graphics;

		if (copytrajectory) {
			this.Waypoints = new List<double[]>(that.Waypoints);
		}
		else {
			this.Waypoints = new List<double[]>();
			this.Waypoints.Add(that.Location);
		}
	}

	/// <summary>
	/// Copy constructor. Perform a deep copy of another vehicle,
	/// scaling the covariance matrices and setting a new detection probability.
	/// </summary>
	/// <param name="that">Copied vehicle.</param>
	/// <param name="motioncovmultiplier">Scalar multiplier for the motion covariance matrix.</param>
	/// <param name="measurecovmultiplier">Scalar multiplier for the measurement covariance matrix.</param>
	/// <param name="pdetection">Probability of detection.</param>
	/// <param name="clutter">Clutter density.</param>
	/// <param name="copytrajectory">If true, the vehicle historic trajectory is copied. Relatively heavy operation.</param>
	public Vehicle(Vehicle that, double motioncovmultiplier, double measurecovmultiplier, double pdetection, double clutter, bool copytrajectory = false)
	           : this(that, copytrajectory)
	{
		this.MotionCovariance      = motioncovmultiplier .Multiply(this.MotionCovariance);
		this.MeasurementCovariance = measurecovmultiplier.Multiply(this.MeasurementCovariance);
		this.detectionProbability  = pdetection;
		this.ClutterDensity        = clutter;
		this.ClutterCount          = this.ClutterDensity * this.FilmArea.Height * this.FilmArea.Width * this.RangeClip.Length;
		this.clutterGen            = new PoissonDistribution(this.ClutterCount);
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
	public void Update(GameTime time, double dx, double dy, double dz, double dyaw, double dpitch, double droll)
	{
		// no input, static friction makes the robot stay put
		if (dx == 0 && dy == 0 && dz == 0 && dyaw == 0 && dpitch == 0 && droll == 0) {
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
		State       = this.State.Add(time.ElapsedGameTime.TotalSeconds.Multiply(
		                                  U.RandomGaussianVector(new double[7] {0, 0, 0, 0, 0, 0, 0}, MotionCovariance)));
		Orientation = Quaternion.Normalize(this.Orientation);

		if (Location.Subtract(Waypoints[Waypoints.Count - 1]).SquareEuclidean() >= 1e-2f) {
			Waypoints.Add(Location);
		}
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
		double[]   diff  = landmark.Subtract(this.Location);
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
	public List<double[]> Measure()
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
		int nclutter = Math.Min(clutterGen.Generate(), (int)(ClutterCount * 10));

		for (int i = 0; i < nclutter; i++) {
			double px    = U.uniform.Next() * FilmArea.Width + FilmArea.Left;
			double py    = U.uniform.Next() * FilmArea.Height + FilmArea.Top;
			double range = U.uniform.Next() * RangeClip.Length + RangeClip.Min;
			measurements.Add(new double[3] {px, py, range});
		}

		return measurements;
	}

	/// <summary>
	/// Obtain the jacobian of the measurement model.
	/// Designed to be used with EKF filters.
	/// </summary>
	/// <param name="landmark">Landmark 3d location against which the measurement is performed.</param>
	/// <returns>Measurement model linearization jacobian.</returns>
	public double[,] MeasurementJacobian(double[] landmark)
	{
		double[]   diff  = landmark.Subtract(this.Location);
		Quaternion local = Quaternion.Conjugate(Orientation) *
			                    new Quaternion((float) diff[0], (float) diff[1], (float) diff[2], 0) * Orientation;

		// the jacobian of the homography projection part is given by
		// f/z I 0
		//  X/|X|
		double mag = Math.Sqrt(local.X * local.X + local.Y * local.Y + local.Z * local.Z);
		double[,] jprojection = {{VisionFocal / local.Z, 0,                     -VisionFocal * local.X / (local.Z * local.Z)},
		                         {0,                     VisionFocal / local.Z, -VisionFocal * local.Y / (local.Z * local.Z)},
		                         {local.X / mag,         local.Y / mag,         local.Z /mag}};

		// the jacobian of the change of coordinates part of the measurement process is the rotation matrix 
		double[,] jrotation = ME.MatrixFromQuaternion(Orientation);

		return jprojection.Multiply(jrotation);
	}

	/// <summary>
	/// Find if a given ladmark is visible from the current pose of the vehicle.
	/// </summary>
	/// <param name="landmark">Queried landmark.</param>
	/// <returns>True if the landmark is visible; false otherwise.</returns>
	public bool Visible(double[] landmark)
	{
		double[]   diff  = landmark.Subtract(this.Location);
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
	/// Transform a measurement vector in measure-space (pixel-range)
	/// into a map-space vector  (x-y plane)
	/// </summary>
	/// <param name="measurement">Measurement expressed as pixel-range.</param>
	/// <returns>Measurement expressed in x-y plane</returns>
	public double[] MeasureToMap(double[] measurement)
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
	/// Render the vehicle on the graphics device.
	/// The graphics device must be ready, otherwise
	/// the method will throw an exception.
	/// <param name="camera">Camera rotation matrix.</param>
	/// </summary>
	public void Render(double[,] camera)
	{
		RenderFOV(camera);
		RenderTrajectory(camera);
		RenderBody(camera);
	}

	/// <summary>
	/// Render the vehicle physical body on the graphics device.
	/// <param name="camera">Camera rotation matrix.</param>
	/// </summary>
	public void RenderBody(double[,] camera)
	{
		const float halflen = 0.06f;
		
		Color innercolor =  Color.LightBlue;
		Color outercolor =  Color.Blue;

		double[] pos = camera.Multiply(Location);
		
		VertexPositionColor[] invertices  = new VertexPositionColor[4];
		double[][]            outvertices = new double[4][];

		outvertices[0] = new double[] {pos[0] - halflen, pos[1] - halflen, pos[2]};
		outvertices[1] = new double[] {pos[0] - halflen, pos[1] + halflen, pos[2]};
		outvertices[2] = new double[] {pos[0] + halflen, pos[1] + halflen, pos[2]};
		outvertices[3] = new double[] {pos[0] + halflen, pos[1] - halflen, pos[2]};

		invertices[0] = new VertexPositionColor(outvertices[0].ToVector3(), innercolor);
		invertices[1] = new VertexPositionColor(outvertices[1].ToVector3(), innercolor);
		invertices[2] = new VertexPositionColor(outvertices[3].ToVector3(), innercolor);
		invertices[3] = new VertexPositionColor(outvertices[2].ToVector3(), innercolor);

		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, invertices, 0, invertices.Length - 2);
		Graphics.DrawUser2DPolygon(outvertices, 0.02f, outercolor, true);
		
		Quaternion x = Orientation * new Quaternion(0.2f, 0, 0, 0) * Quaternion.Conjugate(Orientation);
		pos = camera.Multiply(new double[3] {X + x.X, Y + x.Y, Z + x.Z});

		outvertices[0] = new double[] {pos[0] - 0.4*halflen, pos[1] - 0.4*halflen, pos[2]};
		outvertices[1] = new double[] {pos[0] - 0.4*halflen, pos[1] + 0.4*halflen, pos[2]};
		outvertices[2] = new double[] {pos[0] + 0.4*halflen, pos[1] + 0.4*halflen, pos[2]};
		outvertices[3] = new double[] {pos[0] + 0.4*halflen, pos[1] - 0.4*halflen, pos[2]};

		invertices[0] = new VertexPositionColor(outvertices[0].ToVector3(), innercolor);
		invertices[1] = new VertexPositionColor(outvertices[1].ToVector3(), innercolor);
		invertices[2] = new VertexPositionColor(outvertices[3].ToVector3(), innercolor);
		invertices[3] = new VertexPositionColor(outvertices[2].ToVector3(), innercolor);

		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, invertices, 0, invertices.Length - 2);
		Graphics.DrawUser2DPolygon(outvertices, 0.02f, Color.Blue, true);
		
		x   = Orientation * new Quaternion(0, 0.2f, 0, 0) * Quaternion.Conjugate(Orientation);
		pos = camera.Multiply(new double[3] {X + x.X, Y + x.Y, Z + x.Z});

		outvertices[0] = new double[] {pos[0] - 0.2*halflen, pos[1] - 0.2*halflen, pos[2]};
		outvertices[1] = new double[] {pos[0] - 0.2*halflen, pos[1] + 0.2*halflen, pos[2]};
		outvertices[2] = new double[] {pos[0] + 0.2*halflen, pos[1] + 0.2*halflen, pos[2]};
		outvertices[3] = new double[] {pos[0] + 0.2*halflen, pos[1] - 0.2*halflen, pos[2]};

		invertices[0] = new VertexPositionColor(outvertices[0].ToVector3(), innercolor);
		invertices[1] = new VertexPositionColor(outvertices[1].ToVector3(), innercolor);
		invertices[2] = new VertexPositionColor(outvertices[3].ToVector3(), innercolor);
		invertices[3] = new VertexPositionColor(outvertices[2].ToVector3(), innercolor);

		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, invertices, 0, invertices.Length - 2);
		Graphics.DrawUser2DPolygon(outvertices, 0.02f, Color.Yellow, true);
		
		x   = Orientation * new Quaternion(0, 0, 0.2f, 0) * Quaternion.Conjugate(Orientation);
		pos = camera.Multiply(new double[3] {X + x.X, Y + x.Y, Z + x.Z});

		outvertices[0] = new double[] {pos[0] - 0.8*halflen, pos[1] - 0.8*halflen, pos[2]};
		outvertices[1] = new double[] {pos[0] - 0.8*halflen, pos[1] + 0.8*halflen, pos[2]};
		outvertices[2] = new double[] {pos[0] + 0.8*halflen, pos[1] + 0.8*halflen, pos[2]};
		outvertices[3] = new double[] {pos[0] + 0.8*halflen, pos[1] - 0.8*halflen, pos[2]};

		invertices[0] = new VertexPositionColor(outvertices[0].ToVector3(), innercolor);
		invertices[1] = new VertexPositionColor(outvertices[1].ToVector3(), innercolor);
		invertices[2] = new VertexPositionColor(outvertices[3].ToVector3(), innercolor);
		invertices[3] = new VertexPositionColor(outvertices[2].ToVector3(), innercolor);

		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, invertices, 0, invertices.Length - 2);
		Graphics.DrawUser2DPolygon(outvertices, 0.02f, Color.Red, true);
	}

	/// <summary>
	/// Render the Field-of-View cone on the graphics device.
	/// </summary>
	/// <param name="camera">Camera rotation matrix.</param>
	public void RenderFOV(double[,] camera)
	{
		
		Color incolorA = Color.LightGreen; incolorA.A = 30;
		Color incolorB = Color.LightGreen; incolorB.A = 15;
		Color outcolor = Color.DarkGreen;  outcolor.A = 30;

		double[][] frustum = new double[8][];
		
		frustum[0] = new double[3] {RangeClip.Min * FilmArea.Left  / VisionFocal, RangeClip.Min * FilmArea.Top    / VisionFocal, RangeClip.Min};
		frustum[1] = new double[3] {RangeClip.Min * FilmArea.Right / VisionFocal, RangeClip.Min * FilmArea.Top    / VisionFocal, RangeClip.Min};
		frustum[2] = new double[3] {RangeClip.Min * FilmArea.Right / VisionFocal, RangeClip.Min * FilmArea.Bottom / VisionFocal, RangeClip.Min};
		frustum[3] = new double[3] {RangeClip.Min * FilmArea.Left  / VisionFocal, RangeClip.Min * FilmArea.Bottom / VisionFocal, RangeClip.Min};
		frustum[4] = new double[3] {RangeClip.Max * FilmArea.Left  / VisionFocal, RangeClip.Max * FilmArea.Top    / VisionFocal, RangeClip.Max};
		frustum[5] = new double[3] {RangeClip.Max * FilmArea.Right / VisionFocal, RangeClip.Max * FilmArea.Top    / VisionFocal, RangeClip.Max};
		frustum[6] = new double[3] {RangeClip.Max * FilmArea.Right / VisionFocal, RangeClip.Max * FilmArea.Bottom / VisionFocal, RangeClip.Max};
		frustum[7] = new double[3] {RangeClip.Max * FilmArea.Left  / VisionFocal, RangeClip.Max * FilmArea.Bottom / VisionFocal, RangeClip.Max};

		for (int i = 0; i < frustum.Length; i++) {
			Quaternion local = Orientation *
			                       new Quaternion((float) frustum[i][0], (float) frustum[i][1], (float) frustum[i][2], 0) * Quaternion.Conjugate(Orientation);
			frustum[i] = camera.Multiply(new double[3] {local.X, local.Y, local.Z}.Add(Location));
			frustum[i][2] = -100;
		}
		
		VertexPositionColor[] verticesA = new VertexPositionColor[8];
		VertexPositionColor[] verticesB = new VertexPositionColor[8];
		double[][]            wireA     = new double[4][];
		double[][]            wireB     = new double[4][];
		double[][]            wireC     = new double[4][];
		double[][]            wireD     = new double[4][];
		
		wireA[0] = frustum[0];
		wireA[1] = frustum[1];
		wireA[2] = frustum[5];
		wireA[3] = frustum[6];

		wireB[0] = frustum[1];
		wireB[1] = frustum[2];
		wireB[2] = frustum[6];
		wireB[3] = frustum[7];
		
		wireC[0] = frustum[2];
		wireC[1] = frustum[3];
		wireC[2] = frustum[7];
		wireC[3] = frustum[4];

		wireD[0] = frustum[3];
		wireD[1] = frustum[0];
		wireD[2] = frustum[4];
		wireD[3] = frustum[5];

		verticesA[0] = new VertexPositionColor(frustum[0].ToVector3(), incolorA);
		verticesA[1] = new VertexPositionColor(frustum[4].ToVector3(), incolorA);
		verticesA[2] = new VertexPositionColor(frustum[1].ToVector3(), incolorA);
		verticesA[3] = new VertexPositionColor(frustum[5].ToVector3(), incolorA);
		verticesA[4] = new VertexPositionColor(frustum[2].ToVector3(), incolorA);
		verticesA[5] = new VertexPositionColor(frustum[6].ToVector3(), incolorA);
		verticesA[6] = new VertexPositionColor(frustum[3].ToVector3(), incolorA);
		verticesA[7] = new VertexPositionColor(frustum[7].ToVector3(), incolorA);

		verticesB[0] = new VertexPositionColor(frustum[1].ToVector3(), incolorB);
		verticesB[1] = new VertexPositionColor(frustum[5].ToVector3(), incolorB);
		verticesB[2] = new VertexPositionColor(frustum[2].ToVector3(), incolorB);
		verticesB[3] = new VertexPositionColor(frustum[6].ToVector3(), incolorB);
		verticesB[4] = new VertexPositionColor(frustum[3].ToVector3(), incolorB);
		verticesB[5] = new VertexPositionColor(frustum[7].ToVector3(), incolorB);
		verticesB[6] = new VertexPositionColor(frustum[0].ToVector3(), incolorB);
		verticesB[7] = new VertexPositionColor(frustum[4].ToVector3(), incolorB);
		
		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, verticesA, 0, 2);
		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, verticesA, 4, 2);
		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, verticesB, 0, 2);
		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, verticesB, 4, 2);
		
		Graphics.DrawUser2DPolygon(wireA, 0.02f, outcolor, false);
		Graphics.DrawUser2DPolygon(wireB, 0.02f, outcolor, false);
		Graphics.DrawUser2DPolygon(wireC, 0.02f, outcolor, false);
		Graphics.DrawUser2DPolygon(wireD, 0.02f, outcolor, false);
	}

	/// <summary>
	/// Render the path that the vehicle has traveled so far.
	/// </summary>
	/// <param name="camera">Camera rotation matrix.</param>
	public void RenderTrajectory(double[,] camera)
	{
		double[][] vertices = new double[Waypoints.Count][];
		Color      color    = Color.Yellow;

		for (int i = 0; i < Waypoints.Count; i++) {
			vertices[i] = camera.Multiply(Waypoints[i]);
		}

		Graphics.DrawUser2DPolygon(vertices, 0.02f, color, false);
	}
}
}
