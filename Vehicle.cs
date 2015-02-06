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
	public readonly double[,] MeasurementCovariance = new double[3, 3] {{1e1,  0, 0},
	                                                                    {0,  1e1, 0},
	                                                                    {0, 0, 2e-3}};

	/// <summary>
	/// Probability of detection.
	/// </summary>
	private readonly double detectionProbability = 0.85;

	/// <summary>
	/// Get the internal state as a vector.
	/// </summary>
	public double[] State { get; private set; }

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
	/// Vision camera focal length.
	/// </summary>
	public const double VisionFocal = 480;

	/// <summary>
	/// 2D film clipping area (i.e. sensor size and offset).
	/// </summary>
	public readonly Rectangle FilmArea = new Rectangle(-320, -240, 640, 480); 

	/// <summary>
	/// Render output.
	/// </summary>
	public GraphicsDevice Graphics { get; set; }

	/// <summary>
	/// Trajectory through which the vehicle has moved.
	/// </summary>
	public List<double[]> Waypoints { get; private set; }

	/// <summary>
	/// Construct a new Vehicle object from its initial state.
	/// </summary>
	/// <param name="location">Spatial coordinates.</param>
	/// <param name="theta">Orientation angle.</param>
	/// <param name="axis">Orientation rotation axis.</param>
	public Vehicle(double[] location, double theta, double[] axis)
	{
		double w = Math.Cos(theta / 2);
		double d = Math.Sin(theta / 2);

		axis.Divide(axis.Euclidean(), true);

		this.State       = new double[7] {location[0], location[1], location[2], w, d * axis[0], d * axis[1], d * axis[2]};
		this.Waypoints   = new List<double[]>();
		this.Waypoints.Add(location);
	}

	/// <summary>
	/// Copy constructor. Performs a deep copy of another vehicle.
	/// </summary>
	/// <param name="that">Copied vehicle</param>
	/// <param name="copytrajectory">If true, the vehicle historic trajectory is copied. Relatively heavy operation.</param>
	public Vehicle(Vehicle that, bool copytrajectory = false)
	{
		this.State                 = new double[7];
		that.State.CopyTo(this.State, 0);

		this.detectionProbability  = that.detectionProbability;
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
		Quaternion dorientation = Quaternion.CreateFromYawPitchRoll((float) dyaw, (float) dpitch, (float) droll);

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
	/// <param name="landmarks">Landmark 3d locations against which the measurements are performed.</param>
	/// <returns>Pixel-range measurements.</returns>
	public double[][] MeasureDetected(List<double[]> landmarks)
	{
		double[][] measurements = new double[landmarks.Count][];

		for (int i = 0; i < landmarks.Count; i++) {
			measurements[i] = MeasureDetected(landmarks[i]);
		}

		return measurements;
	}

	/// <summary>
	/// Obtain several measurements from the hidden state.
	/// Ladmarks may be misdetected.
	/// </summary>
	/// <param name="landmarks">Landmark 3d locations against which the measurements are performed.</param>
	/// <returns>Pixel-range measurements.</returns>
	public List<double[]> Measure(List<double[]> landmarks)
	{
		List<double[]> measurements = new List<double[]>();
		List<double[]> landmarklist = new List<double[]>();
		double[][]     measurementlist;

		foreach (double[] landmark in landmarks) {
			if (Visible(landmark)) {
				landmarklist.Add(landmark);
			}
		}

		measurementlist = MeasureDetected(landmarklist);

		// add every measurement with probability = DetectionProbbility
		for (int i = 0; i < landmarklist.Count; i++) {
			if (U.uniform.Next() < detectionProbability) {
				measurements.Add(measurementlist[i]);
			}
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

		if (local.Z <= 0) {
			return false;
		}

		return local.Z > 0 && FilmArea.Left < px && px < FilmArea.Right && FilmArea.Top < py && py < FilmArea.Bottom;
	}

	/// <summary>
	/// Find if a given ladmark is visible from the current pose of the vehicle
	/// using pixel-range coordinates to express the landmark.
	/// </summary>
	/// <param name="measurement">Queried landmark in pixel-range coordinates.</param>
	/// <returns>True if the landmark is visible; false otherwise.</returns>
	public bool VisibleM(double[] measurement)
	{
		return FilmArea.Left < measurement[1] && measurement[1] < FilmArea.Right &&
		       FilmArea.Top  < measurement[2] && measurement[2] < FilmArea.Bottom;
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

		/*landmark = camera.Multiply(landmark);

		VertexPositionColor[] invertices  = new VertexPositionColor[7];
		VertexPositionColor[] outvertices = new VertexPositionColor[7];
		double[][] vertpos = new double[7][];
		
		const double radius    = 0.08;
		const double baseext   = 0.03;
		const double basewidth = 0.035;
		
		Color innercolor =  Color.White;
		Color outercolor =  Color.Black;
		
		double ct0 = Math.Cos(Theta);                   double st0 = Math.Sin(Theta);
		double ct1 = Math.Cos(Theta + 2 * Math.PI / 3); double st1 = Math.Sin(Theta + 2 * Math.PI / 3);
		double ct2 = Math.Cos(Theta - 2 * Math.PI / 3); double st2 = Math.Sin(Theta - 2 * Math.PI / 3);
		
		vertpos[0] = new double[2] {X + radius * ct0, Y + radius * st0};
		vertpos[1] = new double[2] {X + radius * ct1, Y + radius * st1};
		vertpos[2] = new double[2] {X + radius * ct2, Y + radius * st2};

		double[] baseparallel = new double[2] {vertpos[2][0] - vertpos[1][0],
		                                       vertpos[2][1] - vertpos[1][1]};

		baseparallel.Divide(baseparallel.Euclidean(), true);

		double[] baseperpendicular = new double[2] {baseparallel[1], -baseparallel[0]};
		
		vertpos[3] = vertpos[1].Subtract(baseext.Multiply(baseparallel));
		vertpos[4] = vertpos[2].Add     (baseext.Multiply(baseparallel));

		vertpos[5] = vertpos[3].Add(basewidth.Multiply(baseperpendicular));
		vertpos[6] = vertpos[4].Add(basewidth.Multiply(baseperpendicular));
		
		for (int i = 0, k = 0; k < vertpos.Length; i++, k += 2) {
			outvertices[i] = new VertexPositionColor(new Vector3((float) vertpos[k][0], (float) vertpos[k][1], 0), outercolor);
		}

		for (int i = vertpos.Length - 1, k = 1; k < vertpos.Length; i--, k += 2) {
			outvertices[i] = new VertexPositionColor(new Vector3((float) vertpos[k][0], (float) vertpos[k][1], 0), outercolor);
		}

		for (int i = 0; i < vertpos.Length; i++) {
			invertices[i] = new VertexPositionColor(new Vector3((float) vertpos[i][0], (float) vertpos[i][1], 0), innercolor);
		}
		
		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, invertices, 0, invertices.Length - 2);
		Graphics.DrawUser2DPolygon(outvertices, 0.02f, outercolor, true);*/
	}

	/// <summary>
	/// Render the Field-of-View cone on the graphics device.
	/// </summary>
	/// <param name="camera">Camera rotation matrix.</param>
	public void RenderFOV(double[,] camera)
	{
		const double depth = 10;
		
		Color incolorA = Color.LightGreen; incolorA.A = 30;
		Color incolorB = Color.LightGreen; incolorB.A = 15;
		Color outcolor = Color.DarkGreen;  outcolor.A = 30;

		double[][] frustum = new double[5][];
		
		frustum[0] = new double[3] {0, 0, 0};
		frustum[1] = new double[3] {depth * FilmArea.Left  / VisionFocal, depth * FilmArea.Top    / VisionFocal, depth};
		frustum[2] = new double[3] {depth * FilmArea.Right / VisionFocal, depth * FilmArea.Top    / VisionFocal, depth};
		frustum[3] = new double[3] {depth * FilmArea.Right / VisionFocal, depth * FilmArea.Bottom / VisionFocal, depth};
		frustum[4] = new double[3] {depth * FilmArea.Left  / VisionFocal, depth * FilmArea.Bottom / VisionFocal, depth};

		for (int i = 0; i < frustum.Length; i++) {
			Quaternion local = Orientation *
			                       new Quaternion((float) frustum[i][0], (float) frustum[i][1], (float) frustum[i][2], 0) * Quaternion.Conjugate(Orientation);
			frustum[i] = camera.Multiply(new double[3] {local.X, local.Y, local.Z}.Add(Location));
			frustum[i][2] = -100;
		}
		
		VertexPositionColor[] verticesA = new VertexPositionColor[6];
		VertexPositionColor[] verticesB = new VertexPositionColor[6];
		double[][]            wireA     = new double[3][];
		double[][]            wireB     = new double[3][];
		
		wireA[0] = frustum[1];
		wireA[1] = frustum[0];
		wireA[2] = frustum[3];

		wireB[0] = frustum[2];
		wireB[1] = frustum[0];
		wireB[2] = frustum[4];

		verticesA[0] = new VertexPositionColor(frustum[0].ToVector3(), incolorA);
		verticesA[1] = new VertexPositionColor(frustum[1].ToVector3(), incolorA);
		verticesA[2] = new VertexPositionColor(frustum[2].ToVector3(), incolorA);
		verticesA[3] = new VertexPositionColor(frustum[0].ToVector3(), incolorA);
		verticesA[4] = new VertexPositionColor(frustum[3].ToVector3(), incolorA);
		verticesA[5] = new VertexPositionColor(frustum[4].ToVector3(), incolorA);

		verticesB[0] = new VertexPositionColor(frustum[0].ToVector3(), incolorB);
		verticesB[1] = new VertexPositionColor(frustum[2].ToVector3(), incolorB);
		verticesB[2] = new VertexPositionColor(frustum[3].ToVector3(), incolorB);
		verticesB[3] = new VertexPositionColor(frustum[0].ToVector3(), incolorB);
		verticesB[4] = new VertexPositionColor(frustum[4].ToVector3(), incolorB);
		verticesB[5] = new VertexPositionColor(frustum[1].ToVector3(), incolorB);
		
		Graphics.DrawUserPrimitives(PrimitiveType.TriangleList, verticesA, 0, 2);
		Graphics.DrawUserPrimitives(PrimitiveType.TriangleList, verticesB, 0, 2);
		Graphics.DrawUser2DPolygon(wireA, 0.02f, outcolor, false);
		Graphics.DrawUser2DPolygon(wireB, 0.02f, outcolor, false);
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
