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
/// It uses a 2d odometry motion model and
/// a range-bearing measurement model.
/// </summary>
public class Vehicle
{
	/// <summary>
	/// Motion model covariance matrix.
	/// </summary>
	public readonly double[,] MotionCovariance = new double[3, 3] {{5e-3, 0, 0},
	                                                               {0, 5e-3, 0},
	                                                               {0, 0, 5e-3}};

	/// <summary>
	/// Measurement model covariance matrix.
	/// </summary>
	public readonly double[,] MeasurementCovariance = new double[2, 2] {{2e-4, 0},
	                                                                    {0, 2e-4}};

	/// <summary>
	/// Probability of detection.
	/// </summary>
	private readonly double detectionProbability = 0.95;

	/// <summary>
	/// Vision range in radians, measured from straight ahead orientation.
	/// </summary>
	public readonly Range VisionRange = new Range(-0.5f, 0.5f);

	/// <summary>
	/// Cached vision range angle.
	/// </summary>
	public readonly double VisionAngle = 1.0f;

	/// <summary>
	/// Internal state as a vector.
	/// </summary>
	public double[] State { get; private set; }

	/// <summary>
	/// Location x-axis coordinate.
	/// </summary>
	public double X
	{
		get { return  State[0]; }
		private set { State[0] = value; }
	}

	/// <summary>
	/// Location y-axis coordinate.
	/// </summary>
	public double Y
	{
		get { return  State[1]; }
		private set { State[1] = value; }
	}

	/// <summary>
	/// Get the space location of the vehicle, i.e. its coordinates
	/// </summary>
	public double[] Location
	{
		get { return new double[2] {State[0], State[1]}; }
	}

	/// <summary>
	/// Orientation angle measured counterclockwise from (1, 0) in radians.
	/// </summary>
	public double Theta
	{
		get { return  State[2]; }
		private set { State[2] = value; }
	}

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
	/// <param name="x">X coordinate.</param>
	/// <param name="y">Y coordinate.</param>
	/// <param name="theta">Orientation.</param>
	public Vehicle(double x, double y, double theta)
	{
		this.State       = new double[3] {x, y, theta};
		this.VisionAngle = this.VisionRange.Max - this.VisionRange.Min;
		this.Waypoints   = new List<double[]>();
		this.Waypoints.Add(new double[2] {x, y});
	}

	/// <summary>
	/// Copy constructor. Performs a deep copy of another vehicle.
	/// </summary>
	/// <param name="that">Copied vehicle</param>
	/// <param name="copytrajectory">If true, the vehicle historic trajectory is copied. Relatively heavy operation.</param>
	public Vehicle(Vehicle that, bool copytrajectory = false)
	{
		this.State                 = new double[3] {that.X, that.Y, that.Theta};
		this.VisionRange           = new Range(that.VisionRange.Min, that.VisionRange.Max);
		this.VisionAngle           = that.VisionAngle;
		this.detectionProbability  = that.detectionProbability;
		this.MotionCovariance      = that.MotionCovariance.MemberwiseClone();
		this.MeasurementCovariance = that.MeasurementCovariance.MemberwiseClone();
		this.Graphics              = that.Graphics;

		if (copytrajectory) {
			this.Waypoints = new List<double[]>(that.Waypoints);
		}
		else {
			this.Waypoints = new List<double[]>();
			this.Waypoints.Add(new double[2] {that.X, that.Y});
		}
	}

	/// <summary>
	/// Apply the motion model to the vehicle. It corresponds to a
	/// 2D odometry model following the equation:
	/// 
	/// x = x + Rot(theta) dx + N(0, Q)
	/// 
	/// where Rot() is a rotation matrix and N(a, b) is a normal function
	/// with mean 'a' and covariance matrix 'b'.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="dx">Moved distance from odometry in the local parallel direction since last timestep.</param>
	/// <param name="dy">Moved distance from odometry in the local perpendicular direction since last timestep.</param>
	/// <param name="dtheta">Angle variation from odometry since last timestep.</param>
	public void Update(GameTime time, double dx, double dy, double dtheta)
	{
		// no input, static friction makes the robot stay put
		if (dx == 0 && dy == 0 && dtheta == 0) {
			return;
		}

		this.State = this.State.Add(ME.Rotation2H(this.Theta + dtheta / 2).Multiply(new double[] {dx, dy, dtheta}));
		this.State = this.State.Add(time.ElapsedGameTime.TotalSeconds.Multiply(
		                            U.RandomGaussianVector(new double[3] {0, 0, 0}, MotionCovariance)));

		if (Location.Subtract(Waypoints[Waypoints.Count - 1]).SquareEuclidean() >= 1e-2f) {
			Waypoints.Add(Location);
		}
	}

	/// <summary>
	/// Obtain a measurement from the hidden state. It follows a
	/// range-bearing model where the range and bearing from a
	/// map landmark m are:
	/// 
	/// [r, b] = [|m-x|, atan(dy/dx) - theta] + N(0, R)
	/// 
	/// where |a-b| is the euclidean distance between a and b,
	/// dy/dx is the slope of the line joining x and m.
	/// This method always detects the landmark (misdetection
	/// probability is ignored).
	/// </summary>
	/// <param name="landmark">Landmark 2d location against which the measurement is performed.</param>
	/// <returns>Range-bearing measurements.</returns>
	public double[] MeasureDetected(double[] landmark)
	{
		return MeasurePerfect(landmark).Add(U.RandomGaussianVector(new double[2] {0, 0}, MeasurementCovariance));
	}

	/// <summary>
	/// Obtain several measurements from the hidden state.
	/// This method always detects the landmark (misdetection
	/// probability is ignored).
	/// </summary>
	/// <param name="landmarks">Landmark 2d locations against which the measurements are performed.</param>
	/// <returns>Range-bearing measurements.</returns>
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
	/// <param name="landmarks">Landmark 2d locations against which the measurements are performed.</param>
	/// <returns>Range-bearing measurements.</returns>
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
	/// <param name="landmark">Landmark 2d location against which the measurement is performed.</param>
	/// <returns>Measurement model linearization jacobian.</returns>
	public double[,] MeasurementJacobian(double[] landmark)
	{
		double[] diff = landmark.Subtract(this.Location);
		double   mag  = diff.Euclidean();
		double   den  = diff[1] * diff[1] + diff[0] * diff[0];

		return new double[2, 2] {{diff[0] / mag, diff[1] / mag}, {-diff[1]/den, diff[0]/den}};
	}
	
	/// <summary>
	/// Obtain a measurement from the hidden state.
	/// It does not use any randomness or misdetection, 
	/// as it is designed to be used with UKF filters.
	/// </summary>
	/// <param name="landmark">Landmark 2d location against which the measurement is performed.</param>
	/// <returns>Range-bearing measurements.</returns>
	
	public double[] MeasurePerfect(double[] landmark)
	{
		double[] diff  = landmark.Subtract(this.Location);
		double range   = diff.Euclidean();
		double bearing = Math.Atan2(diff[1], diff[0]) - this.Theta;

		return new double[2] {range, U.NormalizeAngle(bearing)};
	
	}

	/// <summary>
	/// Find if a given ladmark is visible from the current pose of the vehicle.
	/// </summary>
	/// <param name="landmark">Queried landmark.</param>
	/// <returns>True if the landmark is visible; false otherwise.</returns>
	public bool Visible(double[] landmark)
	{
		double[] diff  = landmark.Subtract(this.Location);
		double bearing = Math.Atan2(diff[1], diff[0]) - this.Theta;

		double anglediff = bearing - VisionRange.Min;

		return U.NormalizeAngle2(anglediff) < VisionAngle;
	}

	/// <summary>
	/// Find if a given ladmark is visible from the current pose of the vehicle
	/// using range-bearing coordinates to express the landmark.
	/// </summary>
	/// <param name="measurement">Queried landmark in range-bearing coordinates.</param>
	/// <returns>True if the landmark is visible; false otherwise.</returns>
	public bool VisibleM(double[] measurement)
	{
		double anglediff = measurement[1] - VisionRange.Min;

		return U.NormalizeAngle2(anglediff) < VisionAngle;
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
	/// using range-bearing coordinates to express the landmark.
	/// It is modelled as a constant if it's on the FOV and zero if not.
	/// </summary>
	/// <param name="measurement">Queried landmark in range-bearing coordinates.</param>
	/// <returns></returns>
	public double DetectionProbabilityM(double[] measurement)
	{
		return VisibleM(measurement) ? detectionProbability : 0;
	}

	/// <summary>
	/// Transform a measurement vector in measure-space (range-bearing)
	/// into a map-space vector  (x-y plane)
	/// </summary>
	/// <param name="measurement">Measurement expressed as range-bearing.</param>
	/// <returns>Measurement expressed in x-y plane</returns>
	public double[] MeasureToMap(double[] measurement)
	{
		return new double[2] {X + measurement[0] * Math.Cos(measurement[1] + Theta), Y + measurement[0] * Math.Sin(measurement[1] + Theta)};
	}

	/// <summary>
	/// Render the vehicle on the graphics device.
	/// The graphics device must be ready, otherwise
	/// the method will throw an exception.
	/// </summary>
	public void Render()
	{
		RenderTrajectory();
		RenderFOV();
		RenderBody();
	}

	/// <summary>
	/// Render the vehicle physical body on the graphics device.
	/// </summary>
	public void RenderBody()
	{
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
		Graphics.DrawUser2DPolygon(outvertices, 0.02f, outercolor, true);
	}

	/// <summary>
	/// Render the Field-of-View cone on the graphics device.
	/// </summary>
	public void RenderFOV()
	{
		VertexPositionColor[] vertices  = new VertexPositionColor[3];
		
		const double radius = 100;
		
		Color innercolor =  Color.LightGreen; innercolor.A = 30;
		Color outercolor =  Color.DarkGreen;  outercolor.A = 30;
		
		double ct1 = Math.Cos(Theta + VisionRange.Min); double st1 = Math.Sin(Theta + VisionRange.Min);
		double ct2 = Math.Cos(Theta + VisionRange.Max); double st2 = Math.Sin(Theta + VisionRange.Max);
		
		vertices[0] = new VertexPositionColor(new Vector3((float) X,                 (float) Y,                 0), innercolor);
		vertices[1] = new VertexPositionColor(new Vector3((float)(X + radius * ct1), (float)(Y + radius * st1), 0), innercolor);
		vertices[2] = new VertexPositionColor(new Vector3((float)(X + radius * ct2), (float)(Y + radius * st2), 0), innercolor);
		
		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, vertices, 0, vertices.Length - 2);
		Graphics.DrawUser2DPolygon(vertices, 0.02f, outercolor, true);
	}

	/// <summary>
	/// Render the path that the vehicle has travelled so far.
	/// </summary>
	public void RenderTrajectory()
	{
		VertexPositionColor[] vertices = new VertexPositionColor[Waypoints.Count];
		Color color = Color.Yellow;

		for (int i = 0; i < Waypoints.Count; i++) {
			vertices[i] = new VertexPositionColor(new Vector3((float) Waypoints[i][0], (float) Waypoints[i][1], 0), color);
		}

		Graphics.DrawUser2DPolygon(vertices, 0.02f, color, false);
	}
}
}
