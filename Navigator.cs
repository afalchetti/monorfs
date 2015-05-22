// Navigator.cs
// Abstract SLAM solving navigator
// Part of MonoRFS
//
// Copyright (C) 2015 Angelo Falchetti
// All rights reserved.
// Any use, reproduction, distribution or copy of this
// work via any medium is strictly forbidden without
// express written consent from the author.

using System;
using System.Collections.Generic;
using System.Text;
using Accord.Math;
using Accord.Math.Decompositions;
using AForge;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using NUnit.Framework;
using System.Timers;

using TimedState    = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedMapModel = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<monorfs.Gaussian>>>;

namespace monorfs
{
/// <summary>
/// Abstract SLAM solver.
/// </summary>
public abstract class Navigator
{
	/// <summary>
	/// Reference vehicle. Only used if mapping mode is enabled (no localization).
	/// </summary>
	public Vehicle RefVehicle { get; private set; }

	/// <summary>
	/// Estimated trajectory.
	/// </summary>
	public List<double[]> Waypoints { get; set; }

	/// <summary>
	/// Best map history.
	/// </summary>
	public TimedMapModel WayMaps { get; set; }

	/// <summary>
	/// Most accurate estimate of the current vehicle pose.
	/// </summary>
	public abstract SimulatedVehicle BestEstimate { get; set; }

	/// <summary>
	/// Most accurate estimate model of the map.
	/// </summary>
	public abstract List<Gaussian> BestMapModel { get; set; }

	/// <summary>
	/// True if the localization of the vehicle is perfectly known.
	/// </summary>
	public bool OnlyMapping { get; protected set; }

	/// <summary>
	/// Internal render output.
	/// </summary>
	protected GraphicsDevice graphics { get; set; }

	/// <summary>
	/// Render output.
	/// </summary>
	public virtual GraphicsDevice Graphics {
		get
		{
			return graphics;
		}
		set
		{
			graphics = value;
		}
	}

	/// <summary>
	/// Gaussian prediction sigma-interval ellipse normalized render model.
	/// </summary>
	private double[][] pinterval;

	/// <summary>
	/// Construct a Navigator using the indicated vehicle as a reference.
	/// </summary>
	/// <param name="vehicle">Vehicle to track.</param>
	/// <param name="onlymapping">If true, don't do SLAM, but mapping (i.e. the localization is assumed known).</param>
	public Navigator(Vehicle vehicle, bool onlymapping = false)
	{
		OnlyMapping = onlymapping;

		// a reference copy, which gives the navigator precise info about
		// the real vehicle pose
		RefVehicle = vehicle;

		Waypoints = new List<double[]>();
		Waypoints.Add(new double[4] {0, vehicle.X, vehicle.Y, vehicle.Z});

		WayMaps = new TimedMapModel();
		WayMaps.Add(Tuple.Create(0.0, new List<Gaussian>()));

		const int segments = 32;
		pinterval = new double[segments][];

		// pinterval will be the 5-sigma ellipse
		for (int i = 0; i < segments; i++) {
			this.pinterval[i] = new double[2] {5 * Math.Cos(2 * Math.PI * i / segments), 5*Math.Sin(2 * Math.PI * i / segments)};
		}
	}

	/// <summary>
	/// Change the mode of the navigator to solving full slam.
	/// </summary>
	public void StartSlam()
	{
		if (OnlyMapping) {
			OnlyMapping = false;
			StartSlamInternal();
		}
	}

	/// <summary>
	/// Do any additional work needed to start the slam mode.
	/// </summary>
	public virtual void StartSlamInternal() {}

	/// <summary>
	/// Change the mode of the navigator to do only mapping.
	/// The vehicle pose resets to the correct location, but the map is inherited
	/// from the previous best particle (MAP).
	/// </summary>
	public void StartMapping()
	{
		if (!OnlyMapping) {
			OnlyMapping = true;
			StartMappingInternal();
		}
	}

	/// <summary>
	/// Do any additional work needed to start the mapping mode.
	/// </summary>
	public virtual void StartMappingInternal() {}

	/// <summary>
	/// Reset any information about the map to nothing.
	/// </summary>
	public abstract void ResetMapModel();

	/// <summary>
	/// Remove all the localization and mapping history and start it again from the current position.
	/// </summary>
	public virtual void ResetHistory()
	{
		Waypoints.Clear();
		WayMaps  .Clear();
		
		Waypoints = new List<double[]>();
		Waypoints.Add(new double[4] {0, BestEstimate.X, BestEstimate.Y, BestEstimate.Z});

		WayMaps = new TimedMapModel();
		WayMaps.Add(Tuple.Create(0.0, new List<Gaussian>()));
	}

	/// <summary>
	/// Update the vehicle state.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="dx">Moved distance from odometry in the local vertical movement-perpendicular direction since last timestep.</param>
	/// <param name="dy">Moved distance from odometry in the local horizontal movement-perpendicular direction since last timestep.</param>
	/// <param name="dz">Moved distance from odometry in the local depth movement-parallel direction since last timestep.</param>
	/// <param name="dyaw">Angle variation from odometry in the yaw coordinate since last timestep.</param>
	/// <param name="dpitch">Angle variation from odometry in the pitch coordinate since last timestep.</param>
	/// <param name="droll">Angle variation from odometry in the roll coordinate since last timestep.</param>
	public abstract void Update(GameTime time, double dx, double dy, double dz,
	                           double dyaw, double dpitch, double droll);

	/// <summary>
	/// Update both the estimated map and the localization.
	/// This method is the core of the whole program.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="measurements">Sensor measurements in pixel-range form.</param>
	/// <param name="predict">Predict flag; if false, no prediction step is done.</param>
	/// <param name="correct">Correct flag; if false, no correction step is done.</param>
	/// <param name="prune">Prune flag; if false, no prune step is done.</param>
	public abstract void SlamUpdate(GameTime time, List<double[]> measurements,
	                                bool predict, bool correct, bool prune);
	// FIXME predict, correct, prune should NOT be arguments as it is not general to all Navigators
	//       and honestly, they are not used anymore, they were useful as debugging while coding this
	//       but now it's done and works perfectly (maybe add them as an optional alternative
	//       for PHDNavigator debugging).
	//       I think they can be removed (or put as configuration constants)
	
	/// <summary>
	/// Update the trajectory history with the
	/// current best estimate and a given time.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	protected void UpdateTrajectory(GameTime time)
	{
		Vehicle best = BestEstimate;
		Waypoints.Add(new double[4] {time.TotalGameTime.TotalSeconds, best.X, best.Y, best.Z});
	}
	
	/// <summary>
	/// Update the trajectory history with the
	/// current best map estimate and a given time.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	protected void UpdateMapHistory(GameTime time)
	{
		WayMaps.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, BestMapModel));
	}

	/// <summary>
	/// Render the navigation HUD and the trajectory on the graphics device.
	/// </summary>
	/// <remarks>
	/// The graphics device must be ready, otherwise
	/// the method will throw an exception.
	/// </remarks>
	/// <param name="camera">Camera rotation matrix.</param>
	public virtual void Render(double[][] camera)
	{
		RenderTrajectory(camera);
		RenderMap(camera);
	}

	/// <summary>
	/// Render the map model, i.e. its estimated landmarks and covariances.
	/// </summary>
	/// <param name="camera">Camera rotation matrix.</param>
	public void RenderMap(double[][] camera) {
		foreach (Gaussian component in BestMapModel) {
			RenderGaussian(component, camera);
		}
	}

	/// <summary>
	/// Render estimate of the path that the vehicle has traveled so far.
	/// </summary>
	/// <param name="camera">Camera rotation matrix.</param>
	public void RenderTrajectory(double[][] camera)
	{
		double[][] vertices = new double[Waypoints.Count][];
		Color color = Color.Blue;

		for (int i = 0; i < Waypoints.Count; i++) {
			vertices[i] = camera.Multiply(new double[3] {Waypoints[i][1], Waypoints[i][2], Waypoints[i][3]});
		}

		Graphics.DrawUser2DPolygon(vertices, 0.02f, color, false);
	}

	/// <summary>
	/// Render the 5-sigma ellipse of gaussian.
	/// </summary>
	/// <param name="gaussian">Gaussian to be rendered.</param>
	/// <param name="camera">Camera rotation matrix.</param>
	public void RenderGaussian(Gaussian gaussian, double[][] camera)
	{
		Color incolor  = Color.DeepSkyBlue; incolor.A  = 200;
		Color outcolor = Color.Blue;        outcolor.A = 200;
		
		double weight;
		
		if (gaussian.Weight < 1.0) {
			weight     = 0.04 * gaussian.Weight;
			incolor.A  = (byte) (200 * gaussian.Weight);
			outcolor.A = (byte) (200 * gaussian.Weight);
		}
		else if (gaussian.Weight < 2.0) {
			weight = 0.01 * (gaussian.Weight - 1) + 0.04;
			outcolor = Color.Black;
		}
		else {
			weight   = 0.04;
			outcolor = Color.Red;
		}

		double[][] covariance = camera.Multiply(gaussian.Covariance).MultiplyByTranspose(camera).Submatrix(0, 1, 0, 1);

		var        decomp = new EigenvalueDecomposition(covariance.ToMatrix());
		double[,]  stddev = decomp.DiagonalMatrix;

		for (int i = 0; i < stddev.GetLength(0); i++) {
			stddev[i, i] = (stddev[i, i] > 0) ? Math.Sqrt(stddev[i, i]) : 0;
		}

		double[,]  rotation = decomp.Eigenvectors;
		double[][] linear   = rotation.Multiply(stddev).ToArray();

		if (linear.Determinant() < 0) {
			linear = linear.ReverseColumns();
		}
		
		double[][] points = new double[pinterval.Length][];
		VertexPositionColor[] vertices = new VertexPositionColor[pinterval.Length];

		for (int i = 0; i < points.Length; i++) {
			points  [i]  = linear.Multiply(pinterval[i]);
			points  [i]  = new double[3] {points[i][0], points[i][1], 0}.Add(camera.Multiply(gaussian.Mean));
			vertices[i]  = new VertexPositionColor(points[i].ToVector3(), incolor);
		}

		short[] index = new short[vertices.Length];

		for (int i = 0, k = 0; k < vertices.Length; i++, k +=2) {
			index[k] = (short) i;
		}

		for (int i = vertices.Length - 1, k = 1; k < vertices.Length; i--, k += 2) {
			index[k] = (short) i;
		}

		Graphics.DrawUserIndexedPrimitives(PrimitiveType.TriangleStrip, vertices, 0, vertices.Length, index, 0, vertices.Length - 2);
		Graphics.DrawUser2DPolygon(points, (float) weight, outcolor, true);
	}
}
}
