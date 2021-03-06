﻿// Navigator.cs
// Abstract SLAM solving navigator
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
using Accord.Math.Decompositions;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

using TimedState      = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedTrajectory = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<System.Tuple<double, double[]>>>>;
using TimedMapModel   = System.Collections.Generic.List<System.Tuple<double, monorfs.Map>>;

namespace monorfs
{
/// <summary>
/// Abstract SLAM solver.
/// </summary>
public abstract class Navigator<MeasurerT, PoseT, MeasurementT> : IDisposable
	where PoseT        : IPose<PoseT>, new()
	where MeasurementT : IMeasurement<MeasurementT>, new()
	where MeasurerT    : IMeasurer<MeasurerT, PoseT, MeasurementT>, new()
{
	/// <summary>
	/// Saved odometry vector length.
	/// </summary>
	protected static int OdoSize;

	/// <summary>
	/// Saved state vector length.
	/// </summary>
	protected static int StateSize;

	/// <summary>
	/// Saved measurement vector length.
	/// </summary>
	protected static int MeasureSize;

	/// <summary>
	/// Maximum euclidean distance for a gaussian to be deemed relevant when evaluating the density.
	/// Note that ideally this would be Mahalanobis distance, but that is not scalable; a
	/// conservative euclidean distance should work fine, e.g. the prior measurement 5-sigma distance.
	/// </summary>
	public static double DensityDistanceThreshold { get { return Config.DensityDistanceThreshold; } }

	/// <summary>
	/// True if the algorithm performs online SLAM, i.e. uses information incrementally,
	/// generating a new estimate each time step; false otherwise (i.e. uses all the
	/// information at once and may run arbitrarily long).
	/// </summary>
	public virtual bool Online { get { return true; } }

	/// <summary>
	/// If true, the component rendering distinguishes between visible/nonvisible
	/// instead of their weight.
	/// </summary>
	public bool ShowVisible { get { return Config.ShowVisible; } }

	/// <summary>
	/// Time difference between iterations, important if the motion covariance is to be used.
	/// This should scale the covariance as in dt * MotionCovariance.
	/// </summary>
	protected static double MotionDt { get { return Config.MeasureElapsed.TotalSeconds; } }

	/// <summary>
	/// Reference vehicle. Only used if mapping mode is enabled (no localization).
	/// </summary>
	public Vehicle<MeasurerT, PoseT, MeasurementT> RefVehicle { get; private set; }

	/// <summary>
	/// Estimated trajectory history.
	/// Accounts for the fact that estimates can retroactively change.
	/// </summary>
	public TimedTrajectory WayTrajectories { get; protected set; }

	/// <summary>
	/// Most current estimated trajectory.
	/// </summary>
	public TimedState WayPoints {
		get { return WayTrajectories[WayTrajectories.Count - 1].Item2; }
	}

	/// <summary>
	/// Best map history.
	/// </summary>
	public TimedMapModel WayMaps { get; protected set; }

	/// <summary>
	/// Most accurate estimate of the current vehicle pose.
	/// </summary>
	public abstract TrackVehicle<MeasurerT, PoseT, MeasurementT> BestEstimate { get; }

	/// <summary>
	/// Most accurate estimate model of the map.
	/// </summary>
	public abstract Map BestMapModel { get; }

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
		get { return graphics;  }
		set { graphics = value; }
	}

	/// <summary>
	/// Gaussian prediction sigma-interval ellipse normalized render model.
	/// </summary>
	private double[][] pinterval;

	/// <summary>
	/// Calculate global constants.
	/// </summary>
	static Navigator()
	{
		OdoSize     = new PoseT().OdometrySize;
		StateSize   = new PoseT().StateSize;
		MeasureSize = new MeasurementT().Size;
	}

	/// <summary>
	/// Construct a Navigator using the indicated vehicle as a reference.
	/// </summary>
	/// <param name="vehicle">Vehicle to track.</param>
	/// <param name="onlymapping">If true, don't do SLAM, but mapping (i.e. the localization is assumed known).</param>
	protected Navigator(Vehicle<MeasurerT, PoseT, MeasurementT> vehicle, bool onlymapping = false)
	{
		OnlyMapping = onlymapping;

		// a reference copy, which gives the navigator precise info about
		// the real vehicle pose
		RefVehicle = vehicle;

		WayTrajectories = new TimedTrajectory();
		WayTrajectories.Add(Tuple.Create(0.0, new TimedState {Tuple.Create(0.0, Util.SClone(vehicle.Pose.State))}));

		WayMaps = new TimedMapModel();
		WayMaps.Add(Tuple.Create(0.0, new Map(3)));

		const int segments = 32;
		pinterval = new double[segments][];

		// pinterval will be the 5-sigma ellipse
		for (int i = 0; i < segments; i++) {
			pinterval[i] = new double[2] {5 * Math.Cos(2 * Math.PI * i / segments), 5 * Math.Sin(2 * Math.PI * i / segments)};
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
	protected virtual void StartSlamInternal() {}

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
	protected virtual void StartMappingInternal() {}

	/// <summary>
	/// Reset any information about the map to nothing.
	/// </summary>
	public abstract void ResetMapModel();

	/// <summary>
	/// Remove all the localization and mapping history and start it again from the current position.
	/// </summary>
	public virtual void ResetHistory()
	{
		WayTrajectories.Clear();
		WayMaps        .Clear();

		WayTrajectories.Add(Tuple.Create(0.0, new TimedState {Tuple.Create(0.0, Util.SClone(BestEstimate.Pose.State))}));
		WayMaps        .Add(Tuple.Create(0.0, new Map(3)));
	}

	/// <summary>
	/// Update the vehicle state.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="reading">Odometry reading (dx, dy, dz, dpitch, dyaw, droll).</param>
	public abstract void Update(GameTime time, double[] reading);

	/// <summary>
	/// Update both the estimated map and the localization.
	/// This method is the core of the whole program.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="measurements">Sensor measurements in pixel-range form.</param>
	public abstract void SlamUpdate(GameTime time, List<MeasurementT> measurements);
	
	/// <summary>
	/// Update the trajectory history with the
	/// current best estimate and a given time.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	protected void UpdateTrajectory(GameTime time)
	{
		TimedState waypoints = new TimedState(BestEstimate.WayPoints);
		WayTrajectories.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, waypoints));
	}
	
	/// <summary>
	/// Update the trajectory history with the
	/// current best map estimate and a given time.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	protected void UpdateMapHistory(GameTime time)
	{
		WayMaps.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, new Map(BestMapModel)));
	}

	/// <summary>
	/// Render the navigation HUD and the trajectory on the graphics device.
	/// </summary>
	/// <remarks>
	/// The graphics device must be ready, otherwise
	/// the method will throw an exception.
	/// </remarks>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public virtual void Render(double[][] camera)
	{
		RenderMap(camera);
		RenderTrajectory(camera);
		RefVehicle.Render(camera);
	}

	/// <summary>
	/// Render the map model, i.e. its estimated landmarks and covariances.
	/// </summary>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public void RenderMap(double[][] camera) {
		foreach (Gaussian component in BestMapModel) {
			RenderGaussian(component, camera);
		}
	}

	/// <summary>
	/// Render estimate of the path that the vehicle has traveled so far.
	/// </summary>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public void RenderTrajectory(double[][] camera)
	{
		DrawUtils.DrawTrajectory<PoseT>(Graphics, WayPoints, Color.Blue, camera);
	}

	/// <summary>
	/// Render the 5-sigma ellipse of gaussian (using the default color).
	/// </summary>
	/// <param name="gaussian">Gaussian to be rendered.</param>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public void RenderGaussian(Gaussian gaussian, double[][] camera)
	{
		RenderGaussian(gaussian, camera, Color.DeepSkyBlue);
	}

	/// <summary>
	/// Render the 5-sigma ellipse of gaussian.
	/// </summary>
	/// <param name="gaussian">Gaussian to be rendered.</param>
	/// <param name="camera">Camera 4d transform matrix.</param>
	/// <param name="incolor">Color to be used for filling the gaussian.</param>
	public void RenderGaussian(Gaussian gaussian, double[][] camera, Color incolor)
	{
		Color outcolor = Color.Blue;
		incolor.A      = 200;
		outcolor.A     = 200;
		
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

		if (ShowVisible) {
			outcolor = (BestEstimate.Visible(gaussian.Mean)) ? Color.Blue : Color.Red;
		}

		double[]   camloc     = camera.TransformH(gaussian.Mean);
		double[][] camrot     = camera.Submatrix(0, 2, 0, 2);
		double     camzoom    = 1.0 / camera[3][3];
		double[][] covariance = camrot.Multiply(gaussian.Covariance).MultiplyByTranspose(camrot).Submatrix(0, 1, 0, 1);

		var        decomp = new EigenvalueDecomposition(covariance.ToMatrix());
		double[,]  stddev = decomp.DiagonalMatrix;

		for (int i = 0; i < stddev.GetLength(0); i++) {
			stddev[i, i] = (stddev[i, i] > 0) ? camzoom * Math.Sqrt(stddev[i, i]) : 0;
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
			points  [i]  = new double[3] {points[i][0], points[i][1], 0}.Add(camloc);
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

	/// <summary>
	/// Dispose of any resources.
	/// </summary>
	public virtual void Dispose() {}
}
}
