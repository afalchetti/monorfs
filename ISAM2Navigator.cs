// ISAM2Navigator.cs
// SLAM solving navigator using the iSAM2 iterative algorithm
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
using System.Runtime.InteropServices;

using TimedState    = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedMapModel = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<monorfs.Gaussian>>>;
using AForge.Math.Geometry;

namespace monorfs
{
/// <summary>
/// SLAM solver. It uses the iSAM2 iterative algorithm.
/// </summary>
public class ISAM2Navigator : Navigator
{
	/// <summary>
	/// The handle.
	/// </summary>
	private HandleRef handle;

	/// <summary>
	/// Vehicle pose estimate.
	/// </summary>
	private SimulatedVehicle estimate;

	/// <summary>
	/// Map model estimation.
	/// </summary>
	private List<Gaussian> mapmodel;

	/// <summary>
	/// Previous SLAM step vehicle pose estimate.
	/// </summary>
	private SimulatedVehicle prevestimate;

	/// <summary>
	/// Most accurate estimate of the current vehicle pose.
	/// </summary>
	public override SimulatedVehicle BestEstimate
	{
		get
		{
			return estimate;
		}
		set
		{
			estimate = value;
		}
	}

	/// <summary>
	/// Most accurate estimate model of the map.
	/// </summary>
	public override List<Gaussian> BestMapModel
	{
		get
		{
			return mapmodel;
		}
		set
		{
			mapmodel = value;
		}
	}

	/// <summary>
	/// Previous SLAM step vehicle pose estimate.
	/// </summary>
	public SimulatedVehicle PreviousEstimate
	{
		get
		{
			return prevestimate;
		}
		set
		{
			prevestimate = value;
		}
	}

	/// <summary>
	/// Construct a PHDNavigator using the indicated vehicle as a reference.
	/// </summary>
	/// <param name="vehicle">Vehicle to track.</param>
	/// <param name="onlymapping">If true, don't do SLAM, but mapping
	/// (i.e. the localization is assumed known). Currently not used.</param>
	public ISAM2Navigator(Vehicle vehicle, bool onlymapping = false)
		: base(vehicle, onlymapping)
	{
		int      nmeasurement     = vehicle.MeasurementCovariance.Length;
		int      nmotion          = vehicle.MotionCovariance.Length;
		double[] measurementnoise = new double[nmeasurement];
		double[] motionnoise      = new double[nmotion];
		double   focal            = vehicle.VisionFocal;

		for (int i = 0; i < nmeasurement; ++i) {
			measurementnoise[i] = vehicle.MeasurementCovariance[i][i];
		}

		for (int i = 0; i < nmotion; ++i) {
			motionnoise[i] = vehicle.MotionCovariance[i][i];
		}

		handle = NewNavigator(measurementnoise, motionnoise, focal);
	}

	/// <summary>
	/// Change the mode of the navigator to solving full slam.
	/// Currently do nothing.
	/// </summary>
	public override void StartSlamInternal() {}

	/// <summary>
	/// Change the mode of the navigator to do only mapping.
	/// Currently do nothing.
	/// </summary>
	public override void StartMappingInternal() {}

	/// <summary>
	/// Reset the map model to an empty map.
	/// Currently do nothing.
	/// </summary>
	public override void ResetMapModel() {}

	/// <summary>
	/// Update the vehicle pose.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="dx">Moved distance from odometry in the local vertical movement-perpendicular direction since last timestep.</param>
	/// <param name="dy">Moved distance from odometry in the local horizontal movement-perpendicular direction since last timestep.</param>
	/// <param name="dz">Moved distance from odometry in the local depth movement-parallel direction since last timestep.</param>
	/// <param name="dyaw">Angle variation from odometry in the yaw coordinate since last timestep.</param>
	/// <param name="dpitch">Angle variation from odometry in the pitch coordinate since last timestep.</param>
	/// <param name="droll">Angle variation from odometry in the roll coordinate since last timestep.</param>
	public override void Update(GameTime time, double dx, double dy, double dz,
	                            double dyaw, double dpitch, double droll)
	{
		BestEstimate.Update(time, dx, dy, dz, dyaw, dpitch, droll);
		UpdateTrajectory(time);
	}

	/// <summary>
	/// Update both the estimated map and the localization.
	/// This means adding the appropiate graph factor and optimizing.
	/// This method is the core of the whole program.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="measurements">Sensor measurements in pixel-range form.</param>
	public override void SlamUpdate(GameTime time, List<double[]> measurements)
	{
		double[] odometry            = Vehicle.StateDiff(estimate, prevestimate);
		double[] marshalmeasurements = new double[7 * measurements.Count];

		for (int i = 0, h = 0; i < measurements.Count; i++) {
			marshalmeasurements[h++] = measurements[i][0];
			marshalmeasurements[h++] = measurements[i][1];
			marshalmeasurements[h++] = measurements[i][2];
		}

		Update(odometry, marshalmeasurements, findLabels(measurements), measurements.Count);

		// FIXME note that the whole path could change retroactively but that requires
		//       more comprehensive structural changes so, for now, only update the
		//       "current position"; this is similar to the "best retroactive particle"
		//       of PHD SLAM and only getting the current one is more suitable for
		//       real-time simulation
		List<double[]> trajectory = GetTrajectory();
		BestEstimate.State = trajectory[trajectory.Count - 1];

		List<Gaussian> mapmodel = GetMapModel();
		WayMaps.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, mapmodel));

		UpdateMapHistory(time);
	}

	/// <summary>
	/// Find the data association labels from the new valid measurements and
	/// the internal previous map model using nearest-neighbour association.
	/// </summary>
	/// <param name="measurements">New measurements.</param>
	/// <returns>Association labels.</returns>
	public int[] findLabels(List<double[]> measurements)
	{
		// TODO implement this
		return new int[measurements.Count];
	}

	/// <summary>
	/// Dispose of any resources.
	/// </summary>
	public override void Dispose()
	{
		deletenavigator(handle);
	}
	
	private unsafe HandleRef NewNavigator(double[] measurementNoise, double[] motionNoise, double focal)
	{
		IntPtr ptr = IntPtr.Zero;

		fixed(double* ptrMeasurementNoise = measurementNoise) {
		fixed(double* ptrMotionNoise      = motionNoise) {
			ptr = newnavigator((IntPtr) ptrMeasurementNoise, (IntPtr) ptrMotionNoise, focal);
		}
		}

		return new HandleRef(this, ptr);
	}

	private unsafe void Update(double[] odometry, double[] measurements, int[] labels, int nmeasurements)
	{
		fixed(double* ptrOdometry     = odometry) {
		fixed(double* ptrMeasurements = measurements) {
		fixed(int*    ptrLabels       = labels) {
			update(handle, (IntPtr) ptrOdometry, (IntPtr) ptrMeasurements, (IntPtr) ptrLabels, nmeasurements);
		}
		}
		}
	}

	private unsafe List<double[]> GetTrajectory()
	{
		int            length;
		double[]       point      = new double[7];
		List<double[]> trajectory = new List<double[]>();

		double* ptrtrajectory = (double*) gettrajectory(handle, out length);

		for (int i = 0, k = 0; i < length; i++, k += 7) {
			point[0] = ptrtrajectory[k + 0];
			point[1] = ptrtrajectory[k + 1];
			point[2] = ptrtrajectory[k + 2];
			point[3] = ptrtrajectory[k + 3];
			point[4] = ptrtrajectory[k + 4];
			point[5] = ptrtrajectory[k + 5];
			point[6] = ptrtrajectory[k + 6];

			trajectory.Add(point);
		}

		return trajectory;
	}

	private unsafe List<Gaussian> GetMapModel()
	{
		int            length;
		Gaussian       component;
		double[]       mean       = new double[3];
		double[][]     covariance = {new double[3], new double[3], new double[3]};
		List<Gaussian> mapmodel   = new List<Gaussian>();

		double* ptrmapmodel = (double*) getmapmodel(handle, out length);
		double* ptrmapcov   = (double*) mapmarginals(handle, out length);

		for (int i = 0, k = 0, h = 0; i < length; i++, k += 3, h += 9) {
			mean[0]          = ptrmapmodel[k + 0];
			mean[1]          = ptrmapmodel[k + 1];
			mean[2]          = ptrmapmodel[k + 2];

			covariance[0][0] = ptrmapcov[k + 0];
			covariance[0][1] = ptrmapcov[k + 1];
			covariance[0][2] = ptrmapcov[k + 2];
			covariance[1][0] = ptrmapcov[k + 3];
			covariance[1][1] = ptrmapcov[k + 4];
			covariance[1][2] = ptrmapcov[k + 5];
			covariance[2][0] = ptrmapcov[k + 6];
			covariance[2][1] = ptrmapcov[k + 7];
			covariance[2][2] = ptrmapcov[k + 8];

			component = new Gaussian(mean, covariance, 1.0);
			mapmodel.Add(component);
		}

		return mapmodel;
	}

	[DllImport("libisam2.so")]
	private extern static IntPtr newnavigator(IntPtr measurementnoise, IntPtr motionnoise, double focal);

	[DllImport("libisam2.so")]
	private extern static void deletenavigator(HandleRef navigator);

	[DllImport("libisam2.so")]
	private extern static void update(HandleRef navigator, IntPtr odometry, IntPtr measurements, IntPtr labels, int nmeasurements);

	[DllImport("libisam2.so")]
	private extern static IntPtr gettrajectory(HandleRef navigator, out int length);

	[DllImport("libisam2.so")]
	private extern static IntPtr getmapmodel(HandleRef navigator, out int length);

	[DllImport("libisam2.so")]
	private extern static IntPtr trajectorymarginals(HandleRef navigator, out int length);

	[DllImport("libisam2.so")]
	private extern static IntPtr mapmarginals(HandleRef navigator, out int length);
}
}
