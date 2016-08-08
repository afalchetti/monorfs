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

using AForge;
using Accord.Math;
using Accord.Statistics.Distributions.Univariate;

using Microsoft.Xna.Framework;

using FP = monorfs.FileParser;

namespace monorfs
{
/// <summary>
/// Simulation vehicle model.
/// It uses a 3d odometry motion model (yaw-pitch-roll) and
/// a pixel-range measurement model.
/// </summary>
public class SimulatedVehicle<MeasurerT, PoseT, MeasurementT> : Vehicle<MeasurerT, PoseT, MeasurementT>
	where PoseT        : IPose<PoseT>, new()
	where MeasurementT : IMeasurement<MeasurementT>, new()
	where MeasurerT    : IMeasurer<MeasurerT, PoseT, MeasurementT>, new()
{
	/// <summary>
	/// Internal probability of detection.
	/// </summary>
	private readonly double detectionProbability = Config.DetectionProbability;

	/// <summary>
	/// Amount of expected clutter (spuriousness) on the measurement process.
	/// </summary>
	public readonly double ClutterDensity = Config.ClutterDensity;

	/// <summary>
	/// Probability of detection.
	/// </summary>
	public double PD { get { return detectionProbability; } }

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
	public static bool PerfectStill { get { return Config.PerfectStill; } }

	/// <summary>
	/// Poisson distributed random generator with parameter lambda = ClutterCount.
	/// </summary>
	/// <remarks>The type is general distribution to allow for zero clutter (dirac distribution)</remarks>
	private UnivariateDiscreteDistribution clutterGen;

	/// <summary>
	/// Construct a new Vehicle from a "zero initial state", i.e. location = {0, 0, 0},
	/// theta = 0 and rotation axis = {1, 0, 0} with no landmarks.
	/// </summary>
	public SimulatedVehicle()
		: this(new PoseT().IdentityP(), new List<double[]>()) {}

	/// <summary>
	/// Construct a new Vehicle object from its initial state.
	/// </summary>
	/// <param name="initial">Initial pose.</param>
	/// <param name="landmarks">Landmark 3d locations against which the measurements are performed.</param>
	public SimulatedVehicle(PoseT initial, List<double[]> landmarks)
		: this(initial, landmarks, new MeasurerT()) {}

	/// <summary>
	/// Construct a new Vehicle object from its initial state.
	/// </summary>
	/// <param name="initial">Initial pose.</param>
	/// <param name="landmarks">Landmark 3d locations against which the measurements are performed.</param>
	/// <param name="measurer">Measuring config and methods.</param>
	public SimulatedVehicle(PoseT initial, List<double[]> landmarks, MeasurerT measurer)
		: base(initial, measurer)
	{
		ClutterCount = ClutterDensity * measurer.Volume();

		if (ClutterCount > 0) {
			clutterGen = new PoissonDistribution(ClutterCount);
		}
		else {
			clutterGen = new DegenerateDistribution(0);
		}

		Landmarks = landmarks;

		HasDataAssociation = true;
		DataAssociation    = new List<int>();
	}

	/// <summary>
	/// Copy constructor. Perform a deep copy of another simulated vehicle.
	/// </summary>
	/// <param name="that">Copied simulated vehicle.</param>
	/// <param name="copytrajectory">If true, the vehicle historic trajectory is copied. Relatively heavy operation.</param>
	public SimulatedVehicle(SimulatedVehicle<MeasurerT, PoseT, MeasurementT> that,
	                        bool copytrajectory = false)
		: this((Vehicle<MeasurerT, PoseT, MeasurementT>) that, copytrajectory)
	{
		this.detectionProbability = that.detectionProbability;
		this.ClutterDensity       = that.ClutterDensity;
		this.ClutterCount         = that.ClutterCount;
		this.clutterGen           = that.clutterGen;
	}

	/// <summary>
	/// Perform a deep copy of another general vehicle.
	/// </summary>
	/// <param name="that">Copied general vehicle.</param>
	/// <param name="copytrajectory">If true, the vehicle historic trajectory is copied. Relatively heavy operation.</param>
	public SimulatedVehicle(Vehicle<MeasurerT, PoseT, MeasurementT> that,
	                        bool copytrajectory = false)
		: this(that, 1, 1, Config.DetectionProbability, Config.ClutterDensity, copytrajectory) {}

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
	public SimulatedVehicle(Vehicle<MeasurerT, PoseT, MeasurementT> that,
	                        double motioncovmultiplier, double measurecovmultiplier,
	                        double pdetection, double clutter, bool copytrajectory = false)
		: base(that, copytrajectory)
	{
		this.detectionProbability  = pdetection;
		this.ClutterDensity        = clutter;
		this.ClutterCount          = this.ClutterDensity * this.Measurer.Volume();
		this.motionCovariance      = motioncovmultiplier.Multiply(that.MotionCovariance);
		this.MeasurementCovariance = measurecovmultiplier.Multiply(that.MeasurementCovariance);

		if (this.ClutterCount > 0) {
			this.clutterGen = new PoissonDistribution(this.ClutterCount);
		}
		else {
			this.clutterGen = new DegenerateDistribution(0);
		}
	}

	/// <summary>
	/// Apply the motion model to the vehicle. It corresponds to a
	/// 3D odometry model following the equation:
	/// 
	/// x = x + q dx q*
	/// o = dq o dq*
	/// 
	/// where q is the midrotation quaternion (halfway between the old and new orientations) and N(a, b) is a normal function
	/// with mean 'a' and covariance matrix 'b'.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="reading">Odometry reading (dx, dy, dz, dpitch, dyaw, droll).</param>
	public override void Update(GameTime time, double[] reading)
	{
		// no input, static friction makes the robot stay put (if there is any static friction)
		if (PerfectStill && reading.IsEqual(0)) {
			Pose         = Pose        .AddOdometry(reading);
			OdometryPose = OdometryPose.AddOdometry(reading);

			WayPoints.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, Util.SClone(Pose.State)));
		}
		else {
			base.Update(time, reading);
		}
	}

	/// <summary>
	/// Obtain a measurement from the hidden state.
	/// This method always detects the landmark (misdetection
	/// probability is ignored).
	/// </summary>
	/// <param name="landmark">Landmark 3d location against which the measurement is performed.</param>
	/// <returns>Pixel-range measurements.</returns>
	public MeasurementT MeasureDetected(double[] landmark)
	{
		MeasurementT measurement = Measurer.MeasurePerfect(Pose, landmark);
		double[]     mlinear     = measurement.ToLinear();
		double[]     noise       = Util.RandomGaussianVector(new double[mlinear.Length], MeasurementCovariance);

		return measurement.FromLinear(mlinear.Add(noise));
	}

	/// <summary>
	/// Obtain several measurements from the hidden state.
	/// This method always detects the landmark (misdetection
	/// probability is ignored).
	/// </summary>
	/// <returns>Pixel-range measurements.</returns>
	public MeasurementT[] MeasureDetected()
	{
		MeasurementT[] measurements = new MeasurementT[Landmarks.Count];

		for (int i = 0; i < Landmarks.Count; i++) {
			measurements[i] = MeasureDetected(Landmarks[i]);
		}

		return measurements;
	}

	/// <summary>
	/// Obtain several measurements from the hidden state.
	/// Ladmarks may be misdetected.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <returns>Pixel-range measurements.</returns>
	public override List<MeasurementT> Measure(GameTime time)
	{
		List<MeasurementT> measurements = new List<MeasurementT>();
		Map                visible      = new Map(3);
		DataAssociation                 = new List<int>();

		double[][] diraccov = new double[3][] { new double[3] {0.001, 0, 0},
		                                        new double[3] {0, 0.001, 0},
		                                        new double[3] {0, 0, 0.001} };

		// add every measurement with probability = DetectionProbability
		for (int i = 0; i < Landmarks.Count; i++) {
			if (Visible(Landmarks[i])) {
				if (Util.Uniform.Next() < detectionProbability) {
					measurements   .Add(MeasureDetected(Landmarks[i]));
					DataAssociation.Add(i);
					visible.Add(new Gaussian(Landmarks[i], diraccov, 1.0));
				}
				else {
					visible.Add(new Gaussian(Landmarks[i], diraccov, 0.0)); 
					// weight indicates visible but not detected
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
			measurements.Add(Measurer.RandomMeasure());
			DataAssociation.Add(int.MinValue);
		}

		MappedMeasurements.Clear();
		foreach (MeasurementT z in measurements) {
			MappedMeasurements.Add(Measurer.MeasureToMap(Pose, z));
		}

		WayVisibleMaps.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, visible));

		return measurements;
	}

	/// <summary>
	/// Obtain a linearization for the motion update equation.
	/// It follows the form
	/// f(x[k-1], u) - x[k] ~ F dx[k-1] + G dx[k] + a.
	/// </summary>
	/// <param name="odometry">Odometry.</param>
	public double[][] MotionJacobian(double[] odometry)
	{
		return Pose.AddOdometryJacobian(odometry);
	}

	/// <summary>
	/// Find if a given ladmark is visible from the current pose of the vehicle.
	/// </summary>
	/// <param name="landmark">Queried landmark.</param>
	/// <returns>True if the landmark is visible; false otherwise.</returns>
	public bool Visible(double[] landmark)
	{
		return Measurer.VisibleM(Measurer.MeasurePerfect(Pose, landmark));
	}

	/// <summary>
	/// Get the probability of detection of a particular landmark.
	/// It is modelled as a constant if it's on the FOV and zero if not. 
	/// </summary>
	/// <param name="landmark">Queried landmark.</param>
	/// <returns></returns>
	public double DetectionProbability(double[] landmark)
	{
		return Measurer.FuzzyVisibleM(Measurer.MeasurePerfect(Pose, landmark)) * detectionProbability;
	}

	/// <summary>
	/// Get the probability of detection of a particular landmark
	/// using measurement coordinates to express the landmark.
	/// It is modelled as a constant if it's on the FOV and zero if not.
	/// </summary>
	/// <param name="measurement">Queried landmark in measurmeent coordinates.</param>
	/// <returns></returns>
	public double DetectionProbabilityM(MeasurementT measurement)
	{
		return Measurer.FuzzyVisibleM(measurement) * detectionProbability;
	}

	/// <summary>
	/// Create a vehicle from a simulation file.
	/// </summary>
	/// <param name="descriptor">Scene descriptor text.</param>
	/// <returns>Simulated vehicle parsed from file.</returns>
	public static SimulatedVehicle<MeasurerT, PoseT, MeasurementT> FromFile(string descriptor)
	{
		Dictionary<string, List<string>> dict = Util.ParseDictionary(descriptor);

		double[] vehiclepose = FP.ParseDoubleList(dict["pose"][0]);

		double[] measurerdata = null;
		string   measurerkey  = (dict.ContainsKey("focal")) ? "focal" :
		                        (dict.ContainsKey("params")) ? "params" : "";
		// "focal" is deprecated in lieu of "params" which is more global to different kinds of sensors

		if (!string.IsNullOrEmpty(measurerkey)) {
			measurerdata = FP.ParseDoubleList(dict[measurerkey][0]);
		}

		PoseT dummy = new PoseT();
		PoseT pose = dummy.FromState(vehiclepose);

		List<double[]> maploc      = new List<double[]>();
		List<string>   mapdescript = dict["landmarks"];

		for (int i = 0; i < mapdescript.Count; i++) {
			double[] landmark = FP.ParseDoubleList(mapdescript[i]);

			if (landmark.Length != 3) {
				throw new FormatException("Map landmarks must be 3D");
			}

			maploc.Add(landmark);
		}

		if (measurerdata != null) {
			MeasurerT dummyM    = new MeasurerT();
			MeasurerT measurer = dummyM.FromLinear(measurerdata);
			return new SimulatedVehicle<MeasurerT, PoseT, MeasurementT>(pose, maploc, measurer);
		}
		else {
			return new SimulatedVehicle<MeasurerT, PoseT, MeasurementT>(pose, maploc);
		}
	}
}
}
