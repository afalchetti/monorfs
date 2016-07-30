// LoopyPHDNavigator.cs
// SLAM solving navigator using the PHD smoother
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

using Microsoft.Xna.Framework;

using TimedState        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedArray        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedMapModel     = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<monorfs.Gaussian>>>;
using TimedGaussian     = System.Collections.Generic.List<System.Tuple<double, monorfs.Gaussian>>;
using TimedMeasurements = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<double[]>>>;
using System.Threading.Tasks;

namespace monorfs
{
/// <summary>
/// SLAM solver. It uses the PHD loopy belief propagator.
/// </summary>
public class LoopyPHDNavigator : Navigator
{
	/// <summary>
	/// Internal motion model covariance matrix. Yaw-pitch-roll representation.
	/// </summary>
	public double[][] MotionCovarianceL { get { return Config.MotionCovarianceL; } }

	/// <summary>
	/// Pose default covariance.
	/// </summary>
	public static double[][] PoseCovariance { get { return (1/8.0).Multiply(MotionDt.Multiply(Config.MotionCovarianceQ)); } }

	/// <summary>
	/// Learning rate for the gradient descent method.
	/// </summary>
	public static double GradientAscentRate { get { return Config.GradientAscentRate; } }

	/// <summary>
	/// Inner PHD filter algorithm.
	/// </summary>
	public PHDNavigator InnerFilter { get; private set; }

	/// <summary>
	/// Messages from pose[k-1] to pose[k].
	/// The first pose should receive a Dirac delta at the origin
	/// as message from the past, equivalent to the euclidean prior.
	/// </summary>
	public TimedGaussian MessagesFromPast { get; private set; }

	/// <summary>
	/// Messages from pose[k+1] to pose[k].
	/// The last pose should receive a unit factor as message
	/// from the future, since there's no future.
	/// </summary>
	public TimedGaussian MessagesFromFuture { get; private set; }

	/// <summary>
	/// Messages from the map to the poses.
	/// </summary>
	public TimedGaussian MessagesFromMap { get; private set; }

	/// <summary>
	/// Measurement sets for RFS map update delta factors.
	/// </summary>
	public TimedMeasurements Measurements { get; private set; }

	/// <summary>
	/// Odometry readings.
	/// </summary>
	public TimedArray Odometry { get; private set; }

	/// <summary>
	/// Internal representation of the pose trajectory for RFS map update delta factors.
	/// </summary>
	private TimedState mapMessages;

	/// <summary>
	/// Pose trajectory for RFS map update delta factors.
	/// </summary>
	public TimedState MapMessages
	{
		get
		{
			return mapMessages;
		}

		private set
		{
			mapMessages = value;
			MapCached   = false;
		}
	}

	/// <summary>
	/// Detached fused pose estimates.
	/// </summary>
	public TimedGaussian DetachedEstimate { get; private set; }

	/// <summary>
	/// Current pose estimates.
	/// </summary>
	public TimedGaussian FusedEstimate
	{
		get
		{
			if (TrajectoryDetached) {
				return DetachedEstimate;
			}
			else {
				return FuseGaussians(FuseGaussians(MessagesFromPast, MessagesFromFuture), MessagesFromMap);
			}
		}

		set
		{
			DetachedEstimate   = value;
			TrajectoryDetached = true;
		}
	}

	/// <summary>
	/// Most accurate estimate of the current vehicle pose.
	/// </summary>
	public override TrackVehicle BestEstimate
	{
		get
		{
			TrackVehicle maxlike = new TrackVehicle();

			maxlike.WayPoints = new TimedState();

			foreach (var pose in FusedEstimate) {
				maxlike.WayPoints.Add(Tuple.Create(pose.Item1, Util.SClone(pose.Item2.Mean)));
			}

			maxlike.Pose = new Pose3D(maxlike.WayPoints[maxlike.WayPoints.Count - 1].Item2);

			return maxlike;
		}

		set
		{
			FusedEstimate = new TimedGaussian();

			foreach (var pose in value.WayPoints) {
				FusedEstimate.Add(Tuple.Create(pose.Item1,
				                          new Gaussian(Util.SClone(pose.Item2), PoseCovariance, 1.0)));
			}


		}
	}

	/// <summary>
	/// Detached map model estimate.
	/// </summary>
	public Map DetachedMapModel { get; private set; }

	/// <summary>
	/// Cached version of the map from the last time it changed.
	/// </summary>
	public Map CachedMapModel { get; private set; }

	/// <summary>
	/// If true, the map has already been computed and can be recovered from the cache.
	/// </summary>
	public bool MapCached { get; private set; }

	/// <summary>
	/// Most accurate estimate model of the map.
	/// Could be detached from the message factors, e.g. at startup.
	/// </summary>
	public override Map BestMapModel
	{
		get
		{
			if (!MapCached) {
				if (MapDetached) {
					CachedMapModel = DetachedMapModel;
				}
				else {
					CachedMapModel = Filter(MapMessages, Measurements);
				}
			}

			MapCached = true;
			return CachedMapModel;
		}

		set
		{
			DetachedMapModel = value;
			CachedMapModel   = DetachedMapModel;

			MapCached        = true;
			MapDetached      = true;
		}
	}

	/// <summary>
	/// True if the best trajectory estimate doesn't necessarily correspond to the
	/// messages from the map and the neighbouring poses, e.g. at startup.
	/// In detached state, the fused estimate will not be updated when messages change.
	/// </summary>
	public bool TrajectoryDetached { get; private set; }

	/// <summary>
	/// True if the best map model doesn't necessarily correspond to the
	/// messages from the poses, e.g. at startup.
	/// In detached state, the map model will not be updated when messages change.
	/// </summary>
	public bool MapDetached { get; private set; }

	/// <summary>
	/// Construct a PHDNavigator using the indicated vehicle as a reference.
	/// </summary>
	/// <param name="vehicle">Vehicle to track.</param>
	/// <param name="commands">List of commands given to the vehicle.</param>
	/// <param name="initialestimate">Navigation algorithm used to get an initial estimate.</param>
	public LoopyPHDNavigator(Vehicle vehicle, List<double[]> commands, Navigator initialestimate)
		: base(vehicle, false)
	{
		InnerFilter  = new PHDNavigator(vehicle, 1, false);

		TimedArray        odometry;
		TimedMeasurements measurements;
		BestEstimate = InitialEstimate(vehicle, commands, initialestimate, out odometry, out measurements);

		Odometry     = odometry;
		Measurements = measurements;

		initMessages();
	}

	/// <summary>
	/// Construct a LoopyPHDNavigator using precalculated estimates.
	/// </summary>
	/// <param name="vehicle">Vehicle to track.</param>
	/// <param name="trajectory">Initial trajectory estimate.</param>
	/// <param name="odometry">Odometry readings.</param>
	/// <param name="measurements">Measurement readings.</param>
	public LoopyPHDNavigator(Vehicle vehicle, TimedState trajectory,
	                         TimedArray odometry, TimedMeasurements measurements)
		: base(vehicle, false)
	{
		InnerFilter  = new PHDNavigator(vehicle, 1, false);
		Odometry     = odometry;
		Measurements = measurements;

		var tvehicle       = new TrackVehicle();
		tvehicle.WayPoints = trajectory;
		BestEstimate       = tvehicle;

		initMessages();
	}

	/// <summary>
	/// Generates the first messages between nodes.
	/// </summary>
	private void initMessages()
	{
		MessagesFromPast   = new TimedGaussian();
		MessagesFromFuture = new TimedGaussian();

		double[][] infcov = Util.InfiniteCovariance(FusedEstimate[0].Item2.Mean.Length);

		for (int i = 0; i < FusedEstimate.Count; i++) {
			MessagesFromPast  .Add(Tuple.Create(0.0, (Gaussian) null));
			MessagesFromFuture.Add(Tuple.Create(0.0, (Gaussian) null));
		}

		Parallel.For(0, FusedEstimate.Count, i => {
			MessagesFromPast  [i] = Tuple.Create(FusedEstimate[i].Item1, new Gaussian(FusedEstimate[i].Item2.Mean, infcov, 1.0));
			MessagesFromFuture[i] = Tuple.Create(FusedEstimate[i].Item1, new Gaussian(FusedEstimate[i].Item2.Mean, infcov, 1.0));
		});

		MapMessages        = GetMapMessages();
		MessagesFromMap    = GetMessagesFromMap();
		MessagesFromPast   = GetMessagesFromPast();
		MessagesFromFuture = GetMessagesFromFuture();

		Console.WriteLine(FusedEstimate     [4].Item2.Mean.ToString("F3"));
		Console.WriteLine(MessagesFromPast  [4].Item2.Mean.ToString("F3"));
		Console.WriteLine(MessagesFromFuture[4].Item2.Mean.ToString("F3"));
		Console.WriteLine(MessagesFromMap   [4].Item2.Mean.ToString("F3"));
		Console.WriteLine(" - - - - - - ");
		Console.WriteLine(Odometry[4].Item2.ToString("F3"));

		SimulatedVehicle vx = new SimulatedVehicle();
		vx.Pose = new Pose3D(FusedEstimate[4].Item2.Mean);
		vx.Update(new GameTime(), Odometry[4].Item2);

		Console.WriteLine(vx.Pose.State.ToString("F3"));
		Console.WriteLine(" - - - - - - ");
		Console.WriteLine(FusedEstimate     [5].Item2.Mean.ToString("F3"));
		Console.WriteLine(MessagesFromPast  [5].Item2.Mean.ToString("F3"));
		Console.WriteLine(MessagesFromFuture[5].Item2.Mean.ToString("F3"));
		Console.WriteLine(MessagesFromMap   [5].Item2.Mean.ToString("F3"));
		Console.WriteLine(" |=========| ");

		TrajectoryDetached = false;
		MapDetached        = false;
	}

	/// <summary>
	/// Reset the model of every particle to an empty map.
	/// </summary>
	public override void ResetMapModel()
	{
		// TODO implement something here
	}

	/// <summary>
	/// Remove all the localization and mapping history and start it again from the current position.
	/// </summary>
	public override void ResetHistory()
	{
		// TODO think about the optimization time/trajectory time tricky situation
	}

	/// <summary>
	/// Produce an initial estimate based on a vehicle.
	/// </summary>
	/// <param name="vehicle">Reference vehicle.</param>
	/// <param name="commands">Commands sent to the vehicle.</param>
	/// <param name="initnav">Navigator to generate the initial estimate.</param>
	/// <param name="odometry">Odometry readings.</param>
	/// <param name="measurements">Measurement readings.</param>
	/// <returns>Initial estimate.</returns>
	public TrackVehicle InitialEstimate(Vehicle vehicle, List<double[]> commands, Navigator initnav,
	                                    out TimedArray odometry, out TimedMeasurements measurements)
	{
		Simulation sim = new Simulation("", vehicle, initnav, commands, true, false);

		sim.RunHeadless();

		odometry              = sim.Explorer.WayOdometry;
		measurements          = sim.WayMeasurements;

		TrackVehicle tvehicle = new TrackVehicle();
		tvehicle.WayPoints    = sim.Navigator.WayTrajectories[sim.Navigator.WayTrajectories.Count - 1].Item2;

		return tvehicle;
	}

	/// <summary>
	/// Updates the state of the vehicle.
	/// In this class, does nothing.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="reading">Odometry reading (dx, dy, dz, dpitch, dyaw, droll).</param>
	public override void Update(GameTime time, double[] reading) {}

	//private int xd = 0;

	/// <summary>
	/// Globally update the vehicle and map estimates.
	/// This method is the core of the whole program.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="measurements">Sensor measurements in pixel-range form. Not used.</param>
	public override void SlamUpdate(GameTime time, List<double[]> measurements)
	{
		//if (++xd > 00) {
			//return;
		//}

		TimedGaussian nextMessagesFromPast   = GetMessagesFromPast();
		TimedGaussian nextMessagesFromFuture = GetMessagesFromFuture();
		TimedGaussian nextMessagesFromMap    = GetMessagesFromMap();
		TimedState    nextMapMessages        = GetMapMessages();

		MessagesFromPast   = nextMessagesFromPast;
		MessagesFromFuture = nextMessagesFromFuture;
		MessagesFromMap    = nextMessagesFromMap;
		MapMessages        = nextMapMessages;
	}

	/// <summary>
	/// Get messages from the past given the current estimates.
	/// </summary>
	/// <returns>Messages from the past.</returns>
	public TimedGaussian GetMessagesFromPast()
	{
		TimedGaussian messages = new TimedGaussian();
		//double[][]    Rinv     = Config.MotionCovarianceL.Inverse();

		messages.Add(Tuple.Create(FusedEstimate[0].Item1, Util.DiracDelta(new double[7] {0, 0, 0, 1, 0, 0, 0})));

		for (int i = 0; i < FusedEstimate.Count - 1; i++) {
			messages.Add(Tuple.Create(0.0, (Gaussian) null));
		}

//		for (int i = 1; i < FusedEstimate.Count; i++) {
		Parallel.For(1, FusedEstimate.Count, i => {
			//double[]   iF, iG;
			//double[][] iFF, iFG, iGF, iGG;

			Pose3D linearpoint = new Pose3D(FusedEstimate[i].Item2.Mean);

			//Gaussian fusedlieG  = Util.Quat2Lie(FusedEstimate[i].Item2, linearpoint);
			Gaussian fusedlieF  = Util.Quat2Lie(FusedEstimate[i-1].Item2, linearpoint);
			Gaussian futurelieF = Util.Quat2Lie(MessagesFromFuture[i-1].Item2, linearpoint);
			Gaussian halffusedF = Gaussian.Unfuse(fusedlieF, futurelieF);

//			LinearInfoChop(halffusedF.Mean, fusedlieG.Mean, linearpoint, Odometry[i-1].Item2, Rinv,
//			               out iF, out iG, out iFF, out iFG, out iGF, out iGG);
//
//			double[][] bigfactor  = iGF.Multiply(iFF.Add(halffusedF.CanonicalMatrix).PseudoInverse());
//			double[]   posevector = iG .Subtract(bigfactor.Multiply(iF.Add(halffusedF.CanonicalVector)));
//			double[][] posematrix = iGG.Subtract(bigfactor.Multiply(iFG));
//
//			// incorrect: Add should be in the Pose3D space!!
//			Gaussian estimate2 = Util.Lie2Quat(new Gaussian(halffusedF.Mean.Add(Odometry[i-1].Item2),
//			                                                halffusedF.Covariance.Add(Config.MotionCovarianceL), 1.0),
//			linearpoint);
//
//			Gaussian estimate = Util.Lie2Quat(Gaussian.Canonical(posevector, posematrix, 1.0), linearpoint);

			Pose3D hfpose  = linearpoint.Add(halffusedF.Mean);
			Pose3D estpose = hfpose.Add(Odometry[i-1].Item2);

			Gaussian estimate = Util.Lie2Quat(new Gaussian(estpose.Subtract(linearpoint),
			                                               halffusedF.Covariance.Add(MotionCovarianceL), 1.0),
			                                  linearpoint);

//			if (i == 6) {
//				Console.WriteLine(new Pose3D(FusedEstimate[i-1].Item2.Mean));
//
//				Console.WriteLine(new Pose3D(estimate.Mean));
//				Console.WriteLine(new Pose3D(FusedEstimate[i].Item2.Mean));
//				Console.WriteLine(" . . - . . - . . .");
//
//				Console.WriteLine(estimate.Covariance.ToString("f6"));
//				Console.WriteLine("| --- --- --- --- |");
//
//				Console.WriteLine(halffusedF.Covariance.Add(MotionCovarianceL).ToString("f6"));
//				Console.WriteLine("===================");
//			}

			messages[i] = Tuple.Create(FusedEstimate[i].Item1, estimate);
		});
//		}
			
		return messages;
	}

	/// <summary>
	/// Get messages from the future given the current estimates.
	/// </summary>
	/// <returns>Messages from the future.</returns>
	public TimedGaussian GetMessagesFromFuture()
	{
		TimedGaussian messages = new TimedGaussian();
		double[][]    Rinv     = Config.MotionCovarianceL.Inverse();

		for (int i = 0; i < FusedEstimate.Count - 1; i++) {
			messages.Add(Tuple.Create(0.0, (Gaussian) null));
		}
		
//		for (int i = 0; i < FusedEstimate.Count - 1; i++) {
		Parallel.For(0, FusedEstimate.Count - 1, i => {
			//double[]   iF, iG;
			//double[][] iFF, iFG, iGF, iGG;

			Pose3D linearpoint = new Pose3D(FusedEstimate[i].Item2.Mean);

			//Gaussian   fusedlieF  = Util.Quat2Lie(FusedEstimate[i].Item2, linearpoint);
			Gaussian   fusedlieG  = Util.Quat2Lie(FusedEstimate[i+1].Item2, linearpoint);
			Gaussian   futurelieG = Util.Quat2Lie(MessagesFromFuture[i+1].Item2, linearpoint);
			Gaussian   halffusedG = Gaussian.Unfuse(fusedlieG, futurelieG);
//
//			LinearInfoChop(fusedlieF.Mean, halffusedG.Mean, linearpoint, Odometry[i].Item2, Rinv,
//			               out iF, out iG, out iFF, out iFG, out iGF, out iGG);
//
//			double[][] bigfactor  = iFG.Multiply(iGG.Add(halffusedG.CanonicalMatrix).PseudoInverse());
//			double[]   posevector = iF .Subtract(bigfactor.Multiply(iG.Add(halffusedG.CanonicalVector)));
//			double[][] posematrix = iFF.Subtract(bigfactor.Multiply(iGF));
//
//			Gaussian estimate = Util.Lie2Quat(Gaussian.Canonical(posevector, posematrix, 1.0), linearpoint);

			Pose3D hfpose  = linearpoint.Add(halffusedG.Mean);
			Pose3D estpose = hfpose.Add((-1.0).Multiply(Odometry[i].Item2));

			Gaussian estimate = Util.Lie2Quat(new Gaussian(estpose.Subtract(linearpoint),
			                                               halffusedG.Covariance.Add(MotionCovarianceL), 1.0),
			                                  linearpoint);

			if (double.IsNaN(estimate.Mean[0])) {
				int x = 1;
				Util.Quat2Lie(FusedEstimate[i+1].Item2, linearpoint);
				Gaussian.Unfuse(fusedlieG, futurelieG);
			}

			messages[i] = Tuple.Create(FusedEstimate[i].Item1, estimate);
		});
//		}

		var lastpose = FusedEstimate[FusedEstimate.Count - 1];
		messages.Add(Tuple.Create(lastpose.Item1,
		                          new Gaussian(lastpose.Item2.Mean,
		                                       Util.InfiniteCovariance(lastpose.Item2.Mean.Length), 1.0)));

		return messages;
	}

	/// <summary>
	/// Get messages from the map given the current estimates.
	/// </summary>
	/// <returns>Messages from the map to the poses.</returns>
	public TimedGaussian GetMessagesFromMap()
	{
		// ugly hack for debug speed!!!
		Map dMap = Filter(MapMessages, Measurements);

		TimedGaussian messages = new TimedGaussian();

		for (int i = 0; i < FusedEstimate.Count; i++) {
			messages.Add(Tuple.Create(0.0, (Gaussian) null));
		}

		Parallel.For(0, FusedEstimate.Count, i => {
			// ugly hack for debug speed!!!
			//Map      dMap     = FilterMissing(MapMessages, Measurements, i);
			Gaussian estimate = FitGaussian(new Pose3D(FusedEstimate[i].Item2.Mean), Measurements[i].Item2, dMap);

			messages[i] = Tuple.Create(MapMessages[i].Item1, estimate);
		});

		return messages;
	}

	/// <summary>
	/// Get messages to the map given the current estimates.
	/// </summary>
	/// <returns>Messages from the poses to the map.</returns>
	public TimedState GetMapMessages()
	{
		TimedState messages = new TimedState();

		TimedGaussian hfused = FuseGaussians(MessagesFromPast, MessagesFromFuture);

		for (int i = 0; i < hfused.Count; i++) {
			messages.Add(Tuple.Create(0.0, (double[]) null));
		}

		Parallel.For(0, hfused.Count, i => {
			messages[i] = Tuple.Create(hfused[i].Item1, hfused[i].Item2.Mean);
		});

		return messages;
	}

	/// <summary>
	/// Fuses two trajectory estimates with gaussians in each pose.
	/// </summary>
	/// <returns>Fused estimate.</returns>
	/// <param name="first">First estimate.</param>
	/// <param name="second">Second estimate.</param>
	public static TimedGaussian FuseGaussians(TimedGaussian first, TimedGaussian second)
	{
		TimedGaussian fused = new TimedGaussian();

		if (first.Count != second.Count) {
			throw new ArgumentException("the trajectory estimates must have the same size.");
		}

		for (int i = 0; i < first.Count; i++) {
			fused.Add(Tuple.Create(first[i].Item1, Gaussian.Fuse(first[i].Item2, second[i].Item2)));
		}

		return fused;
	}

	/// <summary>
	/// Obtain a linearization for the motion update equation.
	/// It follows the form
	/// f(x[k-1], u) - x[k] ~ F dx[k-1] + G dx[k] + a.
	/// </summary>
	/// <param name="past">Past pose state.</param>
	/// <param name="present">Present pose state.</param>
	/// <param name="odometry">Odometry.</param>
	/// <param name="F">Linear factor for the pose state.</param>
	/// <param name="G">Linear factor for the next pose state.</param>
	/// <param name="a">Linearization free constant.</param>
	public void MotionLinearization(Pose3D past, Pose3D present, double[] odometry, out double[][] F, out double[][] G, out double[] a)
	{
		F = past.AddJacobian(odometry);
		G = (-1.0).Multiply(Accord.Math.Matrix.JaggedIdentity(odometry.Length));
		// G is not exactly right, but let's see what happens :)
		// should be C(x[k]^)^-1 C(x[k])

		past.Add(odometry);

		a = past.Subtract(present);
	}

	/// <summary>
	/// Linearize the motion model around a pose and odometry and
	/// from the linearized model, get the canonical representation
	/// chopped by variables.
	/// </summary>
	/// <param name="past">Past pose linearization point.</param>
	/// <param name="present">Present pose linearization point.</param>
	/// <param name="linearpoint">Lie transformation linearization point.</param>
	/// <param name="odometry">Odometry reading.</param>
	/// <param name="Rinv">Inverse of the odometry noise covariance.</param>
	/// <param name="infovectorF">Information vector corresponding to the past pose.</param>
	/// <param name="infovectorG">Information vector corresponding to this pose.</param>
	/// <param name="infomatrixFF">Information matrix corresponding to Past x Past.</param>
	/// <param name="infomatrixFG">Information matrix corresponding to Past x This.</param>
	/// <param name="infomatrixGF">Information matrix corresponding to This x Past.</param>
	/// <param name="infomatrixGG">Information matrix corresponding to This x This.</param>
	public void LinearInfoChop(double[] past, double[] present, Pose3D linearpoint,
	                           double[] odometry, double[][] Rinv,
	                           out double[]   infovectorF,  out double[]   infovectorG,
	                           out double[][] infomatrixFF, out double[][] infomatrixFG,
	                           out double[][] infomatrixGF, out double[][] infomatrixGG)
	{
		double[][] F, G;
		double[]   a;

		int vsize = past.Length;

		MotionLinearization(linearpoint.Add(past), linearpoint.Add(present),
		                    odometry, out F, out G, out a);

		double[][] auglinear  = F.VConcatenate(G);
		double[]   infovector = auglinear.Multiply(Rinv).Multiply(a);
		double[][] infomatrix = auglinear.Multiply(Rinv).MultiplyByTranspose(auglinear);

		infovectorF = infovector.Submatrix(0, vsize - 1);
		infovectorG = infovector.Submatrix(vsize);

		infomatrixFF = infomatrix.Submatrix(0,     vsize - 1,     0,     vsize - 1);
		infomatrixFG = infomatrix.Submatrix(vsize, 2 * vsize - 1, 0,     vsize - 1);
		infomatrixGF = infomatrix.Submatrix(0,     vsize - 1,     vsize, 2 * vsize - 1);
		infomatrixGG = infomatrix.Submatrix(vsize, 2 * vsize - 1, vsize, 2 * vsize - 1);
	}

	/// <summary>
	/// Get a map estimate by using the PHD filter.
	/// </summary>
	/// <param name="trajectory">Trajectory estimate.</param>
	/// <param name="factors">Map factors.</param>
	public Map Filter(TimedState trajectory, TimedMeasurements factors)
	{
		return FilterMissing(trajectory, factors, -1);
	}

	/// <summary>
	/// Get a map estimate by using the PHD filter.
	/// </summary>
	/// <param name="trajectory">Trajectory estimate.</param>
	/// <param name="factors">Map factors.</param>
	/// <param name="index">Removed factor index from the filtering;
	/// if it's outside of the trajectory time boundaries, it will filter normally.</param>
	public Map FilterMissing(TimedState trajectory, TimedMeasurements factors, int index)
	{
		InnerFilter.BestMapModel           = new Map();
		InnerFilter.BestEstimate           = new TrackVehicle();
		InnerFilter.BestEstimate.WayPoints = new TimedState();

		if (index < 0) {
			index = trajectory.Count;
			// if the index is bigger or equal to the trajectory length
			// the second loop won't do anything, i.e. full filter
		}

		for (int i = 0; i < index; i++) {
			GameTime time = new GameTime(new TimeSpan((int) (trajectory[i].Item1 * 100000000)), Config.MeasureElapsed);

			InnerFilter.BestEstimate.Pose = new Pose3D(FusedEstimate[i].Item2.Mean);
			InnerFilter.BestEstimate.WayPoints.Add(Tuple.Create(FusedEstimate[i].Item1, FusedEstimate[i].Item2.Mean));

			InnerFilter.SlamUpdate(time, factors[i].Item2);
		}

		// NOTE the frame consecutive to the missing index should have a
		//      time increment of 2dt, since a measurement is missing
		//      however, the PHD does not use dt for any calculations
		//      so it wouldn't do anything. The birth rate could be
		//      proportional to dt, but its effect should be marginal

		for (int i = index + 1; i < trajectory.Count; i++) {
			GameTime time = new GameTime(new TimeSpan((int) (trajectory[i].Item1 * 100000000)), Config.MeasureElapsed);

			InnerFilter.BestEstimate.Pose = new Pose3D(FusedEstimate[i].Item2.Mean);
			InnerFilter.BestEstimate.WayPoints.Add(Tuple.Create(FusedEstimate[i].Item1, FusedEstimate[i].Item2.Mean));

			InnerFilter.SlamUpdate(time, factors[i].Item2);
		}

		return InnerFilter.BestMapModel;
	}

	/// <summary>
	/// Fit a gaussian probability to the pose near its current estimate
	/// using gradient descent and quadratic regression.
	/// </summary>
	/// <param name="pose0">Pose initial estimate.</param>
	/// <param name="measurements">Fixed measurements.</param>
	/// <param name="map">Fixed map.</param>
	/// <returns>Fitted gaussian.</returns>
	public Gaussian FitGaussian(Pose3D pose0, List<double[]> measurements, Map map)
	{
		double     maxvalue;
		Pose3D     maxpose    = LogLikeGradientAscent(pose0, measurements, map, out maxvalue);
		double[][] covariance = LogLikeFitCovariance(maxpose, measurements, map);

		return new Gaussian(maxpose.State, covariance, 1.0);
	}

	/// <summary>
	/// Calculate gradient of the log-likelihood of
	/// the measurements given the pose, Grad[log P(Z|X)].
	/// </summary>
	/// <param name="vehicle">Pose where to calculate the gradient.</param>
	/// <param name="measurements">Fixed measurements.</param>
	/// <param name="map">Fixed map.</param>
	/// <returns>Gradient.</returns>
	public double[] LogLikeGradient(SimulatedVehicle vehicle, List<double[]> measurements, Map map)
	{
		const double eps = 1e-4;

		double[] gradient = new double[vehicle.Pose.State.Length];
		double   val0     = Math.Log(InnerFilter.SetLikelihood(measurements, map, vehicle));

		for (int i = 0; i < gradient.Length; i++) {
			var      dvehicle = new SimulatedVehicle();
			double[] ds       = new double[vehicle.Pose.State.Length];
			ds[i]             = eps;

			dvehicle.Pose = vehicle.Pose.Add(ds);

			gradient[i] = Math.Log(InnerFilter.SetLikelihood(measurements, map, dvehicle)) - val0;
		}


		return gradient;
	}

	/// <summary>
	/// Uses the gradient ascent method to find a local maximum around
	/// the given initial estimate.
	/// </summary>
	/// <param name="initial">Initial pose estimate.</param>
	/// <param name="measurements">Fixed measurements.</param>
	/// <param name="map">Fixed map.</param>
	/// <param name="loglike">Output. Log-likelihood attained at the returned pose</param> 
	/// <returns>Pose that reaches the maximum local log-likelihood.</returns>
	public Pose3D LogLikeGradientAscent(Pose3D initial, List<double[]> measurements, Map map, out double loglike)
	{
		const double loglikethreshold = 0.1;

		SimulatedVehicle vehicle = new SimulatedVehicle();
		vehicle.Pose = new Pose3D(initial);

		double   prevvalue = double.NegativeInfinity;
		double[] gradient;

		loglike  = Math.Log(InnerFilter.SetLikelihood(measurements, map, vehicle));

		while (loglike - prevvalue > loglikethreshold) {
			gradient = LogLikeGradient(vehicle, measurements, map);

			vehicle.Pose = vehicle.Pose.Add(GradientAscentRate.Multiply(gradient));

			prevvalue = loglike;
			loglike   = Math.Log(InnerFilter.SetLikelihood(measurements, map, vehicle));
		}

		return vehicle.Pose;
	}

	/// <summary>
	/// Fit a covariance matrix to the surroundings of a pose, assuming
	/// this pose is locally maximal.
	/// </summary>
	/// <param name="pose">Pose.</param>
	/// <param name="measurements">Fixed measurements.</param>
	/// <param name="map">Fixed map.</param>
	/// <returns>Estimated covariance matrix.</returns>
	public double[][] LogLikeFitCovariance(Pose3D pose, List<double[]> measurements, Map map)
	{
		// TODO implement this
		return PoseCovariance;
	}

	/// <summary>
	/// Render the navigation HUD and the trajectory on the graphics device.
	/// The graphics device must be ready, otherwise
	/// the method will throw an exception.
	/// </summary>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public override void Render(double[][] camera)
	{
		DrawUtils.DrawTrajectory(Graphics, RefVehicle.Groundtruth, Color.Yellow, camera);
		RefVehicle.RenderLandmarks(camera);
		//RefVehicle.RenderBody(camera);
		//RenderTimedGaussian(MessagesFromPast, Color.Red, camera);
		//RenderTimedGaussian(MessagesFromFuture, Color.Green, camera);
		//RenderTimedGaussian(MessagesFromMap, Color.Orange, camera);
		RenderTimedGaussian(FusedEstimate, Color.Gray, camera);
		RenderMap(camera);
	}

	/// <summary>
	/// Render the mean and sigma-ellipses for a sequential series of gaussians.
	/// </summary>
	/// <param name="gaussians">Gaussian sequential series.</param>
	/// <param name="color">Path color.</param>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public void RenderTimedGaussian(TimedGaussian gaussians, Color color, double[][] camera)
	{
		TimedState trajectory = new TimedState();

		foreach (var pose in FusedEstimate) {
			double[]   location = pose.Item2.Mean.Submatrix(0, 2);
			double[][] loccov   = pose.Item2.Covariance.Submatrix(0, 2, 0, 2);

			RenderGaussian(new Gaussian(location, loccov, 1.0), camera, color);
			trajectory.Add(Tuple.Create(pose.Item1, Util.SClone(pose.Item2.Mean)));
		}

		DrawUtils.DrawTrajectory(Graphics, trajectory, color, camera);
	}
}
}
