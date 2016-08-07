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
using System.Threading.Tasks;

using Accord.Math;
using Accord.Math.Decompositions;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

using TimedState        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedArray        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedMapModel     = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<monorfs.Gaussian>>>;
using TimedGaussian     = System.Collections.Generic.List<System.Tuple<double, monorfs.Gaussian>>;
using TimedMeasurements = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<double[]>>>;

namespace monorfs
{
/// <summary>
/// SLAM solver. It uses the PHD loopy belief propagator.
/// </summary>
public class LoopyPHDNavigator<MeasurerT, PoseT, MeasurementT> : Navigator<MeasurerT, PoseT, MeasurementT>
	where PoseT        : IPose<PoseT>, new()
	where MeasurementT : IMeasurement<MeasurementT>, new()
	where MeasurerT    : IMeasurer<MeasurerT, PoseT, MeasurementT>, new()
{
	/// <summary>
	/// Internal motion model covariance matrix. Lie algebra representation.
	/// </summary>
	public double[][] MotionCovariance { get { return Config.MotionCovariance; } }

	/// <summary>
	/// Learning rate for the gradient descent method.
	/// </summary>
	public static double GradientAscentRate { get { return Config.GradientAscentRate; } }

	/// <summary>
	/// Clip value for gradient calculations. Avoids moving into bad areas due to large gradients.
	/// </summary>
	public static double GradientClip { get { return Config.GradientClip; } }

	/// <summary>
	/// Inner PHD filter algorithm.
	/// </summary>
	public PHDNavigator<MeasurerT, PoseT, MeasurementT> InnerFilter { get; private set; }

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
	/// Current pose estimates.
	/// </summary>
	public TimedGaussian FusedEstimate { get; private set; }

	/// <summary>
	/// Linearization point for each pose in the trajectory.
	/// </summary>
	public List<Tuple<double, PoseT>> LinearizationPoints { get; private set; }

	/// <summary>
	/// Measurement sets for RFS map update delta factors.
	/// </summary>
	public TimedMeasurements Measurements { get; private set; }

	/// <summary>
	/// Odometry readings.
	/// </summary>
	public TimedArray Odometry { get; private set; }

	/// <summary>
	/// Internal clock for tick-tocking and schedule the messages.
	/// </summary>
	private int clock;

	/// <summary>
	/// Internal representation of the pose trajectory for RFS map update delta factors.
	/// </summary>
	private List<Tuple<double, PoseT>> mapMessages;

	/// <summary>
	/// Pose trajectory for RFS map update delta factors.
	/// </summary>
	public List<Tuple<double, PoseT>> MapMessages
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
	/// Most accurate estimate of the current vehicle pose.
	/// </summary>
	public override TrackVehicle<MeasurerT, PoseT, MeasurementT> BestEstimate
	{
		get
		{
			var maxlike = new TrackVehicle<MeasurerT, PoseT, MeasurementT>();

			maxlike.WayPoints = new TimedState();

			foreach (var pose in FusedEstimate) {
				maxlike.WayPoints.Add(Tuple.Create(pose.Item1, Util.SClone(pose.Item2.Mean)));
			}

			maxlike.Pose = new PoseT().FromState(maxlike.WayPoints[maxlike.WayPoints.Count - 1].Item2);

			return maxlike;
		}
	}

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
				CachedMapModel = Filter(MapMessages, Measurements);
			}

			MapCached = true;
			return CachedMapModel;
		}
	}

	/// <summary>
	/// Render output.
	/// </summary>
	public override GraphicsDevice Graphics {
		get
		{
			return graphics;
		}
		set
		{
			graphics = value;

			if (RefVehicle.Graphics == null) {
				RefVehicle.Graphics = value;
			}
		}
	}

	/// <summary>
	/// Construct a PHDNavigator using the indicated vehicle as a reference.
	/// </summary>
	/// <param name="vehicle">Vehicle to track.</param>
	/// <param name="commands">List of commands given to the vehicle.</param>
	/// <param name="initialestimate">Navigation algorithm used to get an initial estimate.</param>
	public LoopyPHDNavigator(Vehicle<MeasurerT, PoseT, MeasurementT> vehicle,
	                         List<double[]> commands,
	                         Navigator<MeasurerT, PoseT, MeasurementT> initialestimate)
		: base(vehicle, false)
	{
		InnerFilter   = new PHDNavigator<MeasurerT, PoseT, MeasurementT>(vehicle, 1, true);

		TimedArray        odometry;
		TimedMeasurements measurements;

		TimedState trajectory = InitialEstimate(vehicle, commands, initialestimate, out odometry, out measurements);
		Odometry     = odometry;
		Measurements = measurements;
		clock        = 0;

		initMessages(trajectory);
	}

	/// <summary>
	/// Construct a LoopyPHDNavigator using precalculated estimates.
	/// </summary>
	/// <param name="vehicle">Vehicle to track.</param>
	/// <param name="trajectory">Initial trajectory estimate.</param>
	/// <param name="odometry">Odometry readings.</param>
	/// <param name="measurements">Measurement readings.</param>
	public LoopyPHDNavigator(Vehicle<MeasurerT, PoseT, MeasurementT> vehicle, TimedState trajectory,
	                         TimedArray odometry, TimedMeasurements measurements)
		: base(vehicle, false)
	{
		InnerFilter  = new PHDNavigator<MeasurerT, PoseT, MeasurementT>(vehicle, 1, true);
		Odometry     = odometry;
		Measurements = measurements;
		clock        = 0;

		Odometry    .Add(Tuple.Create(double.PositiveInfinity, new double[OdoSize]));
		Measurements.Insert(0, Tuple.Create(double.PositiveInfinity, new List<double[]>()));

		initMessages(trajectory);
	}

	/// <summary>
	/// Generate the first messages between nodes.
	/// </summary>
	/// <param name="initial">Initial estimation of the trajectory.</param>
	private void initMessages(TimedState initial)
	{
		LinearizationPoints = new List<Tuple<double, PoseT>>();
		MessagesFromPast    = new TimedGaussian();
		MessagesFromFuture  = new TimedGaussian();
		MessagesFromMap     = new TimedGaussian();

		double[][] infcov = Util.InfiniteCovariance(OdoSize);

		for (int i = 0; i < initial.Count; i++) {
			LinearizationPoints.Add(Tuple.Create(0.0, new PoseT().IdentityP()));
			MessagesFromPast   .Add(Tuple.Create(0.0, (Gaussian) null));
			MessagesFromFuture .Add(Tuple.Create(0.0, (Gaussian) null));
			MessagesFromMap    .Add(Tuple.Create(0.0, (Gaussian) null));
		}

		Parallel.For(0, initial.Count,
		             new ParallelOptions { MaxDegreeOfParallelism = Config.NParallel}, i => {
			LinearizationPoints[i] = Tuple.Create(initial[i].Item1, new PoseT().FromState(initial[i].Item2));
			MessagesFromPast   [i] = Tuple.Create(initial[i].Item1, new Gaussian(new double[OdoSize], infcov, 1.0));
			MessagesFromFuture [i] = Tuple.Create(initial[i].Item1, new Gaussian(new double[OdoSize], infcov, 1.0));
			MessagesFromMap    [i] = Tuple.Create(initial[i].Item1, new Gaussian(new double[OdoSize], infcov, 1.0));
		});

		MessagesFromPast[0] = Tuple.Create(MessagesFromPast[0].Item1, Util.DiracDelta(new double[OdoSize]));

		FusedEstimate = GetFusedEstimate();
		MapMessages   = GetMapMessages();
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
	public TimedState InitialEstimate(Vehicle<MeasurerT, PoseT, MeasurementT> vehicle,
	                                  List<double[]> commands,
	                                  Navigator<MeasurerT, PoseT, MeasurementT> initnav,
	                                  out TimedArray odometry, out TimedMeasurements measurements)
	{
		var sim = new Simulation<MeasurerT, PoseT, MeasurementT>("", vehicle, initnav, commands, true, false);

		sim.RunHeadless();

		odometry     = sim.Explorer.WayOdometry;
		measurements = sim.WayMeasurements;

		return sim.Navigator.WayTrajectories[sim.Navigator.WayTrajectories.Count - 1].Item2;
	}

	/// <summary>
	/// Updates the state of the vehicle.
	/// In this class, does nothing.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="reading">Odometry reading (dx, dy, dz, dpitch, dyaw, droll).</param>
	public override void Update(GameTime time, double[] reading) {}

	/// <summary>
	/// Globally update the vehicle and map estimates.
	/// This method is the core of the whole program.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="measurements">Sensor measurements in pixel-range form. Not used.</param>
	public override void SlamUpdate(GameTime time, List<MeasurementT> measurements)
	{
		if (clock % 2 == 0) {
			MessagesFromPast = GetMessagesFromPast();
			MapMessages      = GetMapMessages();
			MessagesFromMap  = GetMessagesFromMap();
		}
		else {
			MessagesFromFuture = GetMessagesFromFuture();
			MessagesFromMap    = GetMessagesFromMap();
			MapMessages        = GetMapMessages();
		}

		Console.WriteLine((clock++ % 2 == 0) ? "tick" : "tock");
	}

	/// <summary>
	/// Get messages from the past given the current estimates.
	/// </summary>
	/// <returns>Messages from the past.</returns>
	public TimedGaussian GetMessagesFromPast()
	{
		TimedGaussian messages = new TimedGaussian();

		messages.Add(Tuple.Create(MessagesFromPast[0].Item1, Util.DiracDelta(new double[OdoSize])));

		for (int i = 1; i < MessagesFromPast.Count; i++) {
			//messages.Add(Tuple.Create(0.0, (Gaussian) null));
			messages.Add(MessagesFromPast[i]);
		}

		for (int i = 1; i < MessagesFromPast.Count; i++) {
		//Parallel.For(1, MessagesFromPast.Count,
		//             new ParallelOptions { MaxDegreeOfParallelism = Config.NParallel}, i => {
			PoseT   linearpoint     = LinearizationPoints[i]    .Item2;
			PoseT   prevlinearpoint = LinearizationPoints[i - 1].Item2;
			Gaussian fusedF     = FusedEstimate     [i - 1].Item2;
			Gaussian futureF    = MessagesFromFuture[i - 1].Item2;
			Gaussian halffusedF = Gaussian.Unfuse(fusedF, futureF);
			
			PoseT hfpose  = prevlinearpoint.Add(halffusedF.Mean);
			PoseT estpose = hfpose.AddOdometry(Odometry[i - 1].Item2);
			
			double[][] jacobian = MotionJacobian(prevlinearpoint, linearpoint,
			                                     halffusedF.Mean, Odometry[i-1].Item2);
			double[][] newcovariance = jacobian.Multiply(halffusedF.Covariance).MultiplyByTranspose(jacobian);

			Gaussian estimate = new Gaussian(estpose.Subtract(linearpoint),
			                                 newcovariance.Add(MotionCovariance), 1.0);
			// simply adding the covariances is an approximation, since the full relation between
			// estimates is d2 = linear + d1 +' odo - linear, which is nonlinear, (and so,
			// a jacobian transformation would be required); nevertheless, when the deviations from
			// the linearization point are small, pose-deltas are a unitary linear space
			// (and they are expected to be small)

			messages[i] = Tuple.Create(MessagesFromPast[i].Item1, estimate);

			FusedEstimate[i] = Tuple.Create(FusedEstimate[i].Item1,
				Gaussian.Fuse(Gaussian.Fuse(messages[i].Item2,
				                            MessagesFromFuture[i].Item2), MessagesFromMap[i].Item2));
		//});
		}
			
		return messages;
	}

	/// <summary>
	/// Get messages from the future given the current estimates.
	/// </summary>
	/// <returns>Messages from the future.</returns>
	public TimedGaussian GetMessagesFromFuture()
	{
		TimedGaussian messages = new TimedGaussian();

		for (int i = 0; i < MessagesFromFuture.Count - 1; i++) {
			//messages.Add(Tuple.Create(0.0, (Gaussian) null));
			messages.Add(MessagesFromFuture[i]);
		}

		var lastpose = FusedEstimate[FusedEstimate.Count - 1];
		messages.Add(Tuple.Create(lastpose.Item1,
		                          new Gaussian(lastpose.Item2.Mean, Util.InfiniteCovariance(OdoSize), 1.0)));

		for (int i = MessagesFromFuture.Count - 2; i >= 0 ; i--) {
		//Parallel.For(0, MessagesFromFuture.Count - 1,
		//             new ParallelOptions { MaxDegreeOfParallelism = Config.NParallel}, i => {
			PoseT   linearpoint     = LinearizationPoints[i]    .Item2;
			PoseT   nextlinearpoint = LinearizationPoints[i + 1].Item2;
			Gaussian fusedG     = FusedEstimate   [i + 1].Item2;
			Gaussian pastG      = MessagesFromPast[i + 1].Item2;
			Gaussian halffusedG = Gaussian.Unfuse(fusedG, pastG);
			
			PoseT hfpose  = nextlinearpoint.Add(halffusedG.Mean);
			PoseT estpose = hfpose.AddOdometry((-1.0).Multiply(Odometry[i].Item2));
			
			double[][] jacobian = MotionJacobian(linearpoint, nextlinearpoint,
			                                     estpose.Subtract(linearpoint), Odometry[i].Item2)
			                          .PseudoInverse();
			double[][] newcovariance = jacobian.Multiply(halffusedG.Covariance).MultiplyByTranspose(jacobian);

			Gaussian estimate = new Gaussian(estpose.Subtract(linearpoint),
			                                 newcovariance.Add(MotionCovariance), 1.0);
			// simply adding the covariances is an approximation, since the full relation between
			// estimates is d2 = linear + d1 -' odo - linear, which is nonlinear, (and so,
			// a jacobian transformation would be required); nevertheless, when the deviations from
			// the linearization point are small, pose-deltas are a unitary linear space
			// (and they are expected to be small)

			messages[i] = Tuple.Create(MessagesFromFuture[i].Item1, estimate);

			FusedEstimate[i] = Tuple.Create(FusedEstimate[i].Item1,
				Gaussian.Fuse(Gaussian.Fuse(messages[i].Item2,
				                            MessagesFromPast[i].Item2), MessagesFromMap[i].Item2));
		//});
		}

		return messages;
	}

	/// <summary>
	/// Get messages from the map given the current estimates.
	/// </summary>
	/// <returns>Messages from the map to the poses.</returns>
	public TimedGaussian GetMessagesFromMap()
	{
		TimedGaussian messages = new TimedGaussian();

		for (int i = 0; i < MessagesFromMap.Count; i++) {
			messages.Add(Tuple.Create(0.0, (Gaussian) null));
		}

		double[][] infcovariance = Util.InfiniteCovariance(OdoSize);

		MeasurementT dummy = new MeasurementT();

		for (int i = 0; i < MessagesFromMap.Count; i++) {
		//Parallel.For(0, MessagesFromMap.Count,
		//             new ParallelOptions { MaxDegreeOfParallelism = Config.NParallel}, i => {
			Gaussian estimate = null;

			if (Measurements[i].Item2.Count > 0) {
				Map dMap = FilterMissing(MapMessages, Measurements, i);
				estimate = GuidedFitGaussian(FusedEstimate[i].Item2.Mean, Measurements[i].Item2.ConvertAll(m => dummy.FromLinear(m)),
				                       dMap, LinearizationPoints[i].Item2);
			}
			else {
					estimate = new Gaussian(FusedEstimate[i].Item2.Mean, infcovariance, 1.0);
			}
			
			messages[i] = Tuple.Create(MapMessages[i].Item1, estimate);

			FusedEstimate[i] = Tuple.Create(FusedEstimate[i].Item1,
				Gaussian.Fuse(Gaussian.Fuse(messages[i].Item2,
				                            MessagesFromPast[i].Item2), MessagesFromFuture[i].Item2));
		//});
		}

		return messages;
	}

	/// <summary>
	/// Get messages to the map given the current estimates.
	/// </summary>
	/// <returns>Messages from the poses to the map.</returns>
	public List<Tuple<double, PoseT>> GetMapMessages()
	{
		var           messages = new List<Tuple<double, PoseT>>();
		TimedGaussian hfused   = FuseGaussians(MessagesFromPast, MessagesFromFuture);

		for (int i = 0; i < hfused.Count; i++) {
			messages.Add(Tuple.Create(0.0, new PoseT().IdentityP()));
		}

		Parallel.For(0, hfused.Count,
		             new ParallelOptions { MaxDegreeOfParallelism = Config.NParallel}, i => {
			PoseT pose  = LinearizationPoints[i].Item2.Add(hfused[i].Item2.Mean);
			messages[i] = Tuple.Create(hfused[i].Item1, pose);
		});

		return messages;
	}

	/// <summary>
	/// Calculate the motion jacobian between poses, considering the effect of
	/// the linearization points, which rotate the frame of reference.
	/// </summary>
	/// <param name="prevlinear">Previous linearization point.</param>
	/// <param name="linear">Linearization point.</param>
	/// <param name="prevmean">Previous pose mean estimate.</param>
	/// <param name="odometry">Odometry reading relating both poses.</param>
	/// <returns>Jacobian.</returns>
	public double[][] MotionJacobian(PoseT prevlinear, PoseT linear, double[] prevmean, double[] odometry)
	{
		// In J(p - q), p does not matter as the jacobian does not depend on it (otherwise should be lin + mean)
		double[][] linjacobian     = new PoseT().IdentityP().SubtractJacobian(linear);
		double[][] odojacobian     = prevlinear.Add(prevmean).AddOdometryJacobian(odometry);
		double[][] prevlinjacobian = prevlinear.AddJacobian(prevmean);

		return linjacobian.Multiply(odojacobian).Multiply(prevlinjacobian);
	}

	/// <summary>
	/// Get current pose estimates from message factors.
	/// </summary>
	  public TimedGaussian GetFusedEstimate()
	{
		return FuseGaussians(FuseGaussians(MessagesFromPast, MessagesFromFuture), MessagesFromMap);
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
	/// Get a map estimate by using the PHD filter.
	/// </summary>
	/// <param name="trajectory">Trajectory estimate.</param>
	/// <param name="factors">Map factors.</param>
	public Map Filter(List<Tuple<double, PoseT>> trajectory, TimedMeasurements factors)
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
	public Map FilterMissing(List<Tuple<double, PoseT>> trajectory, TimedMeasurements factors, int index)
	{
		InnerFilter = new PHDNavigator<MeasurerT, PoseT, MeasurementT>(InnerFilter.RefVehicle, 1, true);

		if (index < 0) {
			index = trajectory.Count;
			// if the index is bigger or equal to the trajectory length
			// the second loop won't do anything, i.e. full filter
		}

		MeasurementT dummy = new MeasurementT();

		for (int i = 0; i < index; i++) {
			GameTime time = new GameTime(new TimeSpan((int) (trajectory[i].Item1 * 100000000)), Config.MeasureElapsed);

			InnerFilter.BestEstimate.Pose = trajectory[i].Item2;
			InnerFilter.BestEstimate.WayPoints.Add(Tuple.Create(trajectory[i].Item1, trajectory[i].Item2.State));

			InnerFilter.SlamUpdate(time, factors[i].Item2.ConvertAll(m => dummy.FromLinear(m)));
		}

		// NOTE the frame consecutive to the missing index should have a
		//      time increment of 2dt, since a measurement is missing
		//      however, the PHD does not use dt for any calculations
		//      so it wouldn't do anything. The birth rate could be
		//      proportional to dt, but its effect should be marginal

		for (int i = index + 1; i < trajectory.Count; i++) {
			GameTime time = new GameTime(new TimeSpan((int) (trajectory[i].Item1 * 100000000)), Config.MeasureElapsed);

			InnerFilter.BestEstimate.Pose = trajectory[i].Item2;
			InnerFilter.BestEstimate.WayPoints.Add(Tuple.Create(trajectory[i].Item1, trajectory[i].Item2.State));

			InnerFilter.SlamUpdate(time, factors[i].Item2.ConvertAll(m => dummy.FromLinear(m)));
		}

		return InnerFilter.BestMapModel;
	}

	/// <summary>
	/// Fit a gaussian probability to the pose distribution
	/// using gradient descent and quadratic regression.
	/// To guide the fitting process, a number of best guesses
	/// are used to initialize the estimation based on the
	/// best data association available.
	/// </summary>
	/// <param name="pose0">Pose initial estimate.</param>
	/// <param name="measurements">Fixed measurements.</param>
	/// <param name="map">Fixed map.</param>
	/// <param name="linearpoint">Linearization point.</param>
	/// <returns>Fitted gaussian.</returns>
	public static Gaussian GuidedFitGaussian(double[] pose0, List<MeasurementT> measurements,
	                                         Map map, PoseT linearpoint)
	{
		List<double[]> guesses  = new List<double[]>();
		MeasurerT      measurer = new MeasurerT();
		PoseT          initpose = linearpoint.Add(pose0);

		Map visible = map.FindAll(g => measurer.VisibleM(measurer.MeasurePerfect(initpose, g.Mean)) &&
		                               g.Weight > 0.8);

		guesses.Add(pose0);

		foreach (Gaussian landmark in visible) {
			foreach (MeasurementT measurement in measurements) {
				PoseT   guess = measurer.FitToMeasurement(initpose, measurement, landmark.Mean);
				guesses.Add(guess.Subtract(linearpoint));
			}
		}

		double   maxvalue = double.NegativeInfinity;
		double[] maxpose  = new double[0];

		foreach (double[] guess in guesses) {
			double   localmax;
			double[] localpose = LogLikeGradientAscent(guess, measurements, visible, linearpoint, out localmax);

			if (localmax > maxvalue) {
				maxvalue = localmax;
				maxpose  = localpose;
			}
		}

		double[][] covariance = LogLikeFitCovariance(maxpose, measurements, visible, linearpoint);

		return new Gaussian(maxpose, covariance, maxvalue);
	}

	/// <summary>
	/// Fit a gaussian probability to the pose near its current estimate
	/// using gradient descent and quadratic regression.
	/// </summary>
	/// <param name="pose0">Pose initial estimate.</param>
	/// <param name="measurements">Fixed measurements.</param>
	/// <param name="map">Fixed map.</param>
	/// <param name="linearpoint">Linearization point.</param>
	/// <returns>Fitted gaussian.</returns>
	public static Gaussian FitGaussian(double[] pose0, List<MeasurementT> measurements, Map map, PoseT linearpoint)
	{
		double     maxvalue;
		double[]   maxpose    = LogLikeGradientAscent(pose0, measurements, map, linearpoint, out maxvalue);
		double[][] covariance = LogLikeFitCovariance(maxpose, measurements, map, linearpoint);

		return new Gaussian(maxpose, covariance, 1.0);
	}

	/// <summary>
	/// Calculate gradient of the log-likelihood of
	/// the measurements given the pose, Grad[log P(Z|X)].
	/// </summary>
	/// <param name="pose">Pose where to calculate the gradient.</param>
	/// <param name="measurements">Fixed measurements.</param>
	/// <param name="map">Fixed map.</param>
	/// <param name="linearpoint">Linearization point.</param>
	/// <returns>Gradient.</returns>
	public static double[] LogLikeGradient(double[] pose, List<MeasurementT> measurements, Map map, PoseT linearpoint)
	{
		const double eps      = 1e-5;
		double[]     gradient = new double[OdoSize];
		double[]     gradient2;
		var          dvehicle = new SimulatedVehicle<MeasurerT, PoseT, MeasurementT>();

		//dvehicle.Pose = linearpoint.Add(pose);

		//PHDNavigator.QuasiSetLogLikelihood(measurements, map, dvehicle, out gradient);

		for (int i = 0; i < gradient.Length; i++) {
			double[] ds = new double[OdoSize];

			ds[i]         = eps;
			dvehicle.Pose = linearpoint.Add(pose.Add(ds));
			double lplus  = PHDNavigator<MeasurerT, PoseT, MeasurementT>.
			                    QuasiSetLogLikelihood(measurements, map, dvehicle, out gradient2);

			ds[i]         = -eps;
			dvehicle.Pose = linearpoint.Add(pose.Add(ds));
			double lminus = PHDNavigator<MeasurerT, PoseT, MeasurementT>.
			                    QuasiSetLogLikelihood(measurements, map, dvehicle, out gradient2);

			gradient[i] = (lplus - lminus) / (2 * eps);
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
	/// <param name="linearpoint">Linearization point.</param>
	/// <param name="loglike">Output. Log-likelihood attained at the returned pose.</param>
	/// <returns>Pose that reaches the maximum local log-likelihood.</returns>
	public static double[] LogLikeGradientAscent(double[] initial, List<MeasurementT> measurements, Map map,
	                                             PoseT linearpoint, out double loglike)
	{
		const double loglikethreshold = 1e-5;

		double[] pose        = initial;
		double[] nextpose    = pose;
		var      nextvehicle = new SimulatedVehicle<MeasurerT, PoseT, MeasurementT>(linearpoint.Add(nextpose), new List<double[]>());

		double   prevvalue = double.NegativeInfinity;
		double[] gradient;

		loglike = PHDNavigator<MeasurerT, PoseT, MeasurementT>.
		              QuasiSetLogLikelihood(measurements, map, nextvehicle, out gradient);
		double nextloglike;
		double multiplier;

		while (loglike - prevvalue > loglikethreshold) {
			//gradient   = LogLikeGradient(pose, measurements, map, linearpoint);
			multiplier = GradientAscentRate;
			gradient   = gradient.Clamp(-GradientClip, GradientClip);

			int counter = 0;
			do {
				nextpose         = pose.Add(multiplier.Multiply(gradient));
				nextvehicle.Pose = linearpoint.Add(nextpose);
				nextloglike      = PHDNavigator<MeasurerT, PoseT, MeasurementT>.
				                       QuasiSetLogLikelihood(measurements, map, nextvehicle, out gradient);
				multiplier      /= 2.0;

				counter++;
			} while (nextloglike < loglike && counter < 16);

			prevvalue = loglike;

			if (nextloglike > loglike) {
				pose    = nextpose;
				loglike = nextloglike;
			}
		}

		return pose;
	}

	/// <summary>
	/// Fit a covariance matrix to the surroundings of a pose, assuming
	/// this pose is locally maximal.
	/// </summary>
	/// <param name="pose">Pose.</param>
	/// <param name="measurements">Fixed measurements.</param>
	/// <param name="map">Fixed map.</param>
	/// <param name="linearpoint">Linearization point.</param>
	/// <returns>Estimated covariance matrix.</returns>
	public static double[][] LogLikeFitCovariance(double[] pose, List<MeasurementT> measurements,
	                                              Map map, PoseT linearpoint)
	{
		const double eps      = 1e-5;
		double[][]   hessian  = new double[OdoSize][];

		for (int i = 0; i < hessian.Length; i++) {
			double[] ds = new double[OdoSize];

			ds[i]          = eps;
			double[] lplus = LogLikeGradient(pose.Add(ds), measurements, map, linearpoint);

			ds[i]           = -eps;
			double[] lminus = LogLikeGradient(pose.Add(ds), measurements, map, linearpoint);

			/*if (lplus.HasNaN() || lminus.HasNaN()) {
				Console.WriteLine("Ouch!");
				ds[i]          = eps;
				lplus = LogLikeGradient(pose.Add(ds), measurements, map, linearpoint);

				ds[i]           = -eps;
				lminus = LogLikeGradient(pose.Add(ds), measurements, map, linearpoint);
			}*/

			hessian[i] = (lplus.Subtract(lminus)).Divide(2 * eps);
		}

		if (hessian.HasNaN()) {
			hessian = MatrixExtensions.Zero(OdoSize);
		}

		// any positive eigenvalue corresponds to a maximizable direction
		// (more likely, numerical error around zero). Such directions shall
		// be disregarded (assume infinite covariance, eigval = 0,
		// i.e. no information).
		// They shouldn't happen often since the point should be maximal
		var decomp = new EigenvalueDecomposition(hessian.ToMatrix());
		double[][] diagonal   = decomp.DiagonalMatrix.ToArray();
		double[][] eigvectors = decomp.Eigenvectors.ToArray();

		for (int i = 0; i < diagonal.Length; i++) {
			diagonal[i][i] = Math.Min(0, diagonal[i][i]);
		}

		hessian = eigvectors.Multiply(diagonal).MultiplyByTranspose(eigvectors);

		return (-1.0).Multiply(hessian).PseudoInverse();
	}

	/// <summary>
	/// Render the navigation HUD and the trajectory on the graphics device.
	/// The graphics device must be ready, otherwise
	/// the method will throw an exception.
	/// </summary>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public override void Render(double[][] camera)
	{
		DrawUtils.DrawTrajectory<PoseT>(Graphics, RefVehicle.Groundtruth, Color.Yellow, camera);
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

		for (int i = 0; i < FusedEstimate.Count; i++) {
			double[]   location = LinearizationPoints[i].Item2.Add(gaussians[i].Item2.Mean).State.Submatrix(0, 2);
			double[][] loccov   = gaussians[i].Item2.Covariance.Submatrix(0, 2, 0, 2);

			var decomp       = new SingularValueDecomposition(loccov.ToMatrix());
			double[] svalues = decomp.Diagonal;

			if (svalues.Max() < 1e5) {
				//RenderGaussian(new Gaussian(location, loccov, 1.0), camera, color);
			}
			trajectory.Add(Tuple.Create(gaussians[i].Item1, Util.SClone(LinearizationPoints[i].Item2.Add(gaussians[i].Item2.Mean).State)));
		}

		DrawUtils.DrawTrajectory<PoseT>(Graphics, trajectory, color, camera);
	}
}
}
