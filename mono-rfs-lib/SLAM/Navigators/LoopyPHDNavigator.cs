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
using TimedMixture      = System.Collections.Generic.List<System.Tuple<double, double, System.Collections.Generic.List<monorfs.Gaussian>>>;
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
	/// True if the algorithm performs online SLAM, i.e. uses information incrementally,
	/// generating a new estimate each time step; false otherwise (i.e. uses all the
	/// information at once and may run arbitrarily long).
	/// </summary>
	public override bool Online { get  { return false; } }

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
	/// Each message consists of a constant term and a gaussian mixture, i.e.
	/// a factor of the form a_0 + sum(a_i * N(mu, sigma)).
	/// </summary>
	public TimedMixture MessagesFromMap { get; private set; }

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
		InnerFilter = new PHDNavigator<MeasurerT, PoseT, MeasurementT>(vehicle, 1, true);

		TimedArray        odometry;
		TimedMeasurements measurements;

		TimedState trajectory = InitialEstimate(vehicle, commands, initialestimate, out odometry, out measurements);
		Odometry     = odometry;
		Measurements = measurements;
		clock        = 0;

		initMessages(trajectory);

		WayMaps         = new List<Tuple<double, Map>>(3);
		WayTrajectories = new List<Tuple<double, TimedState>>();

		GameTime zero = new GameTime();
		UpdateLPTrajectory(zero);
		UpdateMapHistory(zero);
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

		WayMaps         = new List<Tuple<double, Map>>(3);
		WayTrajectories = new List<Tuple<double, TimedState>>();

		GameTime zero = new GameTime();
		UpdateLPTrajectory(zero);
		UpdateMapHistory(zero);
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
		MessagesFromMap     = new TimedMixture();
		MapMessages         = new List<Tuple<double, PoseT>>();

		double[][] infcov = Util.InfiniteCovariance(OdoSize);

		for (int i = 0; i < initial.Count; i++) {
			LinearizationPoints.Add(Tuple.Create(0.0, new PoseT().IdentityP()));
			MessagesFromPast   .Add(Tuple.Create(0.0, (Gaussian) null));
			MessagesFromFuture .Add(Tuple.Create(0.0, (Gaussian) null));
			MessagesFromMap    .Add(Tuple.Create(0.0, 0.0, (List<Gaussian>) null));
			MapMessages        .Add(Tuple.Create(0.0, new PoseT()));
		}

		Parallel.For(0, initial.Count,
		             new ParallelOptions { MaxDegreeOfParallelism = Config.NParallel}, i => {
			LinearizationPoints[i] = Tuple.Create(initial[i].Item1, new PoseT().FromState(initial[i].Item2));
			MessagesFromPast   [i] = Tuple.Create(initial[i].Item1, new Gaussian(new double[OdoSize], infcov, 1.0));
			MessagesFromFuture [i] = Tuple.Create(initial[i].Item1, new Gaussian(new double[OdoSize], infcov, 1.0));
			MessagesFromMap    [i] = Tuple.Create(initial[i].Item1, 0.0, new List<Gaussian>());
		});

		MessagesFromPast[0] = Tuple.Create(MessagesFromPast[0].Item1, Util.DiracDelta(new double[OdoSize]));

		FusedEstimate = GetFusedEstimate();
		UpdateMapMessages();
	}

	/// <summary>
	/// Reset the model of every particle to an empty map.
	/// </summary>
	public override void ResetMapModel()
	{
		// Unsupported
	}

	/// <summary>
	/// Remove all the localization and mapping history and start it again from the current position.
	/// </summary>
	public override void ResetHistory()
	{
		// Unsupported
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
		int    timeindex   = clock % FusedEstimate.Count;
		double temperature = 5.0 / (clock / FusedEstimate.Count + 1);

		for (int i = 0; i < 2; i++) {
			UpdateMessagesFromPast(timeindex, timeindex + 1);
			UpdateMessagesFromFuture(timeindex, timeindex + 1);
			UpdateMessagesFromMap(timeindex, timeindex + 1, clock + 1, temperature);
			UpdateMapMessages();
		}

		Console.Write(clock + ", ");
		clock++;

		GameTime timestamp      = new GameTime();
		timestamp.TotalGameTime = new TimeSpan(10000000L * clock / FusedEstimate.Count);

		UpdateLPTrajectory(timestamp);
		UpdateMapHistory(timestamp);

	}

	/// <summary>
	/// Get all messages from the past given the current estimates.
	/// </summary>
	public void UpdateMessagesFromPast()
	{
		UpdateMessagesFromPast(0, MessagesFromPast.Count);
	}

	/// <summary>
	/// Get all messages from the future given the current estimates.
	/// </summary>
	public void UpdateMessagesFromFuture()
	{
		UpdateMessagesFromFuture(0, MessagesFromFuture.Count);
	}

	/// <summary>
	/// Get all messages from the map given the current estimates.
	/// </summary>
	public void UpdateMessagesFromMap()
	{
		UpdateMessagesFromMap(0, MessagesFromMap.Count, MessagesFromMap.Count, 0);
	}

	/// <summary>
	/// Get all messages to the map given the current estimates.
	/// </summary>
	public void UpdateMapMessages()
	{
		UpdateMapMessages(0, MapMessages.Count);
	}

	/// <summary>
	/// Get messages from the past given the current estimates.
	/// </summary>
	/// <param name="from">Update messages starting from this index (inclusive).</param>
	/// <param name="to">Update messages up to this index (exclusive).</param>
	public void UpdateMessagesFromPast(int from, int to)
	{
		from = Math.Max(1, from);
		to   = Math.Min(MessagesFromPast.Count, to);

		for (int i = from; i < to; i++) {
			PoseT   linearpoint     = LinearizationPoints[i]    .Item2;
			PoseT   prevlinearpoint = LinearizationPoints[i - 1].Item2;
			Gaussian fusedF     = FusedEstimate     [i - 1].Item2;
			Gaussian futureF    = MessagesFromFuture[i - 1].Item2;
			Gaussian halffusedF = Gaussian.Unfuse(fusedF, futureF);
			
			PoseT hfpose  = prevlinearpoint.Add(halffusedF.Mean);
			PoseT estpose = hfpose.AddOdometry(Odometry[i - 1].Item2);
			
			double[][] jacobian = MotionJacobian(prevlinearpoint, linearpoint,
			                                     halffusedF.Mean, Odometry[i - 1].Item2);
			double[][] newcovariance = jacobian.Multiply(halffusedF.Covariance).MultiplyByTranspose(jacobian);

			Gaussian estimate = new Gaussian(estpose.Subtract(linearpoint),
			                                 newcovariance.Add(MotionCovariance), 1.0);
			// simply adding the covariances is an approximation, since the full relation between
			// estimates is d2 = linear + d1 +' odo - linear, which is nonlinear, (and so,
			// a jacobian transformation would be required); nevertheless, when the deviations from
			// the linearization point are small, pose-deltas are a unitary linear space
			// (and they are expected to be small)

			MessagesFromPast[i] = Tuple.Create(MessagesFromPast[i].Item1, estimate);

			FusedEstimate[i] = Tuple.Create(FusedEstimate[i].Item1,
				GetFusedEstimate(MessagesFromPast[i].Item2, MessagesFromFuture[i].Item2,
				                 Tuple.Create(MessagesFromMap[i].Item2, MessagesFromMap[i].Item3)));
		}
	}

	/// <summary>
	/// Get messages from the future given the current estimates.
	/// </summary>
	/// <param name="from">Update messages starting from this index (inclusive).</param>
	/// <param name="to">Update messages up to this index (exclusive).</param>
	public void UpdateMessagesFromFuture(int from, int to)
	{
		from = Math.Max(0, from);
		to   = Math.Min(MessagesFromFuture.Count - 1, to);

		for (int i = to - 1; i >= from; i--) {
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

			MessagesFromFuture[i] = Tuple.Create(MessagesFromFuture[i].Item1, estimate);

			FusedEstimate[i] = Tuple.Create(FusedEstimate[i].Item1,
				GetFusedEstimate(MessagesFromPast[i].Item2, MessagesFromFuture[i].Item2,
				                 Tuple.Create(MessagesFromMap[i].Item2, MessagesFromMap[i].Item3)));
		}
	}

	/// <summary>
	/// Get messages from the map given the current estimates.
	/// </summary>
	/// <param name="from">Update messages starting from this index (inclusive).</param>
	/// <param name="to">Update messages up to this index (exclusive).</param>
	/// <param name="tofilter">Run the filter up to this index (exclusive).</param>
	/// <param name="temperature">Smoothing term in [0, Inf); the biggest,
	/// the less relevant these messages become.</param>
	public void UpdateMessagesFromMap(int from, int to, int tofilter, double temperature)
	{
		// the mixture models must have a gaussian inside beause later on it is
		// assumed that it can be merged into a single gaussian; not having at least
		// one component makes it impossible to guess the number of dimensions; so
		// to make everything smoother, just add an uninformative component
		List<Gaussian> dummymix = new List<Gaussian>();
		var dummyestimate = Tuple.Create(0.0, dummymix);

		MeasurementT dummyM = new MeasurementT();

		from = Math.Max(0, from);
		to   = Math.Min(MessagesFromMap.Count, to);

		Parallel.For(from, to,
		             new ParallelOptions { MaxDegreeOfParallelism = Config.NParallel}, i => {
			var      mixestimate = dummyestimate;
			Gaussian pastfuture  = Gaussian.Fuse(MessagesFromPast[i].Item2, MessagesFromFuture[i].Item2);

			if (Measurements[i].Item2.Count > 0) {
				// typed list (instead of list of array of doubles)
				List<MeasurementT> measurements = Measurements[i].Item2.ConvertAll(m => dummyM.FromLinear(m));

				Map dMap    = FilterMissing(MapMessages, Measurements, i, tofilter);
				mixestimate = GuidedFitMixture(pastfuture.Mean, measurements, dMap, LinearizationPoints[i].Item2);

				for (int k = 0; k < mixestimate.Item2.Count; k++) {
					Gaussian original = mixestimate.Item2[k];
					mixestimate.Item2[k] = new Gaussian(original.Mean,
					                                    original.Covariance.Add(
					                                        (1 + temperature).Multiply(pastfuture.Covariance)),
					                                    original.Weight);
				}
			}

			MessagesFromMap[i] = Tuple.Create(MapMessages[i].Item1, mixestimate.Item1, mixestimate.Item2);
			FusedEstimate  [i] = Tuple.Create(FusedEstimate[i].Item1,
			                                  GetFusedEstimate(MessagesFromPast[i].Item2,
			                                                   MessagesFromFuture[i].Item2,
			                                                   mixestimate));
		});
	}

	/// <summary>
	/// Get messages to the map given the current estimates.
	/// </summary>
	/// <param name="from">Update messages starting from this index (inclusive).</param>
	/// <param name="to">Update messages up to this index (exclusive).</param>
	public void UpdateMapMessages(int from, int to)
	{
		TimedGaussian hfused = FuseGaussians(MessagesFromPast, MessagesFromFuture);

		from = Math.Max(0, from);
		to   = Math.Min(MapMessages.Count, to);

		Parallel.For(from, to,
		             new ParallelOptions { MaxDegreeOfParallelism = Config.NParallel}, i => {
			PoseT pose  = LinearizationPoints[i].Item2.Add(hfused[i].Item2.Mean);
			MapMessages[i] = Tuple.Create(hfused[i].Item1, pose);
		});

		if (from < to) {  // if anything changed, invalidate the map cache
			MapCached = false;
		}
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
		TimedGaussian fused = new TimedGaussian(MessagesFromPast.Count);

		for (int i = 0; i < MessagesFromPast.Count; i++) {
			Gaussian reduced = GetFusedEstimate(MessagesFromPast[i].Item2, MessagesFromFuture[i].Item2,
			                                    Tuple.Create(MessagesFromMap[i].Item2, MessagesFromMap[i].Item3));
			fused.Add(Tuple.Create(MessagesFromPast[i].Item1, reduced));
		}

		return fused;
	}

	/// <summary>
	/// Get current pose estimates from message factors.
	/// </summary>
	public Gaussian GetFusedEstimate(Gaussian past, Gaussian future, Tuple<double, List<Gaussian>> map)
	{
		// should be Gaussian.Multiply, but it is equivalent 
		// (as the fusion only addds a constant multiplicative constant)
		return Mixdown(Fuse(Gaussian.Fuse(past, future), map));
	}

	/// <summary>
	/// Fuse a gaussian with a mixture model.
	/// </summary>
	/// <param name="gaussian">Gaussian component.</param>
	/// <param name="mixture">Mixture model.</param>
	/// <returns></returns>
	public static List<Gaussian> Fuse(Gaussian gaussian, Tuple<double, List<Gaussian>> mixture)
	{
		List<Gaussian> fused   = new List<Gaussian>(mixture.Item2.Count + 1); 
		double         wsum    = 0;
		double         invwsum = 0;

		// weight does not matter, everything is multiplied by the same constant
		// (and will be normalized at the end);
		// a unitary weight gives better FP precision
		gaussian = gaussian.Reweight(1.0);

		// constant term multiplication
		Gaussian prod = gaussian.Reweight(Math.Exp(mixture.Item1));
		fused.Add(prod);
		wsum = prod.Weight;

		// gaussian mixture multiplications
		foreach (Gaussian component in mixture.Item2) {
			prod = Gaussian.Multiply(gaussian, component);
			fused.Add(prod);
			wsum += prod.Weight;
		}

		invwsum = 1.0 / wsum;

		for (int i = 0; i < fused.Count; i++) {
			fused[i] = fused[i].Reweight(fused[i].Weight * invwsum);
		}

		return fused;
	}

	/// <summary>
	/// Compress a gaussian mixture model into one component.
	/// Information will be lost; depending on the scenario,
	/// the output could be so uncertain as to be useless.
	/// </summary>
	/// <param name="mixture">Mixture model.</param>
	/// <returns>Merged gaussian.</returns>
	public static Gaussian Mixdown(List<Gaussian> mixture)
	{
		return Gaussian.Merge(mixture);
	}

	/// <summary>
	/// Compress gaussian mixture models into one component each (disregards the constant terms).
	/// Information will be lost; depending on the scenario, the output could be so
	/// uncertain as to be useless.
	/// </summary>
	/// <param name="mixtures">Timed mixture model.</param>
	/// <returns>Merged gaussians.</returns>
	public static TimedGaussian Mixdown(TimedMixture mixtures)
	{
		TimedGaussian mixdown = new TimedGaussian(mixtures.Count);

		foreach (var mixture in mixtures) {
			mixdown.Add(Tuple.Create(mixture.Item1, Gaussian.Merge(mixture.Item3)));
		}

		return mixdown;
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
		return FilterMissing(trajectory, factors, trajectory.Count, trajectory.Count);
	}

	/// <summary>
	/// Get a map estimate by using the PHD filter.
	/// </summary>
	/// <param name="trajectory">Trajectory estimate.</param>
	/// <param name="factors">Map factors.</param>
	/// <param name="index">Removed factor index from the filtering;
	/// if it's outside of the trajectory time boundaries, it will filter normally.</param>
	/// <param name="to">Filter up to this index (exclusive).</param> 
	public Map FilterMissing(List<Tuple<double, PoseT>> trajectory, TimedMeasurements factors, int index, int to)
	{
		InnerFilter = new PHDNavigator<MeasurerT, PoseT, MeasurementT>(InnerFilter.RefVehicle, 1, true);

		to    = Math.Min(trajectory.Count, to);
		index = (index < 0) ? to : Math.Min(to, index);

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

		for (int i = index + 1; i < to; i++) {
			GameTime time = new GameTime(new TimeSpan((int) (trajectory[i].Item1 * 100000000)), Config.MeasureElapsed);

			InnerFilter.BestEstimate.Pose = trajectory[i].Item2;
			InnerFilter.BestEstimate.WayPoints.Add(Tuple.Create(trajectory[i].Item1, trajectory[i].Item2.State));

			InnerFilter.SlamUpdate(time, factors[i].Item2.ConvertAll(m => dummy.FromLinear(m)));
		}

		return InnerFilter.BestMapModel;
	}

	/// <summary>
	/// Fit a gaussian mixture to the pose distribution
	/// using gradient descent and quadratic regression.
	/// To guide the fitting process, a number of best guesses
	/// are used to initialize the estimation based on the
	/// best data association available.
	/// </summary>
	/// <param name="pose0">Pose initial estimate.</param>
	/// <param name="measurements">Fixed measurements.</param>
	/// <param name="map">Fixed map.</param>
	/// <param name="linearpoint">Linearization point.</param>
	/// <returns>Fitted gaussian mixture.</returns>
	public static Tuple<double, List<Gaussian>> GuidedFitMixture(double[] pose0,
	                                                             List<MeasurementT> measurements,
	                                                             Map map, PoseT linearpoint)
	{
		List<double[]> guesses  = new List<double[]>();
		MeasurerT      measurer = new MeasurerT();
		PoseT          initpose = linearpoint.Add(pose0);

		Map jmap = map.BestMapEstimate;

		guesses.Add(pose0);

		foreach (Gaussian landmark in jmap) {
			foreach (MeasurementT measurement in measurements) {
				PoseT guess = measurer.FitToMeasurement(initpose, measurement, landmark.Mean);

				if (guess.Subtract(initpose).SquareEuclidean() < 0.5 * 0.5) {
					guesses.Add(guess.Subtract(linearpoint));
				}
			}
		}

		double[]       maxpose    = pose0;
		double[]       infdelta   = new double[OdoSize];
		List<Gaussian> components = new List<Gaussian>();

		for (int k = 0; k < OdoSize; k++) {
			infdelta[k] = 1e5;
		}

		// Pose very far from the map (distance to everything is > 10-sigma)
		PoseT infpose     = new PoseT().IdentityP().Add(infdelta);
		var infvehicle    = new SimulatedVehicle<MeasurerT, PoseT, MeasurementT>(infpose, new List<double[]>());
		double emptyspace = PHDNavigator<MeasurerT, PoseT, MeasurementT>.
		                        QuasiSetLogLikelihood(measurements, jmap, infvehicle);

		foreach (double[] guess in guesses) {
			double localmax = PHDNavigator<MeasurerT, PoseT, MeasurementT>.
				QuasiSetLogLikelihood(measurements, jmap,
					new SimulatedVehicle<MeasurerT, PoseT, MeasurementT>(
						linearpoint.Add(guess), new List<double[]>())
					);

			if (localmax - emptyspace < 0) {
				continue;
			}

			double[] localpose = LogLikeGradientAscent(guess, measurements, jmap, linearpoint, out localmax);

			bool alreadycounted = false;
			foreach (Gaussian component in components) {
				if (component.Mahalanobis(localpose) < 0.1) {
					alreadycounted = true;
					break;
				}
			}

			if (alreadycounted) {
				continue;
			}

			double[][] localcov = LogLikeFitCovariance(maxpose, measurements, jmap, linearpoint);

			double logmultiplier = Math.Log(Math.Pow(2 * Math.PI, -localpose.Length / 2) / Math.Sqrt(localcov.PseudoDeterminant()));
			double weight        = Math.Exp(localmax - logmultiplier);

			components.Add(new Gaussian(localpose, localcov, weight));
		}

		return Tuple.Create(emptyspace, components);
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
		var          dvehicle = new SimulatedVehicle<MeasurerT, PoseT, MeasurementT>();

		//dvehicle.Pose = linearpoint.Add(pose);

		//PHDNavigator.QuasiSetLogLikelihood(measurements, map, dvehicle, out gradient);

		for (int i = 0; i < gradient.Length; i++) {
			double[] ds = new double[OdoSize];

			ds[i]         = eps;
			dvehicle.Pose = linearpoint.Add(pose.Add(ds));
			double lplus  = PHDNavigator<MeasurerT, PoseT, MeasurementT>.
			                    QuasiSetLogLikelihood(measurements, map, dvehicle);

			ds[i]         = -eps;
			dvehicle.Pose = linearpoint.Add(pose.Add(ds));
			double lminus = PHDNavigator<MeasurerT, PoseT, MeasurementT>.
			                    QuasiSetLogLikelihood(measurements, map, dvehicle);

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
		const double loglikethreshold = 1e-3;

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
			PHDNavigator<MeasurerT, PoseT, MeasurementT>.
				QuasiSetLogLikelihood(measurements, map, nextvehicle, out gradient);
			double gradsize = gradient.Euclidean();

			if (gradsize > GradientClip) {
				gradient = gradient.Multiply(GradientClip / gradsize);
			}

			multiplier = GradientAscentRate;

			int counter = 0;
			do {
				nextpose         = pose.Add(multiplier.Multiply(gradient));
				nextvehicle.Pose = linearpoint.Add(nextpose);
				nextloglike      = PHDNavigator<MeasurerT, PoseT, MeasurementT>.
				                       QuasiSetLogLikelihood(measurements, map, nextvehicle);
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
		const double eps     = 1e-5;
		double[][]   hessian = new double[OdoSize][];
		var          vehicle = new SimulatedVehicle<MeasurerT, PoseT, MeasurementT>();

		for (int i = 0; i < hessian.Length; i++) {
			double[] ds = new double[OdoSize];
			double[] lplus;
			double[] lminus;

			ds[i]         = eps;
			vehicle.Pose = linearpoint.Add(pose.Add(ds));
			PHDNavigator<MeasurerT, PoseT, MeasurementT>.
		    	QuasiSetLogLikelihood(measurements, map, vehicle, out lplus);

			ds[i]         = -eps;
			vehicle.Pose = linearpoint.Add(pose.Add(ds));
			PHDNavigator<MeasurerT, PoseT, MeasurementT>.
		    	QuasiSetLogLikelihood(measurements, map, vehicle, out lminus);

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
	/// Update the trajectory log with the latest estimate,
	/// considering that each estimate has an associated
	/// linearization point.
	/// </summary>
	/// <param name="time">Time.</param>
	public void UpdateLPTrajectory(GameTime time)
	{
		TimedState trajectory = new TimedState();

		for (int i = 0; i < FusedEstimate.Count; i++) {
			double[] location = LinearizationPoints[i].Item2.Add(FusedEstimate[i].Item2.Mean).State;
			trajectory.Add(Tuple.Create(FusedEstimate[i].Item1, Util.SClone(location)));
		}

		WayTrajectories.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, trajectory));
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
		RenderMeasurements(camera);

		//RefVehicle.RenderBody(camera);
		RenderTimedGaussian(MessagesFromPast, Color.Red, camera);
		RenderTimedGaussian(MessagesFromFuture, Color.Green, camera);
		RenderTimedGaussian(Mixdown(MessagesFromMap), Color.Orange, camera);
		RenderTimedGaussian(FusedEstimate, new Color(128, 120, 160, 140), camera);
		RenderMap(camera);

		foreach (Gaussian component in MessagesFromMap[9].Item3) {
			double[] location = LinearizationPoints[9].Item2.Add(component.Mean).Location;

			double[][] loccov = MatrixExtensions.Zero(3);
			double[][] cov2d  = component.Covariance;
			for (int h = 0; h < cov2d.Length;    h++) {
			for (int k = 0; k < cov2d[h].Length; k++) {
				loccov[h][k] = cov2d[h][k];
			}
			}

			RenderGaussian(new Gaussian(location, loccov, component.Weight), camera, Color.Green);
		}
	}

	public void RenderMeasurements(double[][] camera)
	{
		MeasurerT    measurer = new MeasurerT();
		MeasurementT mdummy   = new MeasurementT();
		double[][]   vertices = new double[7][];

		for (int i = 0; i < FusedEstimate.Count; i++) {
			PoseT pose  = LinearizationPoints[i].Item2.Add(FusedEstimate[i].Item2.Mean);
			vertices[0] = camera.TransformH(pose.Location);

			foreach (double[] measurement in Measurements[i].Item2) {
				double[] mmap = camera.TransformH(measurer.MeasureToMap(pose, mdummy.FromLinear(measurement)));
				vertices[1]   = mmap;
				vertices[2]   = mmap.Add(new double[3] { 0.05,  0.05, 0});
				vertices[3]   = mmap.Add(new double[3] {-0.05, -0.05, 0});
				vertices[4]   = mmap;
				vertices[5]   = mmap.Add(new double[3] {-0.05,  0.05, 0});
				vertices[6]   = mmap.Add(new double[3] { 0.05, -0.05, 0});

				graphics.DrawUser2DPolygon(vertices, 0.02f, new Color(255, 0, 0, 160), false);
			}
		}
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

		for (int i = 0; i < gaussians.Count; i++) {
			if (gaussians[i].Item2 == null) {
				continue;
			}

			PoseT pose = LinearizationPoints[i].Item2.Add(gaussians[i].Item2.Mean);

			trajectory.Add(Tuple.Create(gaussians[i].Item1, Util.SClone(pose.State)));
		}

		DrawUtils.DrawTrajectory<PoseT>(Graphics, trajectory, color, camera);
	}
}
}
