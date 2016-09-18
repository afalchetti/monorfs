// PHDNavigator.cs
// SLAM solving navigator using the PHD filter
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

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

using TimedState    = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedMapModel = System.Collections.Generic.List<System.Tuple<double, monorfs.Map>>;
using SparseMatrix  = monorfs.SparseMatrix<double>;
using SparseItem    = monorfs.SparseItem<double>;

namespace monorfs
{
/// <summary>
/// SLAM solver. It uses the PHD filter.
/// </summary>
public class PHDNavigator<MeasurerT, PoseT, MeasurementT> : Navigator<MeasurerT, PoseT, MeasurementT>
	where PoseT        : IPose<PoseT>, new()
	where MeasurementT : IMeasurement<MeasurementT>, new()
	where MeasurerT    : IMeasurer<MeasurerT, PoseT, MeasurementT>, new()
{
	/// <summary>
	/// Landmark density expected on unexplored areas.
	/// </summary>
	public static double BirthWeight { get { return Config.BirthWeight; } }

	/// <summary>
	/// Landmark density spread expected on unexplored areas
	/// </summary>
	public static double[][] BirthCovariance { get { return Config.BirthCovariance; } }

	/// <summary>
	/// Minimum weight that is kept after a model prune.
	/// </summary>
	public static double MinWeight { get { return Config.MinWeight; } }

	/// <summary>
	/// Minimum particles effective fraction that need to exist to give reasonable exploration.
	/// </summary>
	public static double MinEffectiveParticle { get { return Config.MinEffectiveParticle; } }

	/// <summary>
	/// Maximum number of gaussians kept after a model prune.
	/// </summary>
	public static int MaxQuantity { get { return Config.MaxQuantity; } }

	/// <summary>
	/// Threshold used to decide when to merge close gaussian components.
	/// </summary>
	public static double MergeThreshold { get { return Config.MergeThreshold; } }

	/// <summary>
	/// Threshold use to decide if a new measurement is in unexplored territory.
	/// If it is, it should be explored (birth of a new gaussian).
	/// </summary>
	public static double ExplorationThreshold { get { return Config.ExplorationThreshold; } }

	/// <summary>
	/// Overestimation of the real motion covariance by the navigator.
	/// </summary>
	public static double MotionCovarianceMultiplier { get { return Config.MotionCovarianceMultiplier; } }

	/// <summary>
	/// Overestimation of the real measurement covariance by the navigator.
	/// </summary>
	public static double MeasurementCovarianceMultiplier { get { return Config.MeasurementCovarianceMultiplier; } }

	/// <summary>
	/// Assumed probability of detection.
	/// </summary>
	public static double PD { get { return Config.NavigatorPD; } }

	/// <summary>
	/// Assumed clutter density.
	/// </summary>
	public static double ClutterDensity { get { return Config.NavigatorClutterDensity; } }

	/// <summary>
	/// If true every particle history is rendered onscreen;
	/// otherwise, only the best particle is.
	/// </summary>
	public static bool RenderAllParticles { get { return Config.RenderAllParticles; } }

	/// <summary>
	/// Number of particles for the Montecarlo filter.
	/// </summary>
	public int ParticleCount { get; set; }

	/// <summary>
	/// Particle filter representation of the vehicle pose.
	/// </summary>
	public TrackVehicle<MeasurerT, PoseT, MeasurementT>[] VehicleParticles { get; private set; }

	/// <summary>
	/// Weight associated to each vehicle particle.
	/// </summary>
	public double[] VehicleWeights { get; private set; }

	/// <summary>
	/// PHD representation as a mixture-of-gaussians.
	/// One per vehicle particle.
	/// </summary>
	public Map[] MapModels { get; private set; }

	/// <summary>
	/// Index of the particle with the biggest weight.
	/// </summary>
	public int BestParticle { get; private set; }

	/// <summary>
	/// Most accurate estimate of the current vehicle pose.
	/// </summary>
	public override TrackVehicle<MeasurerT, PoseT, MeasurementT> BestEstimate
	{
		get
		{
			return VehicleParticles[BestParticle];
		}
	}

	/// <summary>
	/// Most accurate estimate model of the map.
	/// </summary>
	public override Map BestMapModel
	{
		get
		{
			return MapModels[BestParticle];
		}
	}

	/// <summary>
	/// Previous frame unexplored particles (delayed birth avoids spurious measurement super-certainty).
	/// </summary>
	private List<double[]>[] toexplore;

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

			for (int i = 0; i < VehicleParticles.Length; i++) {
				VehicleParticles[i].Graphics = value;
			}
		}
	}

	/// <summary>
	/// Construct a PHDNavigator using the indicated vehicle as a reference.
	/// </summary>
	/// <param name="vehicle">Vehicle to track.</param>
	/// <param name="particlecount">Number of particles for the Montecarlo filter.</param>
	/// <param name="onlymapping">If true, don't do SLAM, but mapping (i.e. the localization is assumed known).</param>
	public PHDNavigator(Vehicle<MeasurerT, PoseT, MeasurementT> vehicle, int particlecount, bool onlymapping = false)
		: base(vehicle, onlymapping)
	{
		ParticleCount = particlecount;

		// note that if mapping the __immediate__ particle count is one
		// but the argument is saved as is since the user could start
		// "mapping mode" and then the original number becomes relevant
		// (it may be changed anytime thorugh the ParticleCount property)
		if (onlymapping) {
			particlecount = 1;
		}

		// note that reset does deep cloning so no info flows in slam mode;
		// in mapping mode, info flows only through RefVehicle
		reset(RefVehicle, new Map(3), new List<double[]>(), particlecount);
	}

	/// <summary>
	/// Change the mode of the navigator to solving full slam.
	/// The unique particle currently in use will be multiplied into new localization particles.
	/// </summary>
	protected override void StartSlamInternal()
	{
		CollapseParticles(ParticleCount);
	}

	/// <summary>
	/// Change the mode of the navigator to do only mapping.
	/// The vehicle pose resets to the correct location, but the map is inherited
	/// from the previous best particle (maximum a posteriori).
	/// </summary>
	protected override void StartMappingInternal()
	{
		CollapseParticles(1);
	}

	/// <summary>
	/// Collapse the vehicle particles to the best previous particle.
	/// </summary>
	/// <param name="particlecount">Number of particles for the Montecarlo filter.</param>
	public void CollapseParticles(int particlecount)
	{
		reset(RefVehicle, BestMapModel, toexplore[BestParticle], particlecount);
	}

	/// <summary>
	/// Reset the state of the navigator to set of equal particles.
	/// </summary>
	/// <param name="vehicle">New vehicle pose.</param>
	/// <param name="model">New map model estimate.</param>
	/// <param name="explore">New queued measurements to explore.</param>
	/// <param name="particlecount">Number of particles for the Montecarlo filter.</param>
	private void reset(Vehicle<MeasurerT, PoseT, MeasurementT> vehicle, Map model, List<double[]> explore, int particlecount)
	{
		// do a deep copy, so no real info flows
		// into the navigator, only estimates, in SLAM
		// (this avoid bugs where the user unknowingly uses
		// the groundtruth to estimate the groundtruth itself)
		VehicleParticles = new TrackVehicle<MeasurerT, PoseT, MeasurementT>[particlecount];
		MapModels        = new Map           [particlecount];
		VehicleWeights   = new double        [particlecount];
		toexplore        = new List<double[]>[particlecount];

		for (int i = 0; i < particlecount; i++) {
			VehicleParticles[i] = vehicle.TrackClone(MotionCovarianceMultiplier,
			                                         MeasurementCovarianceMultiplier,
			                                         PD, ClutterDensity, true);
			MapModels       [i] = new Map(model);
			VehicleWeights  [i] = 1.0 / particlecount;
			toexplore       [i] = new List<double[]>(explore);
		}

		BestParticle = 0;
	}

	/// <summary>
	/// Reset the model of every particle to an empty map.
	/// </summary>
	public override void ResetMapModel()
	{
		for (int i = 0; i < MapModels.Length; i++) {
			MapModels[i].Clear();
		}
	}

	/// <summary>
	/// Remove all the localization and mapping history and start it again from the current position.
	/// </summary>
	public override void ResetHistory()
	{
		foreach (var particle in VehicleParticles) {
			particle.ResetHistory();
		}

		base.ResetHistory();
	}

	/// <summary>
	/// Update the vehicle particles.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="reading">Odometry reading (dx, dy, dz, dpitch, dyaw, droll).</param>
	public override void Update(GameTime time, double[] reading)
	{
		if (OnlyMapping) {
			VehicleParticles[0].Pose = RefVehicle.Pose.DClone();
			VehicleParticles[0].WayPoints = new TimedState(RefVehicle.WayPoints);
		}
		else {
			for (int i = 0; i < VehicleParticles.Length; i++) {
				VehicleParticles[i].UpdateNoisy(time, reading);
			}

			// debug: force the correct particle into the mix (it should always win)
			// or use a really close match (it too should win)
			//VehicleParticles[0].State = RefVehicle.State;
			//VehicleParticles[0].WayPoints = new TimedState(RefVehicle.WayPoints);
			//VehicleParticles[0].X += 0.0001;
		}

		UpdateTrajectory(time);
	}

	/// <summary>
	/// Update both the estimated map and the localization.
	/// This means doing a model prediction and a measurement update.
	/// This method is the core of the whole program.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="measurements">Sensor measurements in pixel-range form.</param>
	public override void SlamUpdate(GameTime time, List<MeasurementT> measurements)
	{
		// map update
		Parallel.For(0, VehicleParticles.Length,
		             new ParallelOptions { MaxDegreeOfParallelism = Config.NParallel}, i => {
			Map predicted, corrected;

			predicted = PredictConditional(measurements, VehicleParticles[i], MapModels[i], toexplore[i]);
			corrected = CorrectConditional(measurements, VehicleParticles[i], predicted);
			corrected = PruneModel(corrected);

			if (!OnlyMapping) {
				VehicleWeights[i] *= WeightAlpha(measurements, predicted, corrected, VehicleParticles[i]);
			}

			MapModels[i] = corrected;
		});

		// localization update
		if (!OnlyMapping) {
			double sum     = VehicleWeights.Sum();
			sum            = (sum == 0) ? 1 : sum;
			VehicleWeights = VehicleWeights.Divide(sum);

			double maxweight = 0;

			for (int i = 0; i < VehicleWeights.Length; i++) {
				if (VehicleWeights[i] > maxweight) {
					maxweight    = VehicleWeights[i];
					BestParticle = i;
				}
			}

			if (ParticleDepleted()) {
				ResampleParticles();
			}
		}

		UpdateMapHistory(time);
	}

	/// <summary>
	/// Update the weights of each vehicle using the sequential importance sampling technique.
	/// It uses most-probable-components approximation to solve the problem in polynomial time.
	/// </summary>
	/// <param name="measurements">Sensor measurements in pixel-range form.</param>
	/// <param name="predicted">Predicted map model.</param>
	/// <param name="corrected">Corrected map model.</param>
	/// <param name="pose">Vehicle pose.</param>
	/// <returns>Total weight update factor.</returns>
	public double WeightAlpha(List<MeasurementT> measurements, Map predicted, Map corrected,
	                          SimulatedVehicle<MeasurerT, PoseT, MeasurementT> pose)
	{
		IMap cvisible = corrected.FindAll(g => pose.Visible(g.Mean) && g.Weight > 0.8);

		double plikelihood = 1;
		double clikelihood = 1;

		foreach (var component in cvisible) {
			plikelihood *= predicted.Evaluate(component.Mean);
			clikelihood *= corrected.Evaluate(component.Mean);
		}

		double pcount = predicted.ExpectedSize;
		double ccount = corrected.ExpectedSize;

		double setlikelihood = SetLikelihood(measurements, cvisible, pose);

		double likelihoodratio = plikelihood / clikelihood;

		if (pcount > 0 && ccount > 0) {
			likelihoodratio *= ccount / pcount;
		}

		return setlikelihood * likelihoodratio;
	}

	/// <summary>
	/// Calculate the set likelihood P(Z|X, M).
	/// </summary>
	/// <param name="measurements">Sensor measurements in pixel-range form.</param>
	/// <param name="map">Map model.</param>
	/// <param name="pose">Vehicle pose.</param>
	/// <returns>Set likelihood.</returns>
	public static double SetLikelihood(List<MeasurementT> measurements, IMap map,
	                                   SimulatedVehicle<MeasurerT, PoseT, MeasurementT> pose)
	{
		return Math.Exp(SetLogLikelihood(measurements, map, pose));
	}

	/// <summary>
	/// Generate a log-likelihood matrix relating each measurement with each landmark.
	/// </summary>
	/// <param name="measurements">Sensor measurements in pixel-range form.</param>
	/// <param name="map">Map model.</param>
	/// <param name="pose">Vehicle pose.</param>
	/// <returns>Set log-likelihood matrix.</returns>
	public static SparseMatrix SetLogLikeMatrix(List<MeasurementT> measurements, IMap map,
	                                            SimulatedVehicle<MeasurerT, PoseT, MeasurementT> pose)
	{
		// exact calculation if there are few components/measurements;
		// use the most probable components approximation otherwise
		SparseMatrix logprobs = new SparseMatrix(map.Count + measurements.Count, map.Count + measurements.Count, double.NegativeInfinity);

		double     logclutter = Math.Log(pose.ClutterDensity);
		Gaussian[] zprobs     = new Gaussian[map.Count];

		int n = 0;
		foreach (Gaussian landmark in map) {
			MeasurementT m       = pose.Measurer.MeasurePerfect(pose.Pose, landmark.Mean);
			double[]     mlinear = m.ToLinear();
			zprobs[n] = new Gaussian(mlinear, pose.MeasurementCovariance, pose.DetectionProbabilityM(m));
			n++;
		}

		for (int i = 0; i < zprobs.Length;      i++) {
		for (int k = 0; k < measurements.Count; k++) {
			double d = zprobs[i].Mahalanobis(measurements[k].ToLinear());
			if (d < 5) {
				// prob = log (pD * zprob(measurement))
				// this way multiplying probabilities equals to adding (negative) profits
				logprobs[i, k] = Math.Log(zprobs[i].Weight) + Math.Log(zprobs[i].Multiplier) - 0.5 * d * d;
			}
		}
		}
		
		for (int i = 0; i < map.Count; i++) {
			logprobs[i, measurements.Count + i] = Math.Log(1 - zprobs[i].Weight);
		}

		for (int i = 0; i < measurements.Count; i++) {
			logprobs[map.Count + i, i] = logclutter;
		}

		return logprobs;
	}

	/// <summary>
	/// Calculate the set log-likelihood log(P(Z|X, M)).
	/// </summary>
	/// <param name="measurements">Sensor measurements in pixel-range form.</param>
	/// <param name="map">Map model.</param>
	/// <param name="pose">Vehicle pose.</param>
	/// <returns>Set log-likelihood.</returns>
	public static double SetLogLikelihood(List<MeasurementT> measurements, IMap map,
	                                      SimulatedVehicle<MeasurerT, PoseT, MeasurementT> pose)
	{
		// exact calculation if there are few components/measurements;
		// use the most probable components approximation otherwise
		SparseMatrix       llmatrix      = SetLogLikeMatrix(measurements, map, pose);
		List<SparseMatrix> connectedfull = GraphCombinatorics.ConnectedComponents(llmatrix);
		double[]           logcomp       = new double[200];
		double             total         = 0;

		for (int i = 0; i < connectedfull.Count; i++) {
			int[]        rows;
			int[]        cols;
			SparseMatrix component = connectedfull[i].Compact(out rows, out cols);

			// fill the (Misdetection x Clutter) quadrant of the matrix with zeros (don't contribute)
			// NOTE this is filled after the connected components partition because
			//      otherwise everything would be connected through this quadrant
			for (int k = 0; k < rows.Length; k++) {
				if (rows[k] >= map.Count) {
					for (int h = 0; h < cols.Length; h++) {
						if (cols[h] >= measurements.Count) {
							component[k, h] = 0;
						}
					}
				}
			}

			IEnumerable<Tuple<int[], double>> assignments;
			bool enumerateall = false;
			if (component.Rows.Count <= 5) {
				assignments  = GraphCombinatorics.LexicographicalPairing(component, map.Count);
				enumerateall = true;
			}
			else {
				assignments  = GraphCombinatorics.MurtyPairing(component);
				enumerateall = false;
			}

			int m = 0;
			foreach (Tuple<int[], double> assignment in assignments) {
				if (m >= logcomp.Length || (!enumerateall && logcomp[m] - logcomp[0] < -10)) {
					break;
				}

				logcomp[m] = assignment.Item2;
				m++;
			}

			total += logcomp.LogSumExp(0, m);
		}

		return total;
	}

	/// <summary>
	/// Calculate the set log-likelihood log(P(Z|X, M)), but do not consider visibility
	/// (everything is fully visible). This is used to avoid uninteresting solution to the
	/// optimization problem max_X log(P(Z|X, M)). Also gives the pose gradient at the evaluated pose.
	/// </summary>
	/// <param name="measurements">Sensor measurements in pixel-range form.</param>
	/// <param name="map">Map model.</param>
	/// <param name="pose">Vehicle pose.</param>
	/// <param name="gradient">Log-likelihood gradient wrt. the pose.</param>
	/// <returns>Set log-likelihood.</returns>
	public static double QuasiSetLogLikelihood(List<MeasurementT> measurements, IMap map,
	                                           SimulatedVehicle<MeasurerT, PoseT, MeasurementT> pose,
	                                           out double[] gradient)
	{
		// exact calculation if there are few components/measurements;
		// use the most probable components approximation otherwise
		SparseMatrix llmatrix = new SparseMatrix(map.Count + measurements.Count, map.Count + measurements.Count, double.NegativeInfinity);
		var          dlldp    = new SparseMatrix<double[]>(map.Count + measurements.Count, map.Count + measurements.Count, new double[OdoSize]);

		double       logPD      = Math.Log(pose.PD);
		double       log1PD     = Math.Log(1 - pose.PD);
		double       logclutter = Math.Log(pose.ClutterDensity);
		Gaussian[]   zprobs     = new Gaussian[map.Count];
		double[][][] zjacobians = new double[map.Count][][];

		int n = 0;
		foreach (Gaussian landmark in map) {
			MeasurementT m       = pose.Measurer.MeasurePerfect(pose.Pose, landmark.Mean);
			double[]     mlinear = m.ToLinear();
			zprobs[n]     = new Gaussian(mlinear, pose.MeasurementCovariance, 1);
			zjacobians[n] = pose.Measurer.MeasurementJacobianP(pose.Pose, landmark.Mean);
			n++;
		}
		
		for (int i = 0; i < zprobs.Length;      i++) {
		for (int k = 0; k < measurements.Count; k++) {
			double[] m = measurements[k].ToLinear();
			double   d = zprobs[i].Mahalanobis(m);
			if (d < 5) {
				// prob = log (pD * zprob(measurement))
				// this way multiplying probabilities equals to adding (negative) profits
				llmatrix[i, k] = logPD + Math.Log(zprobs[i].Multiplier) - 0.5 * d * d;
				dlldp   [i, k] = (m.Subtract(zprobs[i].Mean)).Transpose().
				                     Multiply(zprobs[i].CovarianceInverse).
				                     Multiply(zjacobians[i]).
				                     GetRow(0);
			}
		}
		}
		
		for (int i = 0; i < map.Count; i++) {
			llmatrix[i, measurements.Count + i] = log1PD;
		}
		
		for (int i = 0; i < measurements.Count; i++) {
			llmatrix[map.Count + i, i] = logclutter;
		}
		
		List<SparseMatrix> connectedfull = GraphCombinatorics.ConnectedComponents(llmatrix);
		
		double[]   logcomp    = new double[200];
		double[][] dlogcompdp = new double[200][];
		double     total      = 0;
		
		gradient = new double[OdoSize];
		
		for (int i = 0; i < connectedfull.Count; i++) {
			int[]        rows;
			int[]        cols;
			SparseMatrix component = connectedfull[i].Compact(out rows, out cols);
			var          dcomp     = dlldp.Submatrix(rows, cols);
			
			// fill the (Misdetection x Clutter) quadrant of the matrix with zeros (don't contribute)
			// NOTE this is filled after the connected components partition because
			//      otherwise everything would be connected through this quadrant
			for (int k = 0; k < rows.Length; k++) {
				if (rows[k] >= map.Count) {
					for (int h = 0; h < cols.Length; h++) {
						if (cols[h] >= measurements.Count) {
							component[k, h] = 0;
						}
					}
				}
			}
			
			IEnumerable<Tuple<int[], double>> assignments;
			bool enumerateall = false;
			if (component.Rows.Count <= 5) {
				assignments = GraphCombinatorics.LexicographicalPairing(component, map.Count);
				enumerateall = true;
			}
			else {
				assignments  = GraphCombinatorics.MurtyPairing(component);
				enumerateall = false;
			}

			int m = 0;
			foreach (Tuple<int[], double> assignment in assignments) {
				if (m >= logcomp.Length || (!enumerateall && logcomp[m] - logcomp[0] < -10)) {
					break;
				}

				logcomp   [m] = assignment.Item2;
				dlogcompdp[m] = new double[OdoSize];

				for (int p = 0; p < assignment.Item1.Length; p++) {
					dlogcompdp[m] = dlogcompdp[m].Add(dcomp[p, assignment.Item1[p]]);
				}

				m++;
			}

			total   += logcomp.LogSumExp(0, m);
			gradient = gradient.Add(dlogcompdp.TemperedAverage(logcomp, 0, m));

		}

		return total;
	}

	/// <summary>
	/// Resample the vehicle particles with their corresponding weights and map models
	/// using systematic resampling (aka Wheel resampling).
	/// The weights are transferred into the unit range (0, 1] and random number is
	/// generated in the range (0, 1/n], adding 1/n to iterate through the unit range
	/// obtaining appropiate samples.
	/// This method is fast and simple although it loses the important property of
	/// giving independent samples, but in practice works very well.
	/// </summary>
	public void ResampleParticles()
	{
		var      particles = new TrackVehicle<MeasurerT, PoseT, MeasurementT>[VehicleParticles.Length];
		double   random    = (double) Util.Uniform.Next() / VehicleWeights.Length;
		double[] weights   = new double[VehicleWeights.Length];
		Map[]    models    = new Map[MapModels.Length];
		double   maxweight = 0;

		for (int i = 0, k = 0; i < weights.Length; i++) {
			// k should never be out of range, but because of floating point arithmetic,
			// the probabilities may not add up to one. If the random number hits this tiny difference
			// it will throw an exception. In such rare case, just expand the last guy's probability
			for (; random > 0 && k < VehicleWeights.Length; k++) {
				random -= VehicleWeights[k];
			}
			
			particles[i] = RefVehicle.TrackClone(VehicleParticles[k - 1], true);
			models   [i] = new Map(MapModels[k - 1]);
			weights  [i] = 1.0 / weights.Length;
			random      += 1.0 / weights.Length;

			if (VehicleWeights[k - 1] > maxweight) {
				maxweight    = VehicleWeights[k - 1];
				BestParticle = i;
			}
		}

		// debug: force the first particle to remain there
		// use with "make the first particle the real particle" on Update()
		// to also carry its map model
		//particles[0] = VehicleParticles[0];
		//models[0]    = MapModels[0];

		VehicleParticles = particles;
		VehicleWeights   = weights;
		MapModels        = models;
	}

	/// <summary>
	/// Check the depletion state of the particles.
	/// </summary>
	/// <returns>
	/// True if there are too few "effective particles",
	/// i.e. a lot of particle have negligible weights.</returns>
	public bool ParticleDepleted()
	{
		double cum = 0;

		foreach (double weight in VehicleWeights) {
			cum += weight * weight;
		}

		return 1.0 / cum < MinEffectiveParticle * VehicleWeights.Length;
	}

	/// <summary>
	/// Predict the state of the map in the next iteration
	/// given the trajectory of the vehicle.
	/// </summary>
	/// <param name="measurements">Sensor measurements in range-bearing  form.</param>
	/// <param name="pose">Vehicle pose that conditions the mapping.</param>
	/// <param name="model">Associated map model.</param>
	/// <param name="unexplored">Input/Output, unexplored candidates from previous timestep..</param> 
	/// <returns>Predicted map model.</returns>
	/// <remarks>Though in theory the predict doesn't depend on the measurements,
	/// the birth region (i.e. the unexplored areas) is large to represent on gaussians.
	/// To diminish the computational time, only the areas near the new measurements are
	/// used (i.e. the same measurements). This is equivalent to artificially increasing
	/// belief on the new measurements (when in a new area).</remarks>
	public Map PredictConditional(List<MeasurementT> measurements,
	                              SimulatedVehicle<MeasurerT, PoseT, MeasurementT> pose,
	                              Map model, List<double[]> unexplored)
	{
		// gaussian are born on any unexplored areas,
		// as something is expected to be there,
		// but this is too resource-intensive so
		// here's a little cheat: only do it on areas
		// there has been a new measurement lately
		Map predicted = new Map(model);
		
		// birth RFS
		foreach (double[] candidate in unexplored) {
			predicted.Add(new Gaussian(candidate, BirthCovariance, BirthWeight));
		}

		unexplored.Clear();

		foreach (MeasurementT measurement in measurements) {
			double[] candidate = pose.Measurer.MeasureToMap(pose.Pose, measurement);
			if (!Explored(model, candidate)) {
				unexplored.Add(candidate);
			}
		}

		return predicted;
	}

	/// <summary>
	/// Correct the estimated map given the localization
	/// using a set of measurements.
	/// </summary>
	/// <param name="measurements">Sensor measurements in range-bearing  form.</param>
	/// <param name="pose">Vehicle pose that conditions the mapping.</param>
	/// <param name="model">Associated map model.</param>
	/// <returns>Corrected map model.</returns>
	public Map CorrectConditional(List<MeasurementT> measurements,
	                              SimulatedVehicle<MeasurerT, PoseT, MeasurementT> pose,
	                              Map model)
	{
		Map corrected = new Map(3);

		// reduce predicted weight to acocunt for misdetection
		// v += (1-PD) v
		foreach (Gaussian component in model) {
			corrected.Add(new Gaussian(component.Mean,
			                           component.Covariance,
			                           (1 - pose.DetectionProbability(component.Mean)) * component.Weight));
		}

		// measurement PHD update
		double[][]   R  = pose.MeasurementCovariance;
		double[][]   I  = Accord.Math.Matrix.Identity(3).ToArray();
		double[][]   m  = new double  [model.Count][];
		double[][]   mp = new double  [model.Count][];
		double[][][] H  = new double  [model.Count][][];
		double[][][] P  = new double  [model.Count][][];
		double[][][] PH = new double  [model.Count][][];
		double[][][] S  = new double  [model.Count][][];
		Gaussian[]   mc = new Gaussian[model.Count];
		Map          q  = new Map(MeasureSize);

		var qindex = new Dictionary<Gaussian, int>();

		int n = 0;
		foreach (Gaussian component in model) {
			m [n] = component.Mean;
			mp[n] = pose.Measurer.MeasurePerfect(pose.Pose, m[n]).ToLinear();
			H [n] = pose.Measurer.MeasurementJacobianL(pose.Pose, m[n]);
			P [n] = component.Covariance;
			PH[n] = P[n].MultiplyByTranspose(H[n]);
			S [n] = H[n].Multiply(PH[n]).Add(R);
			mc[n] = new Gaussian(mp[n], S[n], component.Weight);

			qindex.Add(component, n);
			q.Add(mc[n]);

			n++;
		}

		// for each measurement z and a priori map component,
		// v += w' N(x, m', P'), where
		// w'   = PD w q(z) / (k(z) + sum over components(w q(z)))
		// m'   = m + K (z - h(m))
		// P'   = [I - K H] P
		// K    = P H^T (H P H^T + R)^-1
		// q(z) = N(z, h(m), H P H^T + R)
		// R is the measurement covariance
		// H is the measurement linear operator (jacobian) from the model
		foreach (MeasurementT measurement in measurements) {
			double PD = pose.DetectionProbabilityM(measurement);

			Map      near      = model.Near(pose.Measurer.MeasureToMap(pose.Pose, measurement), 1.2);
			double[] mlinear   = measurement.ToLinear();
			double   weightsum = q.Evaluate(mlinear);

			foreach (var landmark in near) {
				int i = qindex[landmark];

				// double[][] gain       = PH[i].Multiply(S[i].Inverse());
				double[][] gain       = PH[i].Multiply(mc[i].CovarianceInverse);
				double[]   mean       = m[i].Add(gain.Multiply(mlinear.Subtract(mp[i])));
				double[][] covariance = I.Subtract(gain.Multiply(H[i])).Multiply(P[i]);

				double weight = PD * landmark.Weight * mc[i].Evaluate(measurement.ToLinear()) / (pose.ClutterDensity + PD * weightsum);

				corrected.Add(new Gaussian(mean, covariance, weight));
			}
		}

		return corrected;
	}

	/// <summary>
	/// Remove irrelevant gaussians from the map model.
	/// </summary>
	/// <param name="model">Map model.</param>
	/// <returns>Pruned model.</returns>
	public Map PruneModel(Map model)
	{
		Map      pruned = new Map(3);
		Gaussian candidate;
		Map      close;

		List<Gaussian> landmarks = model.ToList();
		landmarks.Sort((a, b) => Math.Sign(b.Weight - a.Weight));

		int weightcut = 0;

		for (weightcut = 0; weightcut < Math.Min(MaxQuantity, landmarks.Count); weightcut++) {
			if (landmarks[weightcut].Weight < MinWeight) {
				break;
			}
		}

		for (int i = 0; i < weightcut; i++) {
			candidate = landmarks[i];
			close     = new Map(3);

			for (int k = i + 1; k < weightcut; k++) {
				if (Gaussian.AreClose(landmarks[i], landmarks[k], MergeThreshold)) {
					close.Add(landmarks[k]);
					landmarks.RemoveAt(k--);
					weightcut--;
				}
			}

			if (close.Count > 0) {
				candidate = Merge(candidate, close);
			}

			pruned.Add(candidate);
		}

		return pruned;
	}

	/// <summary>
	/// Merge a list of gaussian components into a one big gaussian
	/// that tries to approximate as much as possible the behaviour
	/// of the original mixture.
	/// </summary>
	/// <param name="maincomponent">First gaussian component.</param>
	/// <param name="components">List of the other gaussian components.</param>
	/// <returns>Merged gaussian.</returns>
	private static Gaussian Merge(Gaussian maincomponent, Map components)
	{
		double     weight     = maincomponent.Weight;
		double[]   mean       = maincomponent.Weight.Multiply(maincomponent.Mean);
		double[][] covariance = maincomponent.Weight.Multiply(maincomponent.Covariance);

		// merged gaussian follows the rules
		// w = sum of (wi)
		// m = sum of (wi mi) / w
		// P = sum of (wi (Pi + (mi - m0) (mi - m0)^T)) / w
		foreach (Gaussian component in components) {
			double[] diff = component.Mean.Subtract(maincomponent.Mean);
			weight    += component.Weight;
			mean       = mean      .Add(component.Weight.Multiply(component.Mean));
			covariance = covariance.Add(component.Weight.Multiply(component.Covariance.Add(diff.OuterProduct(diff).ToArray())));
		}

		mean       = mean.Divide(weight);
		covariance = covariance.Divide(weight);

		return new Gaussian(mean, covariance, weight);
	}

	/// <summary>
	/// Queries if a location has been explored on a given map model.
	/// </summary>
	/// <param name="model">Map model.</param>
	/// <param name="x">Location.</param>
	/// <returns>True if the location has been explored.</returns>
	public bool Explored(Map model, double[] x)
	{
		return model.Evaluate(x) >= ExplorationThreshold;
	}

	/// <summary>
	/// Render the navigation HUD and the trajectory on the graphics device.
	/// The graphics device must be ready, otherwise
	/// the method will throw an exception.
	/// </summary>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public override void Render(double[][] camera)
	{
		RefVehicle.Render(camera);

		if (RenderAllParticles) {
			foreach (var particle in VehicleParticles) {
				particle.RenderTrajectory(camera, Color.Blue);
			}
		}
		else {
			RenderTrajectory(camera);
		}

		RenderMap(camera);
	}
}
}
