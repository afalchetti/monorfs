// Navigator.cs
// SLAM solving navigator
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

namespace monorfs
{
/// <summary>
/// SLAM solver. It uses the PHD Filter.
/// </summary>
public class Navigator
{
	/// <summary>
	/// Landmark density expected on unexplored areas.
	/// </summary>
	public const double BirthWeight = 0.05;

	/// <summary>
	/// Landmark density spread expected on unexplored areas
	/// </summary>
	public static readonly double[][] BirthCovariance = new double[3][] {new double[3] {1e-3, 0, 0},
	                                                                     new double[3] {0, 1e-3, 0},
	                                                                     new double[3] {0, 0, 1e-3}};

	/// <summary>
	/// Minimum weight that is kept after a model prune.
	/// </summary>
	public const double MinWeight = 1e-2;

	/// <summary>
	/// Minimum particles effective fraction that need to exist to give reasonable exploration.
	/// </summary>
	public const double MinEffectiveParticle = 0.3;

	/// <summary>
	/// Maximum number of gaussians kept after a model prune.
	/// </summary>
	public const int MaxQuantity = 100;

	/// <summary>
	/// Threshold used to decide when to merge close gaussian components.
	/// </summary>
	public const double MergeThreshold = 3e0;

	/// <summary>
	/// Threshold use to decide if a new measurement is in unexplored territory.
	/// If it is, it should be explored (birth of a new gaussian).
	/// </summary>
	public const double ExplorationThreshold = 1e-5;
	//public const double ExplorationThreshold = 3;

	/// <summary>
	/// Reference vehicle. Only used if mapping mode is enabled (no SLAM).
	/// </summary>
	public Vehicle RefVehicle { get; private set; }

	/// <summary>
	/// Particle filter representation of the vehicle pose.
	/// </summary>
	public SimulatedVehicle[] VehicleParticles { get; private set; }

	/// <summary>
	/// Weight associated to each vehicle particle.
	/// </summary>
	public double[] VehicleWeights { get; private set; }

	/// <summary>
	/// PHD representation as a mixture-of-gaussians.
	/// One per vehicle particle.
	/// </summary>
	public List<Gaussian>[] MapModels { get; private set; }

	/// <summary>
	/// previous frame unexplored particles (delayed birth avoids spurious measurement super-certainty)
	/// </summary>
	private List<double[]>[] toexplore;

	/// <summary>
	/// Expected number of landmarks after the prediction step.
	/// </summary>
	private double[] mpredicted;

	/// <summary>
	/// Expected number of landmarks after the correction step.
	/// </summary>
	private double[] mcorrected;

	/// <summary>
	/// Index of the particle with the biggest weight.
	/// </summary>
	public int BestParticle { get; private set; }

	/// <summary>
	/// True if the localization of the vehicle is known.
	/// </summary>
	public bool OnlyMapping { get; private set; }

	/// <summary>
	/// Prediction interval base model.
	/// </summary>
	private double[][] pinterval;

	/// <summary>
	/// The estimated trajectory.
	/// </summary>
	public List<double[]> Waypoints { get; set; }

	/// <summary>
	/// If true every particle history is rendered onscreen;
	/// otherwise, only the best particle is.
	/// </summary>
	public const bool RenderAllParticles = true;

	/// <summary>
	/// Internel render output.
	/// </summary>
	private GraphicsDevice graphics;

	/// <summary>
	/// Render output.
	/// </summary>
	public GraphicsDevice Graphics {
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
	/// Best map history.
	/// </summary>
	public List<Tuple<double, List<Gaussian>>> WayMaps { get; private set; }

	/// <summary>
	/// Construct a Navigator using the indicated vehicle as a reference.
	/// </summary>
	/// <param name="vehicle">Vehicle to track.</param>
	/// <param name="particlecount">Number of particles for the Montecarlo filter.</param>
	/// <param name="onlymapping">If true, don't do SLAM, but mapping (i.e. the localization is assumed known).</param>
	public Navigator(Vehicle vehicle, int particlecount, bool onlymapping = false)
	{
		OnlyMapping = onlymapping;

		// a reference copy, which gives the navigator precise info about
		// the real vehicle pose
		RefVehicle = vehicle;

		if (onlymapping) {
			particlecount = 1;
		}

		// do a deep copy, so no real info flows
		// into the navigator, only estimates, in SLAM.
		// In mapping, info flows only through RefVehicle
		VehicleParticles = new SimulatedVehicle[particlecount];
		MapModels        = new List<Gaussian>  [particlecount];
		VehicleWeights   = new double          [particlecount];
		toexplore        = new List<double[]>  [particlecount];

		for (int i = 0; i < particlecount; i++) {
			VehicleParticles[i] = new SimulatedVehicle(vehicle, 1.8, 1.8, 0.9, 5e-7);
			MapModels       [i] = new List<Gaussian>();
			VehicleWeights  [i] = 1.0 / particlecount;
			toexplore       [i] = new List<double[]>();
		}

		mpredicted   = new double[particlecount];
		mcorrected   = new double[particlecount];
		BestParticle = 0;

		Waypoints = new List<double[]>();
		Waypoints.Add(new double[4] {0, vehicle.X, vehicle.Y, vehicle.Z});

		WayMaps = new List<Tuple<double, List<Gaussian>>>();
		WayMaps.Add(new Tuple<double, List<Gaussian>>(0, new List<Gaussian>()));

		const int segments = 32;
		pinterval = new double[segments][];

		// pinterval will be the 5-sigma ellipse
		for (int i = 0; i < segments; i++) {
			this.pinterval[i] = new double[2] {5 * Math.Cos(2 * Math.PI * i / segments), 5*Math.Sin(2 * Math.PI * i / segments)};
		}
	}

	/// <summary>
	/// Change the mode of the navigator to solving full slam.
	/// The unique particle currently in use will be multiplied into new localization particles.
	/// </summary>
	/// <param name="particlecount">Number of particles for the Montecarlo filter.</param>
	public void StartSlam(int particlecount)
	{
		if (OnlyMapping) {
			OnlyMapping = false;

			CollapseParticles(particlecount);
		}
	}

	/// <summary>
	/// Change the mode of the navigator to do only mapping.
	/// The vehicle pose resets to the correct location, but the map is inherited
	/// from the previous best particle (MAP).
	/// </summary>
	public void StartMapping()
	{
		if (!OnlyMapping) {
			OnlyMapping = true;

			CollapseParticles(1);
		}
	}

	/// <summary>
	/// Collapse the vehicle particles to the best previous particle.
	/// </summary>
	/// <param name="particlecount">Number of particles for the Montecarlo filter.</param>
	public void CollapseParticles(int particlecount)
	{
		Vehicle        prevvehicle = VehicleParticles[BestParticle];
		List<Gaussian> prevmodel   = MapModels       [BestParticle];
		List<double[]> prevexplore = toexplore       [BestParticle];

		VehicleParticles = new SimulatedVehicle[particlecount];
		MapModels        = new List<Gaussian>  [particlecount];
		VehicleWeights   = new double          [particlecount];
		toexplore        = new List<double[]>  [particlecount];

		for (int i = 0; i < particlecount; i++) {
			VehicleParticles[i] = new SimulatedVehicle(prevvehicle, 1.8, 1.8, 0.7, 5e-7);
			MapModels       [i] = new List<Gaussian>(prevmodel);
			VehicleWeights  [i] = 1.0 / particlecount;
			toexplore       [i] = new List<double[]>(prevexplore);
		}

		mpredicted   = new double[particlecount];
		mcorrected   = new double[particlecount];
		BestParticle = 0;
	}

	/// <summary>
	/// Reset the model of every particle to an empty map.
	/// </summary>
	public void ResetModels()
	{
		for (int i = 0; i < MapModels.Length; i++) {
			MapModels[i].Clear();
		}
	}

	/// <summary>
	/// Remove all the localization and mapping history and start it again from the current position.
	/// </summary>
	public void ResetHistory()
	{
		foreach (var particle in VehicleParticles) {
			particle.ResetHistory();
		}

		Waypoints.Clear();
		WayMaps  .Clear();
		
		Waypoints = new List<double[]>();
		Waypoints.Add(new double[4] {0, VehicleParticles[BestParticle].X, VehicleParticles[BestParticle].Y, VehicleParticles[BestParticle].Z});

		WayMaps = new List<Tuple<double, List<Gaussian>>>();
		WayMaps.Add(new Tuple<double, List<Gaussian>>(0, new List<Gaussian>()));
	}

	/// <summary>
	/// Update the vehicle particles.
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
		if (OnlyMapping) {
			VehicleParticles[0].State = RefVehicle.State;
		}
		else {
			for (int i = 0; i < VehicleParticles.Length; i++) {
				VehicleParticles[i].Update(time, dx, dy, dz, dyaw, dpitch, droll);
			}

			// debug: force the correct particle into the mix (it should always win)
			// or use a really close match (it too should win)
			//VehicleParticles[0].State = RefVehicle.State;
			//VehicleParticles[0].X += 0.0001;
		}

		Vehicle best = VehicleParticles[BestParticle];
		
		double[] prevloc = new double[3] {Waypoints[Waypoints.Count - 1][1],
		                                  Waypoints[Waypoints.Count - 1][2],
		                                  Waypoints[Waypoints.Count - 1][3]};
		
		 // FIXME this "too close to matter" efficiency option is disabled as a workaround to make
		// viewer work smoothly but should eventually be reinstated and a smarter interpolation should be done there
		//if (best.Location.Subtract(prevloc).SquareEuclidean() >= 1e-2f) {
			Waypoints.Add(new double[4] {time.TotalGameTime.TotalSeconds, best.X, best.Y, best.Z});
		//}
	}

	/// <summary>
	/// Update both the estimated map and the localization.
	/// This means doing a model prediction and a measurement update.
	/// This method is the core of the whole program.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="measurements">Sensor measurements in pixel-range form.</param>
	/// <param name="predict">Predict flag; if false, no prediction step is done.</param>
	/// <param name="correct">Correct flag; if false, no correction step is done.</param>
	/// <param name="prune">Prune flag; if false, no prune step is done.</param>
	public void SlamUpdate(GameTime time, List<double[]> measurements, bool predict, bool correct, bool prune)
	{
		// map update
		Parallel.For (0, VehicleParticles.Length, i => {
			List<Gaussian> predicted, corrected;

			if (predict) { predicted = PredictConditional(measurements, VehicleParticles[i], MapModels[i], toexplore[i]); }
			else         { predicted = MapModels[i]; }

			if (correct) { corrected = CorrectConditional(measurements, VehicleParticles[i], predicted); }
			else         { corrected = predicted; }

			if (prune)   { corrected = PruneModel(corrected); }

			if (!OnlyMapping) { VehicleWeights[i] *= WeightAlpha(measurements, predicted, corrected, VehicleParticles[i]); }

			MapModels[i] = corrected;
		});

		// localization update
		if (!OnlyMapping) {
			double sum = VehicleWeights.Sum();

			if (sum == 0) {
				sum = 1;
			}

			VehicleWeights = VehicleWeights.Divide(sum);

			if (ParticleDepleted()) {
				ResampleParticles();
			}
		}

		WayMaps.Add(new Tuple<double, List<Gaussian>>(time.TotalGameTime.TotalSeconds, MapModels[BestParticle]));
	}

	/// <summary>
	/// Update the weights of each vehicle using the sequential importance sampling technique.
	/// It uses most-probable-components approximation to solve the problem in polynomial time.
	/// </summary>
	/// <param name="measurements">Sensor measurements in pixel-range form.</param>
	/// <param name="predicted">Predicted map model.</param>
	/// <param name="corrected">Corrected map model.</param>
	/// <param name="pose">Vehicle pose.</param>
	/// <returns>Total likelihood weight.</returns>
	public double WeightAlpha(List<double[]> measurements, List<Gaussian> predicted, List<Gaussian> corrected, SimulatedVehicle pose)
	{
		List<Gaussian> visible = corrected.FindAll(g => pose.Visible(g.Mean) && g.Weight > 0.8);

		// exact calculation if there are few components/measurements;
		// use the most probable components approximation otherwise
		SparseMatrix logprobs = new SparseMatrix(visible.Count + measurements.Count, visible.Count + measurements.Count, double.NegativeInfinity);

		double     logPD      = (visible.Count > 0) ? Math.Log(pose.DetectionProbability(visible[0].Mean)) : 0;
		double     logclutter = Math.Log(pose.ClutterDensity);
		Gaussian[] zprobs     = new Gaussian[visible.Count];


		for (int i = 0; i < visible.Count; i++) {
			zprobs[i] = new Gaussian(pose.MeasurePerfect(visible[i].Mean), pose.MeasurementCovariance, 1); 
		}

		for (int i = 0; i < zprobs.Length;      i++) {
		for (int k = 0; k < measurements.Count; k++) {
			double d = Mahalanobis(zprobs[i], measurements[k]);
			if (d < 3) {
				// prob = log (pD * zprob(measurement))
				// this way multiplying probabilities equals to adding (negative) profits
				logprobs[i, k] = logPD + Math.Log(zprobs[i].Multiplier) - 0.5 * d * d;
			}
		}
		}
		
		for (int i = 0; i < visible.Count; i++) {
			logprobs[i, measurements.Count + i] = logPD;
		}

		for (int i = 0; i < measurements.Count; i++) {
			logprobs[visible.Count + i, i] = logclutter;
		}

		List<SparseMatrix> connectedfull = GraphCombinatorics.ConnectedComponents(logprobs);
		List<SparseMatrix> components    = new List<SparseMatrix>();

		for (int i = 0; i < connectedfull.Count; i++) {
			int[]        rows;
			int[]        cols;
			SparseMatrix component = connectedfull[i].Compact(out rows, out cols);

			// fill the (Misdetection x Clutter) quadrant of the matrix with zeros (don't contribute)
			for (int k = 0; k < rows.Length; k++) {
				if (rows[k] >= visible.Count) {
					for (int h = 0; h < cols.Length; h++) {
						if (cols[h] >= measurements.Count) {
							component[k, h] = 0;
						}
					}
				}
			}
			
			components.Add(component);
		}

		double total = 1.0;
		foreach (SparseMatrix component in components) {

			IEnumerable<int[]> assignments;
			if (component.Rows.Count <= 8) {
				assignments = GraphCombinatorics.LexicographicalPairing(component, visible.Count);

			}
			else {
				assignments = GraphCombinatorics.MurtyPairing(component);
			}

			int    h         = 0;
			double comptotal = 0;
			double prev      = 0;
			foreach (int[] assignment in assignments) {
				comptotal += Math.Exp(GraphCombinatorics.AssignmentValue(component, assignment));

				if (h >= 200 || comptotal / prev < 1.001) {
					break;
				}

				prev = comptotal;
				h++;
			}

			total *= comptotal;
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
		double             random    = (double) Util.uniform.Next() / VehicleWeights.Length;
		double[]           weights   = new double[VehicleWeights.Length];
		SimulatedVehicle[] particles = new SimulatedVehicle[VehicleParticles.Length];
		List<Gaussian>[]   models    = new List<Gaussian>[MapModels.Length];
		double             maxweight = 0;

		for (int i = 0, k = 0; i < weights.Length; i++) {
			// k should never be out of range, but because of floating point arithmetic,
			// the probabilities may not add up to one. If the random number hits this tiny difference
			// it will throw an exception. In such rare case, just expand the last guy's probability
			for (; random > 0 && k < VehicleWeights.Length; k++) {
				random -= VehicleWeights[k];
			}
			
			particles[i] = new SimulatedVehicle(VehicleParticles[k - 1], RenderAllParticles);
			models   [i] = new List<Gaussian>(MapModels[k - 1]);
			weights  [i] = 1.0 / weights.Length;
			random      += 1.0 / weights.Length;

			if (weights[i] > maxweight) {
				maxweight = weights[i];
				BestParticle = i;
			}
		}

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

		return 1.0/cum < MinEffectiveParticle * VehicleWeights.Length;
	}

	/// <summary>
	/// Predict the state of the map in the next iteration
	/// given the trajectory of the vehicle.
	/// </summary>
	/// <param name="measurements">Sensor measurements in range-bearing  form.</param>
	/// <param name="pose">Vehicle pose that conditions the mapping.</param>
	/// <param name="model">Associated map model.</param>
	/// <returns>Predicted map model.</returns>
	/// <remarks>Though in theory the predict doesn't depend on the measurements,
	/// the birth region (i.e. the unexplored areas) is large to represent on gaussians.
	/// To diminish the computational time, only the areas near the new measurements are
	/// used (i.e. the same measurements). This is equivalent to artificially increasing
	/// belief on the new measurements (when in a new area).</remarks>
	public List<Gaussian> PredictConditional(List<double[]> measurements, SimulatedVehicle pose, List<Gaussian> model, List<double[]> unexplored)
	{
		// gaussian are born on any unexplored areas,
		// as something is expected to be there.
		// but this is too resource-intensive so
		// here's a little cheat: only do it on areas
		// there was be a new measurement lately.
		List<Gaussian> predicted = new List<Gaussian>(model);
		
		// birth RFS
		foreach (double[] candidate in unexplored) {
			predicted.Add(new Gaussian(candidate, BirthCovariance, BirthWeight));
		}

		unexplored.Clear();

		foreach (double[] measurement in measurements) {
			double[] candidate = pose.MeasureToMap(measurement);
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
	public List<Gaussian> CorrectConditional(List<double[]> measurements, SimulatedVehicle pose, List<Gaussian> model)
	{
		List<Gaussian> corrected = new List<Gaussian>();

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
		double[][][] H  = new double  [model.Count][][];
		double[][][] P  = new double  [model.Count][][];
		double[][][] PH = new double  [model.Count][][];
		double[][][] S  = new double  [model.Count][][];
		Gaussian[]   q  = new Gaussian[model.Count];

		for (int i = 0; i < q.Length; i++) {
			Gaussian component = model[i];
			m [i] = component.Mean;
			H [i] = pose.MeasurementJacobian(m[i]);
			P [i] = component.Covariance;
			PH[i] = P[i].MultiplyByTranspose(H[i]);
			S [i] = H[i].Multiply(P[i].MultiplyByTranspose(H[i])).Add(R);
			q [i] = new Gaussian(pose.MeasurePerfect(m[i]), S[i], component.Weight);
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
		foreach (double[] measurement in measurements) {
			double PD        = pose.DetectionProbabilityM(measurement);	
			double weightsum = 0;

			for (int i = 0; i < q.Length; i++) {
				weightsum += q[i].Weight * q[i].Evaluate(measurement);
			}

			for (int i = 0; i < q.Length; i++) {
				if (Mahalanobis(q[i], measurement) > 3) {
					continue;
				}

				double[][] gain       = PH[i].Multiply(S[i].Inverse());
				double[]   mean       = m[i].Add(gain.Multiply(measurement.Subtract(pose.MeasurePerfect(m[i]))));
				double[][] covariance = I.Subtract(gain.Multiply(H[i])).Multiply(P[i]);

				double weight = PD * q[i].Weight * q[i].Evaluate(measurement) / (pose.ClutterDensity + PD * weightsum);

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
	public List<Gaussian> PruneModel(List<Gaussian> model)
	{
		List<Gaussian> pruned = new List<Gaussian>();
		Gaussian       candidate;
		List<Gaussian> close;

		model.Sort((a, b) => Math.Sign(b.Weight - a.Weight));

		for (int i = 0; i < Math.Min(MaxQuantity, model.Count); i++) {
			if (model[i].Weight < MinWeight) {
				break;
			}

			candidate = model[i];
			close     = new List<Gaussian>();

			for (int k = i + 1; k < Math.Min(MaxQuantity, model.Count); k++) {
				if (AreClose(model[i], model[k])) {
					close.Add(model[k]);
					model.RemoveAt(k--);
				}
			}

			if (close.Count > 0) {
				candidate = Merge(candidate, close);
			}

			pruned.Add(candidate);
		}

		// after the procedure, the list is almost sorted (only merges can mess with the order)
		// and this won't occur too many times with overlapping gaussians so this sort is not really necessary;
		// it purpose is to make rendering prettier (heavier gaussians drawn on top of lighter ones)
		//pruned.Sort((a, b) => Math.Sign(b.Weight - a.Weight));

		return pruned;
	}

	/// <summary>
	/// Queries if two gaussians are close to each other taking their
	/// "standard deviation" and a given threshold into consideration.
	/// </summary>
	/// <param name="a">First gaussian.</param>
	/// <param name="b">Second gaussian.</param>
	/// <returns>True if the gaussians are close enough to merge; false otherwise.</returns>
	private bool AreClose(Gaussian a, Gaussian b)
	{
		return Mahalanobis(a, b.Mean) < MergeThreshold;
	}

	/// <summary>
	/// Merge a list of gaussian components into a one big gaussian
	/// that tries to approximate as much as possible the behaviour
	/// of the original mixture.
	/// </summary>
	/// <param name="components">List of gaussian components.</param>
	/// <returns>Merged gaussian.</returns>
	private Gaussian Merge(Gaussian maincomponent, List<Gaussian> components)
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
	/// Obtain the Mahalanobis distance between a gaussian distribution and a point.
	/// </summary>
	/// <param name="distribution">Gaussian distribution in R^N space.</param>
	/// <param name="point">Point in R^N space.</param>
	/// <returns>Distance between distribution and point.</returns>
	private double Mahalanobis(Gaussian distribution, double[] point)
	{
		double[] diff = distribution.Mean.Subtract(point);
		return Math.Sqrt(Accord.Math.Matrix.InnerProduct(diff, distribution.CovarianceInverse.Multiply(diff)));
	}

	/// <summary>
	/// Obtain the squared Mahalanobis distance between a gaussian distribution and a point.
	/// </summary>
	/// <param name="distribution">Gaussian distribution in R^N space.</param>
	/// <param name="point">Point in R^N space.</param>
	/// <returns>Distance between distribution and point.</returns>
	private double MahalanobisSquared(Gaussian distribution, double[] point)
	{
		double[] diff = distribution.Mean.Subtract(point);
		return Accord.Math.Matrix.InnerProduct(diff, distribution.CovarianceInverse.Multiply(diff));
	}

	/// <summary>
	/// Queries if a location has been explored on a given map model.
	/// </summary>
	/// <param name="model">Map model.</param>
	/// <param name="x">Location.</param>
	/// <returns>True if the location has been explored.</returns>
	public bool Explored(List<Gaussian> model, double[] x)
	{
		return EvaluateModel(model, x) >= ExplorationThreshold;

		/*foreach (Gaussian component in model) {
			if (Mahalanobis(component, x) < ExplorationThreshold) {
				return true;
			}
		}

		return false;*/
	}
	/// <summary>
	/// Evaluate the Gaussian Mixture model of the map on a specified location.
	/// </summary>
	/// <param name="model">Evaluated model.</param>
	/// <param name="x">Location.</param>
	/// <returns>PHD density on the specified location.</returns>
	public double EvaluateModel(List<Gaussian> model, double[] x)
	{
		double value = 0;

		foreach (Gaussian component in model) {
			value += component.Weight * component.Evaluate(x);
		}

		return value;
	}

	/// <summary>
	/// Calculates the expected number of landmarks of a certain model.
	/// This turn out to be the sum of all the weights of the gaussians.
	/// </summary>
	/// <param name="model">Map model.</param>
	/// <returns>Expected number of landmarks.</returns>
	public double ExpectedSize(List<Gaussian> model)
	{
		double expected = 0;

		foreach (Gaussian component in model) {
			expected += component.Weight;
		}

		return expected;
	}

	/// <summary>
	/// Render the navigation HUD and the trajectory on the graphics device.
	/// The graphics device must be ready, otherwise
	/// the method will throw an exception.
	/// </summary>
	/// <param name="camera">Camera rotation matrix.</param>
	public void Render(double[][] camera)
	{
		if (RenderAllParticles) {
			foreach (var particle in VehicleParticles) {
				particle.RenderTrajectory(camera, Color.Blue);
			}
		}
		else {
			RenderTrajectory(camera);
		}

		RenderEstimate(camera);
	}

	/// <summary>
	/// Render the navigation HUD.
	/// </summary>
	/// <param name="camera">Camera rotation matrix.</param>
	public void RenderEstimate(double[][] camera) {
		foreach (Gaussian component in MapModels[BestParticle]) {
			RenderGaussian(component, camera);
		}
	}

	/// <summary>
	/// Render the path that the vehicle has traveled so far.
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
	/// Render the n-sigma ellipse of gaussian.
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

		double[][] recover = linear.Multiply(linear);

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
