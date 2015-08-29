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

using Accord.Math;
using AForge;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

using TimedState    = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedMapModel = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<monorfs.Gaussian>>>;

namespace monorfs
{
/// <summary>
/// SLAM solver. It uses the PHD filter.
/// </summary>
public class PHDNavigator : Navigator
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
	/// Number of particles for the Montecarlo filter.
	/// </summary>
	public int ParticleCount { get; set; }

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
	/// Index of the particle with the biggest weight.
	/// </summary>
	public int BestParticle { get; private set; }

	/// <summary>
	/// Most accurate estimate of the current vehicle pose.
	/// </summary>
	public override SimulatedVehicle BestEstimate
	{
		get
		{
			return VehicleParticles[BestParticle];
		}
		set
		{
			VehicleParticles[BestParticle] = value;
		}
	}

	/// <summary>
	/// Most accurate estimate model of the map.
	/// </summary>
	public override List<Gaussian> BestMapModel
	{
		get
		{
			return MapModels[BestParticle];
		}
		set
		{
			MapModels[BestParticle] = value;
		}
	}

	/// <summary>
	/// Previous frame unexplored particles (delayed birth avoids spurious measurement super-certainty).
	/// </summary>
	private List<double[]>[] toexplore;

	/// <summary>
	/// If true every particle history is rendered onscreen;
	/// otherwise, only the best particle is.
	/// </summary>
	public static bool RenderAllParticles { get { return Config.RenderAllParticles; } }

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
	public PHDNavigator(Vehicle vehicle, int particlecount, bool onlymapping = false)
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
		reset(RefVehicle, new List<Gaussian>(), new List<double[]>(), particlecount);
	}

	/// <summary>
	/// Change the mode of the navigator to solving full slam.
	/// The unique particle currently in use will be multiplied into new localization particles.
	/// </summary>
	public override void StartSlamInternal()
	{
		CollapseParticles(ParticleCount);
	}

	/// <summary>
	/// Change the mode of the navigator to do only mapping.
	/// The vehicle pose resets to the correct location, but the map is inherited
	/// from the previous best particle (maximum a posteriori).
	/// </summary>
	public override void StartMappingInternal()
	{
		CollapseParticles(1);
	}

	/// <summary>
	/// Collapse the vehicle particles to the best previous particle.
	/// </summary>
	/// <param name="particlecount">Number of particles for the Montecarlo filter.</param>
	public void CollapseParticles(int particlecount)
	{
		reset(BestEstimate, BestMapModel, toexplore[BestParticle], particlecount);
	}

	/// <summary>
	/// Reset the state of the navigator to set of equal particles.
	/// </summary>
	/// <param name="vehicle">New vehicle pose.</param>
	/// <param name="model">New map model estimate.</param>
	/// <param name="explore">New qeued measurements to explore.</param>
	/// <param name="particlecount">Number of particles for the Montecarlo filter.</param>
	private void reset(Vehicle vehicle, List<Gaussian> model, List<double[]> explore, int particlecount)
	{
		// do a deep copy, so no real info flows
		// into the navigator, only estimates, in SLAM
		// (this avoid bugs where the user unknowingly uses
		// the groundtruth to estimate the groundtruth itself)
		VehicleParticles = new SimulatedVehicle[particlecount];
		MapModels        = new List<Gaussian>  [particlecount];
		VehicleWeights   = new double          [particlecount];
		toexplore        = new List<double[]>  [particlecount];

		for (int i = 0; i < particlecount; i++) {
			VehicleParticles[i] = new SimulatedVehicle(vehicle, 1.8, 1.8, 0.7, 5e-7);
			MapModels       [i] = new List<Gaussian>(model);
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
	/// <param name="dx">Moved distance from odometry in the local vertical movement-perpendicular direction since last timestep.</param>
	/// <param name="dy">Moved distance from odometry in the local horizontal movement-perpendicular direction since last timestep.</param>
	/// <param name="dz">Moved distance from odometry in the local depth movement-parallel direction since last timestep.</param>
	/// <param name="dyaw">Angle variation from odometry in the yaw coordinate since last timestep.</param>
	/// <param name="dpitch">Angle variation from odometry in the pitch coordinate since last timestep.</param>
	/// <param name="droll">Angle variation from odometry in the roll coordinate since last timestep.</param>
	public override void Update(GameTime time, double dx, double dy, double dz,
	                            double dyaw, double dpitch, double droll)
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

		UpdateTrajectory(time);
	}

	/// <summary>
	/// Update both the estimated map and the localization.
	/// This means doing a model prediction and a measurement update.
	/// This method is the core of the whole program.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="measurements">Sensor measurements in pixel-range form.</param>
	public override void SlamUpdate(GameTime time, List<double[]> measurements)
	{
		// map update
		Parallel.For (0, VehicleParticles.Length, i => {
			List<Gaussian> predicted, corrected;

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
			double sum = VehicleWeights.Sum();

			if (sum == 0) {
				sum = 1;
			}

			VehicleWeights = VehicleWeights.Divide(sum);

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
	/// <returns>Total likelihood weight.</returns>
	public double WeightAlpha(List<double[]> measurements, List<Gaussian> predicted, List<Gaussian> corrected, SimulatedVehicle pose)
	{
		List<Gaussian> visible = corrected.FindAll(g => pose.Visible(g.Mean) && g.Weight > 0.8);

		// exact calculation if there are few components/measurements;
		// use the most probable components approximation otherwise
		SparseMatrix logprobs = new SparseMatrix(visible.Count + measurements.Count, visible.Count + measurements.Count, double.NegativeInfinity);

		double     logPD      = Math.Log(pose.PD);
		double     logclutter = Math.Log(pose.ClutterDensity);
		Gaussian[] zprobs     = new Gaussian[visible.Count];


		for (int i = 0; i < visible.Count; i++) {
			zprobs[i] = new Gaussian(pose.MeasurePerfect(visible[i].Mean), pose.MeasurementCovariance, 1); 
		}

		for (int i = 0; i < zprobs.Length;      i++) {
		for (int k = 0; k < measurements.Count; k++) {
			double d = zprobs[i].Mahalanobis(measurements[k]);
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
		double             random    = (double) Util.Uniform.Next() / VehicleWeights.Length;
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
	/// <param name="unexplored">Input/Output, unexplored candidates from previous timestep..</param> 
	/// <returns>Predicted map model.</returns>
	/// <remarks>Though in theory the predict doesn't depend on the measurements,
	/// the birth region (i.e. the unexplored areas) is large to represent on gaussians.
	/// To diminish the computational time, only the areas near the new measurements are
	/// used (i.e. the same measurements). This is equivalent to artificially increasing
	/// belief on the new measurements (when in a new area).</remarks>
	public List<Gaussian> PredictConditional(List<double[]> measurements, SimulatedVehicle pose, List<Gaussian> model, List<double[]> unexplored)
	{
		// gaussian are born on any unexplored areas,
		// as something is expected to be there,
		// but this is too resource-intensive so
		// here's a little cheat: only do it on areas
		// there has been a new measurement lately
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
				if (q[i].Mahalanobis(measurement) > 3) {
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
				if (Gaussian.AreClose(model[i], model[k], MergeThreshold)) {
					close.Add(model[k]);
					model.RemoveAt(k--);
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
	private static Gaussian Merge(Gaussian maincomponent, List<Gaussian> components)
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
	public bool Explored(List<Gaussian> model, double[] x)
	{
		return EvaluateModel(model, x) >= ExplorationThreshold;
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
	/// <param name="camera">Camera 4d transform matrix.</param>
	public override void Render(double[][] camera)
	{
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
