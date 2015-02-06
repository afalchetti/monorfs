// Navigator.cs
// SLAM solving navigator
// Part of MonoRFS
//
// Copyright (C) 2015 Angelo Falchetti
// All rights reserved.
// Any use, reproduction, distribution or copy of this
// work via any medium is strictly forbidden without
// express written consent from the author.

// TODO navigator shouldn't know the exact measurement/model noise variances, should give them on the constructor

using System;
using System.Collections.Generic;
using Accord.Math;
using Accord.Math.Decompositions;
using AForge;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace monorfs
{
/// <summary>
/// SLAM solver. It uses the PHD Filter.
/// </summary>
public class Navigator
{/*
	/// <summary>
	/// Landmark density expected on unexplored areas.
	/// </summary>
	public const double BirthWeight = 0.1;

	/// <summary>
	/// Landmark density spread expected on unexplored areas
	/// </summary>
	public static readonly double[,] BirthCovariance = new double[2, 2] {{1e-2, 0},
	                                                                     {0, 1e-2}};

	/// <summary>
	/// Amount of expected clutter (spuriousness) on the measurement process.
	/// </summary>
	public const double ClutterDensity = 1e-2;

	/// <summary>
	/// Minimum weight that is kept after a model prune.
	/// </summary>
	public const double MinWeight = 1e-2;

	/// <summary>
	/// Maximum number of gaussians kept after a model prune.
	/// </summary>
	public const int MaxQuantity = 50;

	/// <summary>
	/// Threshold used to decide when to merge close gaussian components.
	/// </summary>
	public const double MergeThreshold = 3e0;

	/// <summary>
	/// Threshold use to decide if a new measurement is in unexplored territory.
	/// If it is, it should be explored (birth of a new gaussian).
	/// </summary>
	public const double ExplorationThreshold = 1e-3;
	//public const double ExplorationThreshold = 3;

	/// <summary>
	/// Particle filter representation of the vehicle pose.
	/// </summary>
	public Vehicle[] VehicleParticles { get;  private set; }

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
	/// Expected number of landmarks after the prediction step.
	/// </summary>
	public double[] Mpredicted;

	/// <summary>
	/// Expected number of landmarks after the correction step.
	/// </summary>
	public double[] Mcorrected;

	/// <summary>
	/// Index of the particle with the biggest weight.
	/// </summary>
	public int BestParticle;

	/// <summary>
	/// True if the localization of the vehicle is known.
	/// </summary>
	public bool OnlyMapping;

	/// <summary>
	/// Prediction interval base model.
	/// </summary>
	public double[][] pinterval;

	/// <summary>
	/// The estimated trajectory.
	/// </summary>
	public List<double[]> Waypoints;

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
	/// Construct a Navigator using the indicated vehicle as a reference.
	/// </summary>
	/// <param name="vehicle">Vehicle to track.</param>
	/// <param name="particlecount">Number of particles for the Montecarlo filter.</param>
	/// <param name="onlymapping">If true, don't do SLAM, but mapping (i.e. the localization is assumed known).</param>
	public Navigator(Vehicle vehicle, int particlecount, bool onlymapping = false)
	{
		this.OnlyMapping = onlymapping;

		if (onlymapping) {
			// a reference copy, which gives the navigator precise info about
			// the real vehicle pose
			this.VehicleParticles = new Vehicle       [1];
			this.MapModels        = new List<Gaussian>[1];
			this.VehicleWeights   = new double[1];

			this.VehicleParticles[0] = vehicle;
			this.MapModels       [0] = new List<Gaussian>();
			this.VehicleWeights  [0] = 1;
		}
		else {	
			// if doing SLAM, do a deep copy, so no real info flows
			// into the navigator, only estimates
			this.VehicleParticles = new Vehicle       [particlecount];
			this.MapModels        = new List<Gaussian>[particlecount];
			this.VehicleWeights   = new double        [particlecount];

			for (int i = 0; i < particlecount; i++) {
				this.VehicleParticles[i] = new Vehicle(vehicle);
				this.MapModels       [i] = new List<Gaussian>();
				this.VehicleWeights  [i] = 1.0 / particlecount;
			}
		}
		
		this.Mpredicted   = new double[VehicleParticles.Length];
		this.Mcorrected   = new double[VehicleParticles.Length];
		this.BestParticle = 0;

		this.Waypoints = new List<double[]>();
		this.Waypoints.Add(vehicle.Location);

		const int segments = 32;
		this.pinterval = new double[segments][];

		// pinterval will be the 5-sigma ellipse
		for (int i = 0; i < segments; i++) {
			this.pinterval[i] = new double[2] {5*Math.Cos(2 * Math.PI * i / segments), 5*Math.Sin(2 * Math.PI * i / segments)};
		}
	}

	/// <summary>
	/// Update the vehicle particles.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="dx">Odometry forward movement.</param>
	/// <param name="dy">Odometry perpendicular movement.</param>
	/// <param name="dtheta">Odometry angular movement.</param>
	public void Update(GameTime time, double dx, double dy, double dtheta)
	{
		if (!OnlyMapping) {
			for (int i = 0; i < VehicleParticles.Length; i++) {
				VehicleParticles[i].Update(time, dx, dy, dtheta);
			}
		}

		if (VehicleParticles[BestParticle].Location.Subtract(Waypoints[Waypoints.Count - 1]).SquareEuclidean() >= 1e-2f) {
			Waypoints.Add(VehicleParticles[BestParticle].Location);
		}
	}

	/// <summary>
	/// Update both the estimated map and the localization.
	/// This means doing a model prediction and a measurement update.
	/// This method is the core of the whole program.
	/// </summary>
	/// <param name="measurements">Sensor measurements in range-bearing  form.</param>
	/// <param name="predict">Predict flag; if false, no prediction step is done.</param>
	/// <param name="correct">Correct flag; if false, no correction step is done.</param>
	/// <param name="prune">Prune flag; if false, no prune step is done.</param>
	public void SlamUpdate(List<double[]> measurements, bool predict, bool correct, bool prune)
	{
		// map update
		Parallel.For (0, VehicleParticles.Length, i => {
			if (predict) { MapModels[i] = PredictConditional(measurements, VehicleParticles[i], MapModels[i]); }
			Mpredicted[i] = ExpectedSize(MapModels[i]);

			if (correct) { MapModels[i] = CorrectConditional(measurements, VehicleParticles[i], MapModels[i]); }
			if (prune)   { MapModels[i] = PruneModel(MapModels[i]); }
			Mcorrected[i] = ExpectedSize(MapModels[i]);
		});

		// localization update
		if (!OnlyMapping) {
			UpdateParticleWeights(Mpredicted, Mcorrected);
			ResampleParticles();
		}
	}

	/// <summary>
	/// Update the weights of each vehicle using the sequential importance sampling technique.
	/// It uses the empty map technique for now as it is simple and gives reasonable results.
	/// </summary>
	public void UpdateParticleWeights(double[] Mpredicted, double[] Mcorrected)
	{
		for (int i = 0; i < VehicleWeights.Length; i++) {
			// using the empty-map trick, the new weight follows approximately the formula
			// w' = k^|Z| exp(|Mcorrected| - |Mpredicted| - integral of k) w, where
			// k   is the clutter density,
			// |Z| is the number of measurements
			// |M| is the expected number of landmarks (integral of the map model)
			// interestingly, given k a constant density, k^|Z| and the integral of k are constant too
			// so there's no real need to use them (as they will drop after the renormalization)
			VehicleWeights[i] *= Math.Exp(Mcorrected[i] - Mpredicted[i]);
		}

		VehicleWeights = VehicleWeights.Divide(VehicleWeights.Sum());
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
		double           random    = (double) Util.uniform.Next() / VehicleWeights.Length;
		double[]         weights   = new double[VehicleWeights.Length];
		Vehicle[]        particles = new Vehicle[VehicleParticles.Length];
		List<Gaussian>[] models    = new List<Gaussian>[MapModels.Length];
		double           maxweight = 0;

		for (int i = 0, k = 0; i < weights.Length; i++) {
			// k should never be out of range, but because of floating point arithmetic,
			// the probabilities may not add up to one. If the random number hits this tiny difference
			// it will throw an exception. In such rare case, just expand the last guy's probability
			for (; random > 0 && k < VehicleWeights.Length; k++) {
				random -= VehicleWeights[k];
			}
			
			particles[i] = new Vehicle(VehicleParticles[k - 1]);
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
	public List<Gaussian> PredictConditional(List<double[]> measurements, Vehicle pose, List<Gaussian> model)
	{
		// gaussian are born on any unexplored areas,
		// as something is expected to be there.
		// but this is too resource-intensive so
		// here's a little cheat: only do it on areas
		// there will be a new measurement soon.
		List<double[]> unexplored = new List<double[]>();

		foreach (double[] measurement in measurements) {
			double[] candidate = pose.MeasureToMap(measurement);
			if (!Explored(model, candidate)) {
				unexplored.Add(candidate);
			}
		}

		List<Gaussian> predicted = new List<Gaussian>(model);

		// birth RFS
		foreach (double[] candidate in unexplored) {
			predicted.Add(new Gaussian(candidate, BirthCovariance, BirthWeight));
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
	public List<Gaussian> CorrectConditional(List<double[]> measurements, Vehicle pose, List<Gaussian> model)
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
		double[,]   R  = pose.MeasurementCovariance;
		double[,]   I  = Accord.Math.Matrix.Identity(2);
		double[][]  m  = new double  [model.Count][];
		double[][,] H  = new double  [model.Count][,];
		double[][,] P  = new double  [model.Count][,];
		double[][,] PH = new double  [model.Count][,];
		double[][,] S  = new double  [model.Count][,];
		Gaussian[]  q  = new Gaussian[model.Count];

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
				if (Mahalanobis(q[i], measurement) > 5) {
					continue;
				}

				double[,] gain       = PH[i].Multiply(S[i].Inverse());
				double[]  mean       = m[i].Add(gain.Multiply(measurement.Subtract(pose.MeasurePerfect(m[i]))));
				double[,] covariance = I.Subtract(gain.Multiply(H[i])).Multiply(P[i]);

				double weight = PD * q[i].Weight * q[i].Evaluate(measurement) / (ClutterDensity + PD * weightsum);

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
		double    weight     = maincomponent.Weight;
		double[]  mean       = maincomponent.Weight.Multiply(maincomponent.Mean);
		double[,] covariance = maincomponent.Weight.Multiply(maincomponent.Covariance);

		// merged gaussian follows the rules
		// w = sum of (wi)
		// m = sum of (wi mi) / w
		// P = sum of (wi (Pi + (mi - m0) (mi - m0)^T)) / w
		foreach (Gaussian component in components) {
			double[] diff = component.Mean.Subtract(maincomponent.Mean);
			weight    += component.Weight;
			mean       = mean      .Add(component.Weight.Multiply(component.Mean));
			covariance = covariance.Add(component.Weight.Multiply(component.Covariance.Add(diff.OuterProduct(diff))));
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
		return EvaluateModel(model, x) > ExplorationThreshold;

		/*foreach (Gaussian component in model) {
			if (Mahalanobis(component, x) < ExplorationThreshold) {
				return true;
			}
		}

		return false;* /
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
	/// Render the navigation HUD on the graphics device.
	/// The graphics device must be ready, otherwise
	/// the method will throw an exception.
	/// </summary>
	public void Render()
	{
		foreach (Gaussian component in MapModels[BestParticle]) {
			RenderGaussian(component);
		}
	}

	/// <summary>
	/// Render the path that the vehicle has travelled so far.
	/// </summary>
	public void RenderTrajectory()
	{
		VertexPositionColor[] vertices = new VertexPositionColor[Waypoints.Count];
		Color color = Color.Blue;

		for (int i = 0; i < Waypoints.Count; i++) {
			vertices[i] = new VertexPositionColor(new Vector3((float) Waypoints[i][0], (float) Waypoints[i][1], 0), color);
		}

		Graphics.DrawUser2DPolygon(vertices, 0.02f, color, false);
	}

	public void RenderGaussian(Gaussian gaussian)
	{
		Color innercolor = Color.DeepSkyBlue; innercolor.A = 200;
		Color outercolor = Color.Blue;        outercolor.A = 200;

		var decomp = new EigenvalueDecomposition(gaussian.Covariance);

		double[,] stddev = decomp.DiagonalMatrix;

		for (int i = 0; i < stddev.GetLength(0); i++) {
			stddev[i, i] = (stddev[i, i] > 0) ? Math.Sqrt(stddev[i, i]) : 0;
		}

		double[,] rotation = decomp.Eigenvectors;
		double[,] linear   = rotation.Multiply(stddev);

		VertexPositionColor[] vertices = new VertexPositionColor[pinterval.Length];

		for (int i = 0; i < vertices.Length; i++) {
			double[] position = linear.Multiply(pinterval[i]).Add(gaussian.Mean);
			vertices[i] = new VertexPositionColor(new Vector3((float) position[0], (float) position[1], 0), innercolor);
		}

		short[] index = new short[vertices.Length];

		for (int i = 0, k = 0; k < vertices.Length; i++, k +=2) {
			index[k] = (short) i;
		}

		for (int i = vertices.Length - 1, k = 1; k < vertices.Length; i--, k += 2) {
			index[k] = (short) i;
		}

		Graphics.DrawUserIndexedPrimitives(PrimitiveType.TriangleStrip, vertices, 0, vertices.Length, index, 0, vertices.Length - 2);
		Graphics.DrawUser2DPolygon(vertices, 0.04f * (float) gaussian.Weight, outercolor, true);
	}
*/}

/// <summary>
/// Gaussian descriptor inside a gaussian mixture.
/// </summary>
public class Gaussian
{
	public double Weight { get; private set; }

	public double[] Mean { get; private set; }

	public double[,] Covariance { get; private set; }

	public double[,] CovarianceInverse;

	public double multiplier;

	private static readonly double[] tabulatedexp;

	private const int tabgrid = 128;

	private const double tabdelta = 128 / 16;

	private const double tabmax = 16;

	static Gaussian()
	{
		tabulatedexp = new double[tabgrid + 1];

		for (int i = 0; i < tabgrid + 1; i++) {
			tabulatedexp[i] = Math.Exp(-tabmax * i / tabgrid);
		}
	}

	public Gaussian(double[] mean, double[,] covariance, double weight)
	{
		this.Mean              = mean;
		this.Covariance        = covariance;
		this.Weight            = weight;
		this.multiplier        = Math.Pow(2 * Math.PI, -mean.Length / 2) / Math.Sqrt(covariance.Determinant());
		this.CovarianceInverse = covariance.Inverse();
	}

	public double Evaluate(double[] x)
	{
		double[]  diff = x.Subtract(Mean);
		return multiplier * ApproximateNegExp(0.5 * Accord.Math.Matrix.InnerProduct(diff, CovarianceInverse.Multiply(diff)));
		//return multiplier * Math.Exp(-0.5 * Accord.Math.Matrix.InnerProduct(diff, CovarianceInverse.Multiply(diff)));
	}

	public double ApproximateNegExp(double x)
	{
		if (x <= tabmax) {
			return tabulatedexp[(int)(tabdelta * x)];
		}
		else {
			return 0;
		}
	}
}
}
