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
using Accord.Math;
using Accord.Math.Decompositions;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

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
	public const int MaxQuantity = 200;

	/// <summary>
	/// Threshold used to decide when to merge close gaussian components.
	/// </summary>
	public const double MergeThreshold = 3e0;

	/// <summary>
	/// Threshold use to decide if a new measurement is in unexplored territory.
	/// If it is, it should be explored (birth of a new gaussian).
	/// </summary>
	public const double ExplorationThreshold = 1e-3;

	/// <summary>
	/// Particle filter representation of the vehicle pose.
	/// </summary>
	public Vehicle[] VehicleParticles { get;  private set; }

	/// <summary>
	/// PHD representation as a mixture-of-gaussians. Sorted.
	/// </summary>
	public List<Gaussian> MapModel { get; private set; }

	/// <summary>
	/// Prediction interval base model.
	/// </summary>
	public double[][] pinterval;

	/// <summary>
	/// Render output.
	/// </summary>
	public GraphicsDevice Graphics { get; set; }

	/// <summary>
	/// Construct a Navigator using the indicated vehicle as a reference.
	/// </summary>
	public Navigator(Vehicle vehicle)
	{
		VehicleParticles = new Vehicle[1];
		VehicleParticles[0] = vehicle;

		const int segments = 32;
		pinterval = new double[segments][];

		for (int i = 0; i < segments; i++) {
			pinterval[i] = new double[2] {Math.Cos(2 * Math.PI * i / segments), Math.Sin(2 * Math.PI * i / segments)};
		}

		MapModel = new List<Gaussian>();
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
	public void Update(List<double[]> measurements, bool predict, bool correct, bool prune)
	{
		if (predict) { PredictConditional(measurements, VehicleParticles[0]); }
		if (correct) { CorrectConditional(measurements, VehicleParticles[0]); }
		if (prune)   { PruneModel(); }
	}

	/// <summary>
	/// Predict the state of the map in the next iteration
	/// given the trajectory of the vehicle.
	/// </summary>
	/// <param name="measurements">Sensor measurements in range-bearing  form.</param>
	/// <param name="pose">Vehicle pose that conditions the mapping.</param>
	/// <remarks>Though in theory the predict doesn't depend on the measurements,
	/// the birth region (i.e. the unexplored areas) is large to represent on gaussians.
	/// To diminish the computational time, only the areas near the new measurements are
	/// used (i.e. the same measurements). This is equivalent to artificially increasing
	/// belief on the new measurements (when in a new area).</remarks>
	public void PredictConditional(List<double[]> measurements, Vehicle pose)
	{
		// gaussian are born on any unexplored areas,
		// as something is expected to be there.
		// but this is too resource-intensive so
		// here's a little cheat: only do it on areas
		// there will be a new measurement soon.
		List<double[]> unexplored = new List<double[]>();

		foreach (double[] measurement in measurements) {
			double[] candidate = pose.MeasureToMap(measurement);
			if (EvaluateModel(candidate) < ExplorationThreshold) {
				unexplored.Add(candidate);
			}
		}

		// birth RFS
		foreach (double[] candidate in unexplored) {
			MapModel.Add(new Gaussian(candidate, BirthCovariance, BirthWeight));
		}
	}

	/// <summary>
	/// Correct the estimated map given the localization
	/// using a set of measurements.
	/// </summary>
	/// <param name="measurements">Sensor measurements in range-bearing  form.</param>
	/// <param name="pose">Vehicle pose that conditions the mapping.</param>
	public void CorrectConditional(List<double[]> measurements, Vehicle pose)
	{
		List<Gaussian> corrected = new List<Gaussian>();

		// reduce predicted weight to acocunt for misdetection
		// v += (1-PD) v
		foreach (Gaussian component in MapModel) {
			corrected.Add(new Gaussian(component.Mean,
			                           component.Covariance,
			                           (1 - pose.DetectionProbability(component.Mean)) * component.Weight));
		}

		// measurement PHD update
		double[,]   R  = Vehicle.MeasurementCovariance;
		double[,]   I  = Accord.Math.Matrix.Identity(2);
		double[][]  m  = new double  [MapModel.Count][];
		double[][,] H  = new double  [MapModel.Count][,];
		double[][,] P  = new double  [MapModel.Count][,];
		double[][,] PH = new double  [MapModel.Count][,];
		double[][,] S  = new double  [MapModel.Count][,];
		Gaussian[]  q  = new Gaussian[MapModel.Count];

		for (int i = 0; i < q.Length; i++) {
			Gaussian component = MapModel[i];
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
			double PD = pose.DetectionProbabilityM(measurement);	
			double weightsum = 0;

			for (int i = 0; i < q.Length; i++) {
				weightsum += q[i].Weight * q[i].Evaluate(measurement);
			}

			for (int i = 0; i < q.Length; i++) {
				double[,] gain       = PH[i].Multiply(S[i].Inverse());
				double[]  innovation = measurement.Subtract(pose.MeasurePerfect(m[i]));
				double[]  mean       = m[i].Add(gain.Multiply(innovation));
				double[,] covariance = I.Subtract(gain.Multiply(H[i])).Multiply(P[i]);

				double weight = PD * q[i].Weight * q[i].Evaluate(measurement) / (ClutterDensity + PD * weightsum);

				corrected.Add(new Gaussian(mean, covariance, weight));
			}
		}

		this.MapModel = corrected;
	}

	/// <summary>
	/// Remove irrelevant gaussians from the map model.
	/// </summary>
	public void PruneModel()
	{
		List<Gaussian> pruned = new List<Gaussian>();
		Gaussian       candidate;
		List<Gaussian> close;

		this.MapModel.Sort((a, b) => Math.Sign(b.Weight - a.Weight));

		for (int i = 0; i < Math.Min(MaxQuantity, MapModel.Count); i++) {
			if (MapModel[i].Weight < MinWeight) {
				break;
			}

			candidate = MapModel[i];
			close     = new List<Gaussian>();

			for (int k = i + 1; k < Math.Min(MaxQuantity, MapModel.Count); k++) {
				if (areClose(MapModel[i], MapModel[k])) {
					close.Add(MapModel[k]);
					MapModel.RemoveAt(k--);
				}
			}

			if (close.Count > 0) {
				candidate = merge(candidate, close);
			}

			pruned.Add(candidate);
		}

		MapModel = pruned;
	}

	/// <summary>
	/// Queries if two gaussians are close to each other taking their
	/// "standard deviation" and a given threshold into consideration.
	/// </summary>
	/// <param name="a">First gaussian.</param>
	/// <param name="b">Second gaussian.</param>
	/// <returns>True if the gaussians are close enough to merge; false otherwise.</returns>
	private bool areClose(Gaussian a, Gaussian b)
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
	private Gaussian merge(Gaussian maincomponent, List<Gaussian> components)
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
		return Accord.Math.Matrix.InnerProduct(diff, distribution.Covariance.Inverse().Multiply(diff));
	}

	/// <summary>
	/// Evaluate the Gaussian Mixture model of the map on a specified location.
	/// </summary>
	/// <param name="x">Location.</param>
	/// <returns>PHD density on the specified location.</returns>
	public double EvaluateModel(double[] x)
	{
		double value = 0;

		foreach (Gaussian component in MapModel) {
			value += component.Weight * component.Evaluate(x);
		}

		return value;
	}

	/// <summary>
	/// Render the navigation HUD on the graphics device.
	/// The graphics device must be ready, otherwise
	/// the method will throw an exception.
	/// </summary>
	public void Render()
	{
		foreach (var component in  MapModel) {
			RenderGaussian(component);
		}
	}

	public void RenderGaussian(Gaussian gaussian)
	{
		//Color innercolor = Color.Coral;   innercolor.A = 200;
		//Color outercolor = Color.Crimson; outercolor.A = 200;

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
}

/// <summary>
/// Gaussian descriptor inside a gaussian mixture.
/// </summary>
public class Gaussian
{
	public double Weight { get; private set; }

	public double[] Mean { get; private set; }

	public double[,] Covariance { get; private set; }

	private double[,] CovarianceInverse;

	//private double[,] DeviationInverse;

	private double multiplier;

	/*private static double[] tabulated;

	private static int tabgrid;

	private static double tabmax;

	static Gaussian()
	{
		Gaussian standard = new Gaussian(new double[1] {0}, new double[1, 1] {{1}}, 1);

		tabgrid   = 200;
		tabmax    = 5.0*5.0;  // tabulate from 0 to 5 standard deviations
		tabulated = new double[tabgrid];

		for (int i = 0; i < tabgrid; i++) {
			tabulated[i] = standard.Evaluate(new double[1] {tabmax * Math.Sqrt(i) / tabgrid});
		}
	}*/

	public Gaussian(double[] mean, double[,] covariance, double weight)
	{
		this.Mean              = mean;
		this.Covariance        = covariance;
		this.Weight            = weight;
		this.multiplier        = Math.Pow(2 * Math.PI, -mean.Length / 2) / Math.Sqrt(covariance.Determinant());
		this.CovarianceInverse = covariance.Inverse();

		/*var decomp = new CholeskyDecomposition(covariance);
		double[,] sqrtdiag = decomp.DiagonalMatrix;

		for (int i = 0; i < sqrtdiag.GetLength(0); i++) {
			sqrtdiag[i, i] = Math.Sqrt(sqrtdiag[i, i]);
		}

		this.DeviationInverse = decomp.LeftTriangularFactor.Multiply(sqrtdiag).Inverse();*/
	}

	public double Evaluate(double[] x)
	{
		double[]  diff = x.Subtract(Mean);
		return multiplier * Math.Exp(-0.5 * Accord.Math.Matrix.InnerProduct(diff, CovarianceInverse.Multiply(diff)));
	}

	/*public double ApproximateEvaluate(double[] x)
	{
		return ApproximateEvaluate1D(DeviationInverse.Multiply(x).Subtract(Mean).SquareEuclidean(Mean));
	}

	public double ApproximateEvaluate1D(double x2)
	{
		x2 = (x2 <= tabmax) ? x2 : tabmax;
		return tabulated[(int)(tabgrid * x2 / tabmax)];
	}*/
}
}
