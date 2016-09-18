// Gaussian.cs
// Gaussian component details
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
using System.Text;

using Accord.Math;

namespace monorfs
{
/// <summary>
/// Gaussian descriptor inside a gaussian mixture.
/// </summary>
public class Gaussian
{
	/// <summary>
	/// Gaussian coefficient on a mixture.
	/// </summary>
	/// <remarks>
	/// Though it may be seen as external to the gaussian distribution,
	/// this class is a gaussian descriptor, not a gaussian distribution.
	/// </remarks>
	public double Weight { get; private set; }

	/// <summary>
	/// Distribution center of symmetry. It is also the most dense point.
	/// </summary>
	public double[] Mean { get; private set; }

	/// <summary>
	/// Multidimensional dispersion measure.
	/// </summary>
	public double[][] Covariance { get; private set; }

	// Cached values

	/// <summary>
	/// Cached covariance inverse.
	/// </summary>
	public double[][] CovarianceInverse { get; private set; }

	/// <summary>
	/// Matrix from the canonical representation.
	/// </summary>
	public double[][] CanonicalMatrix
	{
		get         { return CovarianceInverse;  }
		private set { CovarianceInverse = value; }
	}

	/// <summary>
	/// Vector from the canonical representation.
	/// </summary>
	public double[] CanonicalVector { get; private set; }

	/// <summary>
	/// Cached coefficient from the function definition.
	/// </summary>
	public double Multiplier { get; private set; }

	// Tabular constants

	/// <summary>
	/// Negative exponential function tabulation.
	/// </summary>
	private static readonly double[] tabulatedexp;

	/// <summary>
	/// Number of tabulated points for the exponential function.
	/// </summary>
	private const int tabgrid = 32;

	/// <summary>
	/// Maximum evaluation point for the exponential function.
	/// </summary>
	private const double tabmax = 16;

	/// <summary>
	/// Cached tabulation difference between each data point.
	/// </summary>
	private const double tabdelta = 32 / 16;

	/// <summary>
	/// Construct the negative exponential tabulation.
	/// </summary>
	static Gaussian()
	{
		tabulatedexp = new double[tabgrid + 1];

		for (int i = 0; i < tabgrid + 1; i++) {
			tabulatedexp[i] = Math.Exp(-tabmax * i / tabgrid);
		}
	}

	/// <summary>
	/// Construct a default gaussian.
	/// </summary>
	private Gaussian() {}

	/// <summary>
	/// Construct a new Gaussian object given its parameters.
	/// </summary>
	/// <param name="mean">Gaussian mean value.</param>
	/// <param name="covariance">Gaussian covariance matrix.</param>
	/// <param name="weight">Gaussian coefficient in the mixture.</param>
	public Gaussian(double[] mean, double[][] covariance, double weight)
	{
		Mean              = mean;
		Covariance        = covariance;
		Weight            = (!double.IsNaN(weight)) ? weight : 0;
		Multiplier        = Math.Pow(2 * Math.PI, -mean.Length / 2) / Math.Sqrt(covariance.PseudoDeterminant());
		CovarianceInverse = covariance.PseudoInverse();
		CanonicalVector   = CovarianceInverse.Multiply(Mean);
	}

	/// <summary>
	/// Construct a new Gaussian object with alternative canonical parameters.
	/// </summary>
	/// <param name="vector">Canonical vector.</param>
	/// <param name="matrix">Canonical matrix.</param>
	/// <param name="weight">Gaussian coefficient in the mixture.</param>
	public static Gaussian Canonical(double[] vector, double[][] matrix, double weight)
	{
		Gaussian gaussian = new Gaussian();

		gaussian.CanonicalVector = vector;
		gaussian.CanonicalMatrix = matrix;

		gaussian.Covariance = matrix.PseudoInverse();
		gaussian.Mean       = gaussian.Covariance.Multiply(vector);

		gaussian.Weight     = (!double.IsNaN(weight)) ? weight : 0;
		gaussian.Multiplier = Math.Pow(2 * Math.PI, -gaussian.Mean.Length / 2) / Math.Sqrt(gaussian.Covariance.Determinant());

		return gaussian;
	}

	/// <summary>
	/// Evaluate the gaussian component, although without the weight multiplication.
	/// </summary>
	/// <param name="x">Evaluation point.</param>
	/// <returns>Distribution density.</returns>
	public double Evaluate(double[] x)
	{
		double[]  diff = x.Subtract(Mean);
		return Multiplier * ApproximateNegExp(0.5 * diff.InnerProduct(CovarianceInverse.Multiply(diff)));
		//return Multiplier * Math.Exp(-0.5 * diff.InnerProduct(CovarianceInverse.Multiply(diff)));
	}

	/// <summary>
	/// Approximate (pre-tabulated) negative exponential function, exp(-x).
	/// </summary>
	/// <param name="x">Evaluation point.</param>
	/// <returns>Function evaluation.</returns>
	public static double ApproximateNegExp(double x)
	{
		if (0 <= x && x <= tabmax) {
			return tabulatedexp[(int)(tabdelta * x)];
		}
		else if (-tabmax <= x && x < 0) {
			return 1.0/tabulatedexp[(int)(tabdelta * -x)];
		}
		else {
			return 0;
		}
	}

	/// <summary>
	/// Queries if two gaussians are close to each other taking their
	/// "standard deviation" and a given threshold into consideration.
	/// </summary>
	/// <param name="a">First gaussian.</param>
	/// <param name="b">Second gaussian.</param>
	/// <param name="threshold">Mahalanobis distance to be considered close.</param> 
	/// <returns>True if the gaussians are close enough to merge; false otherwise.</returns>
	public static bool AreClose(Gaussian a, Gaussian b, double threshold)
	{
		return a.SquareMahalanobis(b.Mean) < threshold * threshold;
	}

	/// <summary>
	/// Fuse two gaussian by multiplication (Bayes).
	/// </summary>
	/// <param name="a">First gaussian.</param>
	/// <param name="b">Second gaussian.</param>
	public static Gaussian Fuse(Gaussian a, Gaussian b)
	{
		double[][] infomatrix = a.CanonicalMatrix.Add(b.CanonicalMatrix);
		double[]   infovector = a.CanonicalVector.Add(b.CanonicalVector);

		return Canonical(infovector, infomatrix, 1.0);

	}

	/// <summary>
	/// Unfuse a gaussian component from a fused gaussian.
	/// Inverse of of Fuse()
	/// </summary>
	/// <param name="a">Fused gaussian.</param>
	/// <param name="b">Gaussian component to remove.</param>
	public static Gaussian Unfuse(Gaussian a, Gaussian b)
	{
		double[][] infomatrix = a.CanonicalMatrix.Subtract(b.CanonicalMatrix);
		double[]   infovector = a.CanonicalVector.Subtract(b.CanonicalVector);

		return Canonical(infovector, infomatrix, 1.0);

	}

	/// <summary>
	/// Obtain the Mahalanobis distance between the gaussian distribution and a point.
	/// </summary>
	/// <param name="point">Point in R^N space.</param>
	/// <returns>Distance between distribution and point.</returns>
	public double Mahalanobis(double[] point)
	{
		double[] diff = Mean.Subtract(point);
		return Math.Sqrt(diff.InnerProduct(CovarianceInverse.Multiply(diff)));
	}

	/// <summary>
	/// Obtain the squared Mahalanobis distance between the gaussian distribution and a point.
	/// </summary>
	/// <param name="point">Point in R^N space.</param>
	/// <returns>Squared distance between distribution and point.</returns>
	public double SquareMahalanobis(double[] point)
	{
		double[] diff = Mean.Subtract(point);
		return diff.InnerProduct(CovarianceInverse.Multiply(diff));
	}

	/// <summary>
	/// Get a linear string representation in column-major order.
	/// </summary>
	/// <param name="format">Double formatting descriptor.</param>
	/// <returns>String representation.</returns>
	public string ToString(string format)
	{
		StringBuilder serialized = new StringBuilder();

		serialized.Append(Weight.ToString(format));
		serialized.Append(";");
		
		serialized.Append(Mean[0].ToString(format));

		for (int i = 1; i < Mean.Length; i++) {
			serialized.Append(" ");
			serialized.Append(Mean[i].ToString(format));
		}

		serialized.Append(";");
		
		serialized.Append(Covariance[0][0].ToString(format));
		
		for (int k = 1; k < Covariance[0].Length; k++) {
			serialized.Append(" ");
			serialized.Append(Covariance[0][k].ToString(format));
		}

		for (int i = 1; i < Covariance   .Length; i++) {
		for (int k = 0; k < Covariance[i].Length; k++) {
			serialized.Append(" ");
			serialized.Append(Covariance[i][k].ToString(format));
		}
		}

		return serialized.ToString();
	}

	/// <summary>
	/// Get a linear string representation in column-major order.
	/// </summary>
	/// <returns>String representation.</returns>
	public override string ToString()
	{
		return this.ToString("g6");
	}

	/// <summary>
	/// Efficient equality comparer with another gaussian.
	/// </summary>
	/// <param name="that">Compared gaussian.</param>
	/// <param name="threshold">Acceptable differece threshold per each item
	/// (weight, mean component or covariance component).</param>
	/// <returns>True if both gaussians are exactly the same.</returns>
	public bool Equals(Gaussian that, double threshold)
	{
		return this.Weight.IsEqual(that.Weight, threshold) &&
		       this.Mean.IsEqual(that.Mean, threshold) &&
		       this.Covariance.IsEqual(that.Covariance, threshold);
	}

	/// <summary>
	/// Efficient equality comparer with another gaussian using
	/// exact floating point comparison.
	/// </summary>
	/// <param name="that">Compared gaussian.</param>
	/// <returns>True if both gaussians are exactly the same.</returns>
	public bool Equals(Gaussian that)
	{
		return this.Equals(that, 0.0);
	}

	/// <summary>
	/// Compare this object with another.
	/// </summary>
	/// <param name="that">Compared object.</param>
	/// <returns>True if the objects are the same.</returns>
	public override bool Equals(object that)
	{
		return that is Gaussian && this.Equals(that as Gaussian);
	}

	/// <summary>
	/// Get a unique code that is equal for any two equal Gaussians.
	/// </summary>
	/// <returns>Hash code.</returns>
	public override int GetHashCode()
	{
		int hash = 17;

		hash = unchecked(37 * hash + Weight.GetHashCode());

		for (int i = 0; i < Mean.Length; i++) {
			hash = unchecked(37 * hash + Mean[i].GetHashCode());
		}

		for (int i = 0; i < Covariance.Length; i++) {
		for (int k = 0; k < Covariance[i].Length; k++) {
			hash = unchecked(37 * hash + Covariance[i][k].GetHashCode());
		}
		}

		return hash;
	}
}
}
