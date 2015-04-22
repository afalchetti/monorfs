// Gaussian.cs
// Gaussian component details
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
	public double[][] CovarianceInverse;

	/// <summary>
	/// Cached coefficient from the function definition.
	/// </summary>
	public double Multiplier;

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
		Multiplier        = Math.Pow(2 * Math.PI, -mean.Length / 2) / Math.Sqrt(covariance.Determinant());
		CovarianceInverse = covariance.Inverse();
	}

	/// <summary>
	/// Evaluate the gaussian component, although without the weight multiplication.
	/// </summary>
	/// <param name="x">Evaluation point.</param>
	/// <returns>Distribution density.</returns>
	public double Evaluate(double[] x)
	{
		double[]  diff = x.Subtract(Mean);
		return Multiplier * ApproximateNegExp(0.5 * Accord.Math.Matrix.InnerProduct(diff, CovarianceInverse.Multiply(diff)));
		//return multiplier * Math.Exp(-0.5 * Accord.Math.Matrix.InnerProduct(diff, CovarianceInverse.Multiply(diff)));
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
	/// Get a linear string representation in column-major order.
	/// </summary>
	public string LinearSerialization
	{
		get
		{
			StringBuilder serialized = new StringBuilder();

			serialized.Append(Weight.ToString("F6"));
			serialized.Append(";");
			
			serialized.Append(Mean[0].ToString("F6"));

			for (int i = 1; i < Mean.Length; i++) {
				serialized.Append(" ");
				serialized.Append(Mean[i].ToString("F6"));
			}

			serialized.Append(";");
			
			serialized.Append(Covariance[0][0].ToString("F6"));
			
			for (int k = 1; k < Covariance[0].Length; k++) {
				serialized.Append(" ");
				serialized.Append(Covariance[0][k].ToString("F6"));
			}

			for (int i = 1; i < Covariance   .Length; i++) {
			for (int k = 0; k < Covariance[i].Length; k++) {
				serialized.Append(" ");
				serialized.Append(Covariance[i][k].ToString("F6"));
			}
			}

			return serialized.ToString();
		}
	}
}
}
