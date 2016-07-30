// Util.cs
// Utility definitions
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
using System.IO;
using System.Text.RegularExpressions;

using AForge;
using Accord.Math;
using AForge.Math.Random;
using Accord.Math.Decompositions;

namespace monorfs
{
/// <summary>
/// Utility definitions.
/// </summary>
public static class Util
{
	/// <summary>
	/// Gaussian random generator.
	/// </summary>
	public static GaussianGenerator Gaussian;

	/// <summary>
	/// Global uniform random generator.
	/// </summary>
	public static UniformGenerator Uniform;

	/// <summary>
	/// Static generator initialization.
	/// </summary>
	static Util()
	{
		Gaussian = new GaussianGenerator(0, 1);
		Uniform  = new UniformGenerator(new Range(0, 1));
	}

	/// <summary>
	/// Normalize an angle to the range [-pi, pi)
	/// </summary>
	/// <param name="angle">Unnormalized angle.</param>
	/// <returns>Normalized angle.</returns>
	public static double NormalizeAngle(double angle)
	{
		// normal will be normalized to [0, 2pi), 
		// so add pi to undo the later "normal - pi"
		double normal = (angle + Math.PI) % (2 * Math.PI);
		normal = (normal >= 0) ? normal : normal + 2 * Math.PI;

		return normal - Math.PI;
	}

	/// <summary>
	/// Normalize an angle to the range [0, 2pi)
	/// </summary>
	/// <param name="angle">Unnormalized angle.</param>
	/// <returns>Normalized angle.</returns>
	public static double NormalizeAngle2(double angle)
	{
		double normal = angle % (2 * Math.PI);

		return (normal >= 0) ? normal : normal + 2 * Math.PI;
	}

	/// <summary>
	/// Get the cross-product matrix associated to a vector.
	/// </summary>
	/// <param name="vector">Associated vector.</param>
	/// <returns>Cross-product matrix. Linear transfromation equivalent to
	/// calculate the cross product with the vector</returns>
	public static double[][] CrossProductMatrix(double[] vector)
	{
		return new double[3][] { new double[3] {         0, -vector[2],  vector[1]},
		                         new double[3] { vector[2],          0, -vector[0]},
		                         new double[3] {-vector[1],  vector[0],          0} };
	}

	/// <summary>
	/// Generate a (quasi-)Dirac delta distribution.
	/// A tiny covariance is used to avoid numerical errors.
	/// </summary>
	/// <param name="x0">Position for the delta.</param>
	/// <returns>Dirac distribution.</returns>
	public static Gaussian DiracDelta(double[] x0)
	{
		const double eps        = 1e-12;
		double[][]   covariance = new double[x0.Length][];

		for (int i = 0; i < covariance.Length; i++) {
			covariance[i]    = new double[x0.Length];
			covariance[i][i] = eps;
		}

		return new Gaussian(x0, covariance, 1.0);
	}

	/// <summary>
	/// Generate an uninformative covariance.
	/// </summary>
	/// <param name="size">Space dimension.</param>
	/// <returns>Covariance.</returns>
	public static double[][] InfiniteCovariance(int size)
	{
		double[][] cov = new double[size][];

		for (int i = 0; i < size; i++) {
			cov[i]    = new double[size];
			cov[i][i] = 1e12;
		}

		return cov;
	}

	/// <summary>
	/// Maps the [0, 1] range to itself from a linear interpolation value
	/// to a smoother interpolation value that has softer derivative changes along the interval.
	/// </summary>
	/// <param name="x">Linear transition.</param>
	/// <returns>Smooth value.</returns>
	public static double SmoothTransition(double x)
	{
		if (x < 0.5) {
			return 2 * x * x;
		}
		else {
			return 1 - 2 * (x - 1) * (x - 1);
		}
	}

	/// <summary>
	/// Realize a random vector from a gaussian distribution around a specified mean
	/// vector with a given covvariance.
	/// </summary>
	/// <param name="mean">Distribution mean vector.</param>
	/// <param name="covariance">Distribution covariance matrix.</param>
	/// <returns>Random vector.</returns>
	public static double[] RandomGaussianVector(double[] mean, double[][] covariance)
	{
		// first find the square root of the covariance matrix
		// C such as C * C^T = covariance
		var       cholesky = new CholeskyDecomposition(covariance.ToMatrix());
		double[]  sqrtdiag = cholesky.Diagonal;
		double[,] covroot;

		for (int i = 0; i < sqrtdiag.Length; i++) {
			sqrtdiag[i] = Math.Sqrt(sqrtdiag[i]);
		}

		covroot = cholesky.LeftTriangularFactor.MultiplyByDiagonal(sqrtdiag);

		// and then use the random variable Y = mean + C * Z
		// with Z an independent canonical gaussian random vector ~N(0, 1)
		// to obtain the multinormal correlated  random vector
		double[]  canonical = new double[mean.Length];
		for (int i = 0; i < canonical.Length; i++) {
			canonical[i] = Gaussian.Next();
		}

		return mean.Add(covroot.Multiply(canonical));
	}

	/// <summary>
	/// Seed the internal Accord random generator.
	/// </summary>
	public static void SeedGenerators(int seed)
	{
		Gaussian.SetSeed(seed);
		Uniform.SetSeed(seed);
	}

	/// <summary>
	/// Typesafe shallow clone of an array.
	/// </summary>
	/// <returns>Shallow clone.</returns>
	/// <param name="array">Array.</param>
	/// <typeparam name="T">Elements' type.</typeparam>
	public static T[] SClone<T>(T[] array)
	{
		return (T[]) array.Clone();
	}

	/// <summary>
	/// Get a dictionary from a tabular string descriptor.
	/// All entries have 'string' type. The entry separator is a newline.
	/// Keys must not have any whitespace on their line and values associated
	/// to a key are marked by a tab at the biggining of their line.
	/// </summary>
	/// <param name="descriptor">String representing the dictionary. Separated by newlines and tabs.</param>
	/// <returns>The dictionary.</returns>
	public static Dictionary<string, List<string>> ParseDictionary(string descriptor)
	{
		var dictionary = new Dictionary<string, List<string>>();
		descriptor     = descriptor.Replace("\r\n", "\n").Replace('\r', '\n');
        string[] lines = descriptor.Split('\n');
		string   key   = "";

		Regex emptyline = new Regex(@"^\s*$");

		if (lines.Length > 0 && !Regex.Match(lines[0], @"^\S+").Success) {
			// malformed input, can't start with a child node
			return dictionary;
		}

		foreach (string line in lines) {
			if (emptyline.Match(line).Success) {
				continue;
			}

			if (line[0] != '\t') {
				// the line is a key
				key = line;
				dictionary.Add(key, new List<string>());
			}
			else {
				// the line is a child
				// add it without the leading tab
				dictionary[key].Add(line.Substring(1));
			}
		}

		return dictionary;
	}

	/// <summary>
	/// Create a temporary directory.
	/// Unless maliciously attacked or extremely bad luck
	/// (extreme data race + 57bits randomness), the directory
	/// will be unique - sadly, no mkdtemp in Windows|.Net.
	/// </summary>
	/// <returns>Directory path.</returns>
	public static string TemporaryDir()
	{
		string dir = "";

		// after 5 tries, just use the folder, even if it already exists
		for (int i = 0; i < 5; i++) {
			dir = Path.Combine(Path.GetTempPath(), Path.GetRandomFileName());

			if (!Directory.Exists(dir)) {
				break;
			}
		}

		Directory.CreateDirectory(dir);

		return dir;
	}
}
}
