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

using Accord;
using Accord.Math;
using AForge.Math.Random;
using Accord.Math.Decompositions;
using AForge;

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
	public static GaussianGenerator gaussian;

	/// <summary>
	/// Global uniform random generator.
	/// </summary>
	public static UniformGenerator uniform;

	/// <summary>
	/// Static generator initialization.
	/// </summary>
	static Util()
	{
		gaussian = new GaussianGenerator(0, 1);
		uniform  = new UniformGenerator(new Range(0, 1));
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
	/// Calculate the jacobian of the yaw-pitch-roll to quaternion function.
	/// </summary>
	/// <returns>Jacobian.</returns>
	/// <param name="yaw">Yaw evaluation point.</param>
	/// <param name="pitch">Pitch evaluation point.</param>
	/// <param name="roll">Roll evaluation point.</param>
	public static double[][] YPR2QJacobian(double yaw, double pitch, double roll)
	{
		double r2 = 0.5 * roll;
		double p2 = 0.5 * pitch;
		double y2 = 0.5 * yaw;

		double sr = Math.Sin(r2);
		double cr = Math.Cos(r2);
		double sp = Math.Sin(p2);
		double cp = Math.Cos(p2);
		double sy = Math.Sin(y2);
		double cy = Math.Cos(y2);

		double wc =  0.5 * cy * cp * cr;
		double ws =  0.5 * sy * sp * sr;

		double xc =  0.5 * cy * sp * cr;
		double xs =  0.5 * sy * cp * sr;

		double yc =  0.5 * sy * cp * cr;
		double ys = -0.5 * cy * sp * sr;

		double zc =  0.5 * cy * cp * sr;
		double zs = -0.5 * sy * sp * cr;

		double w = wc + ws;
		double x = xc + xs;
		double y = yc + ys;
		double z = zc + zs;

		double a = wc - ws;
		double b = xc - xs;
		double c = yc - ys;
		double d = zc - zs;

		return new double[4][] { new double[3] {-y, -b, -z},
		                         new double[3] { z,  a,  y},
		                         new double[3] { w, -d, -x},
		                         new double[3] {-x, -c,  w} };
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
			canonical[i] = gaussian.Next();
		}

		return mean.Add(covroot.Multiply(canonical));
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
