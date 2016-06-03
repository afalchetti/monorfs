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
	/// Get the matrix representation of a rotation quaternion.
	/// </summary>
	/// <param name="q">Rtation quaternion.</param>
	/// <returns>Matrix representation.</returns>
	public static double[][] Quat2Matrix(Quaternion q)
	{
		return new double[3][]
		{
			new double[3] {q.W*q.W + q.X*q.X - q.Y*q.Y - q.Z*q.Z, 2 * (q.X*q.Y - q.W*q.Z),               2 * (q.W*q.Y + q.X*q.Z)},
			new double[3] {2 * (q.X*q.Y + q.W*q.Z),               q.W*q.W - q.X*q.X + q.Y*q.Y - q.Z*q.Z, 2 * (q.Y*q.Z - q.W*q.X)},
			new double[3] {2 * (q.X*q.Z - q.W*q.Y),               2 * (q.W*q.X + q.Y*q.Z),               q.W*q.W - q.X*q.X - q.Y*q.Y + q.Z*q.Z}
		};
	}

	/// <summary>
	/// Transform a quaternion-based pose into its Lie algebra representation around
	/// the given linearization point, i.e. linearpoint's rotation becomes the identity.
	/// </summary>
	/// <param name="pose">Quaternion-based state with an associated covariance.</param>
	/// <param name="linearpoint">Linearization point.</param>
	/// <returns>Lie-based state with associated covariance.</returns>
	public static Gaussian Quat2Lie(Gaussian pose, Pose3D linearpoint)
	{
		double[]   relpose      = new Pose3D(pose.Mean).Subtract(linearpoint);
		Quaternion linrotation  = linearpoint.Orientation;
		Quaternion poserotation = new Pose3D().FromLinear(relpose).Orientation;
		double[][] rj           = Q2LieJacobian(linrotation);

		double[][] jacobian = new double[6][] { new double[7] {1, 0, 0, 0,        0,        0,        0},
		                                        new double[7] {0, 1, 0, 0,        0,        0,        0},
		                                        new double[7] {0, 0, 1, 0,        0,        0,        0},
		                                        new double[7] {0, 0, 0, rj[0][0], rj[0][1], rj[0][2], rj[0][3]},
		                                        new double[7] {0, 0, 0, rj[1][0], rj[1][1], rj[1][2], rj[1][3]},
		                                        new double[7] {0, 0, 0, rj[2][0], rj[2][1], rj[2][2], rj[2][3]}};

		double[][] covariance = jacobian.Multiply(pose.Covariance).MultiplyByTranspose(jacobian);

		return new Gaussian(relpose, covariance, 1.0);
	}

	/// <summary>
	/// Transform a Lie-based pose into its quaternion representation around
	/// the given linearization point, i.e. linearpoint's rotation becomes the identity.
	/// </summary>
	/// <param name="pose">Lie-based state with an associated covariance.</param>
	/// <param name="linearpoint">Linearization point.</param>
	/// <returns>Quaternion-based state with associated covariance.</returns>
//	public static double[] Lie2Quat(Pose3D pose, Pose3D linearpoint)
//	{
//		return pose.Subtract(linearpoint);
//
//		Quaternion linrotation  = new Quaternion(linearpoint[3], linearpoint[4], linearpoint[5], linearpoint[6]);
//		double[]   poserotation = new double[] {pose[3], pose[4], pose[5]};
//		Quaternion quatmean     = linrotation.Add(poserotation);
//
//		return new double[7] {pose[0], pose[1], pose[2], quatmean.W, quatmean.X, quatmean.Y, quatmean.Z};
//	}

	/// <summary>
	/// Transform a Lie-based pose into its quaternion representation around
	/// the given linearization point, i.e. linearpoint's rotation becomes the identity.
	/// </summary>
	/// <param name="pose">Lie-based state with an associated covariance.</param>
	/// <param name="linearpoint">Linearization point.</param>
	/// <returns>Quaternion-based state with associated covariance.</returns>
	public static Gaussian Lie2Quat(Gaussian pose, Pose3D linearpoint)
	{
		Pose3D     globpose    = linearpoint.Add(pose.Mean);
		Quaternion linrotation = linearpoint.Orientation;
//		double[]   poserotation = new double[] {pose.Mean[3], pose.Mean[4], pose.Mean[5]};
//
//		Quaternion quatmean = linrotation.Add(poserotation);
//		double[]   mean     = new double[7] {pose.Mean[0], pose.Mean[1], pose.Mean[2], quatmean.W, quatmean.X, quatmean.Y, quatmean.Z};

		double[][] rj      = Q2LieJacobian(linrotation).PseudoInverse();

		double[][] jacobian = new double[7][] { new double[6] {1, 0, 0, 0,        0,        0},
		                                        new double[6] {0, 1, 0, 0,        0,        0},
		                                        new double[6] {0, 0, 1, 0,        0,        0},
		                                        new double[6] {0, 0, 0, rj[0][0], rj[0][1], rj[0][2]},
		                                        new double[6] {0, 0, 0, rj[1][0], rj[1][1], rj[1][2]},
		                                        new double[6] {0, 0, 0, rj[2][0], rj[2][1], rj[2][2]},
		                                        new double[6] {0, 0, 0, rj[3][0], rj[3][1], rj[3][2]}};

		double[][] covariance = jacobian.Multiply(pose.Covariance).MultiplyByTranspose(jacobian);

		return new Gaussian(globpose.State, covariance, 1.0);
	}

	/// <summary>
	/// Calculate the jacobian of the yaw-pitch-roll to quaternion function.
	/// </summary>
	/// <param name="yaw">Yaw evaluation point.</param>
	/// <param name="pitch">Pitch evaluation point.</param>
	/// <param name="roll">Roll evaluation point.</param>
	/// <returns>Jacobian.</returns>
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
	/// Calculate the jacobian of the quaternion to lie algebra function.
	/// </summary>
	/// <param name="q">quaternion evaluation point.</param>
	/// <returns>Jacobian.</returns>
	public static double[][] Q2LieJacobian(Quaternion q)
	{
		q = q.Normalize();

		double mag  = Math.Sqrt(1-q.W*q.W);
		double mag2 = mag * mag;

		double[][] wjacobian = null;
		double alpha = 0;

		if (mag < 1e-6) {
			wjacobian = new double[3][] { new double[1] {0},
			                              new double[1] {0},
			                              new double[1] {0}};
			alpha = 2;
		}
		else {
			wjacobian = new double[3][] { new double[1] {-2 * q.X / mag2},
			                              new double[1] {-2 * q.Y / mag2},
			                              new double[1] {-2 * q.Z / mag2}};

			alpha = 2 * Math.Acos(q.W) / mag;
		}

		double nx = 0;
		double ny = 0;
		double nz = 0;

		if (mag > 1e-15) {
			nx = q.X / mag;
			ny = q.Y / mag;
			nz = q.Z / mag;
		}
		

		double[][] xyzjacobian = new double[3][] {new double[3] {-nx * nx, -ny * nx, -nz * nx},
		                                          new double[3] {-nx * ny, -ny * ny, -nz * ny},
		                                          new double[3] {-nx * nz, -ny * nz, -nz * nz}};

		xyzjacobian.AddToDiagonal(1.0, true);
		xyzjacobian = alpha.Multiply(xyzjacobian);

		return wjacobian.Concatenate(xyzjacobian);
	}

	/// <summary>
	/// Calculate alternative motion covariances: from the angle (pitch-yaw-roll)
	/// covariance, calculate the Quaternion and Lie algebra ones.
	/// </summary>
	/// <param name="angle">Pitch-yaw-roll covariance.</param>
	/// <param name="quaternion">Quaternion covariance.</param>
	/// <param name="lie">Lie algebra covariance.</param>
	public static void GetMotionCovariances(double[][] angle, out double[][] quaternion, out double[][] lie)
	{
		double[][] rq = YPR2QJacobian(0, 0, 0);
		double[][] rl;

		// rl = Q2LieJacobian(new Quaternion(0, 0, 0, 1));
		//
		// // though incorrect (it's -inf), this avoids inf * 0
		// // on the calculation of the lie jacobian; taking the limit
		// // shows that using zero gives the correct behavior
		// rl[0][0] = 0;
		// rl[1][0] = 0;
		// rl[2][0] = 0;
		// 
		// rl = rl.Multiply(rq);

		// NOTE since quaternions are ill-defined outside the unitary
		// manifold, the lie jacobian is not so direct to define and interpret
		// (a straightforward use of derivatives leads to a nonreparable singularity
		// that changes value depending on the direction it is approached);
		// the previous computation, which handles the zero case exceptionally,
		// returns a permutation of the identity; the lie covariance is then the
		// same than the angle covariance, which at least seems intuitive.
		// anyway, the whole calculation is bypassed since the result is trivial:
		rl = new double[3][] { new double[3] {0, 1, 0},
		                       new double[3] {1, 0, 0},
		                       new double[3] {0, 0, 1} };

		double[][] qjacobian = new double[7][] { new double[6] {1, 0, 0, 0,        0,        0},
		                                         new double[6] {0, 1, 0, 0,        0,        0},
		                                         new double[6] {0, 0, 1, 0,        0,        0},
		                                         new double[6] {0, 0, 0, rq[0][0], rq[0][1], rq[0][2]},
		                                         new double[6] {0, 0, 0, rq[1][0], rq[1][1], rq[1][2]},
		                                         new double[6] {0, 0, 0, rq[2][0], rq[2][1], rq[2][2]},
		                                         new double[6] {0, 0, 0, rq[3][0], rq[3][1], rq[3][2]} };
		
		double[][] ljacobian = new double[6][] { new double[6] {1, 0, 0, 0,        0,        0},
		                                         new double[6] {0, 1, 0, 0,        0,        0},
		                                         new double[6] {0, 0, 1, 0,        0,        0},
		                                         new double[6] {0, 0, 0, rl[0][0], rl[0][1], rl[0][2]},
		                                         new double[6] {0, 0, 0, rl[1][0], rl[1][1], rl[1][2]},
		                                         new double[6] {0, 0, 0, rl[2][0], rl[2][1], rl[2][2]} };

		quaternion = qjacobian.Multiply(angle).MultiplyByTranspose(qjacobian);
		lie        = ljacobian.Multiply(angle).MultiplyByTranspose(ljacobian);

		// regularization: if variance is zero in some component the matrix probably will be
		// semi-definite instead of definite positive and some algorithms will not work properly;
		// add a tiny amount of variance in such case
		for (int i = 0; i < quaternion.Length; i++) {
			if (quaternion[i][i] < 1e-9) {
				quaternion[i][i] = 1e-9;
			}
		}

		for (int i = 0; i < lie.Length; i++) {
			if (lie[i][i] < 1e-9) {
				lie[i][i] = 1e-9;
			}
		}
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
			cov[i][i] = 1e20;
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
