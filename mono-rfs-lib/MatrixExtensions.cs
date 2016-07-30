// MatrixExtensions.cs
// Utility extensions to matrix class (double[][])
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

using Accord.Math;

using Microsoft.Xna.Framework;
using Accord.Math.Decompositions;

namespace monorfs
{
/// <summary>
/// Utility extensions to the matrix and vector classes (double[][] and double[])
/// </summary>
public static class MatrixExtensions
{
	public readonly static Vector3 UpVector = new Vector3(0, 0, 1);

	/// <summary>
	/// Reverse the order of the columns of a matrix.
	/// </summary>
	/// <param name="matrix">Original matrix.</param>
	/// <returns>Reversed matrix.</returns>
	public static double[][] ReverseColumns(this double[][] matrix)
	{
		double[][] reversed = new double[matrix.Length][];

		for (int i = 0; i < reversed.Length; i++) {
			reversed[i] = new double[matrix[i].Length];
		}
		
		for (int i = 0; i < reversed   .Length; i++) {
		for (int k = 0; k < reversed[i].Length; k++) {
			reversed[i][matrix[i].Length - k - 1] = matrix[i][k];
		}
		}

		return reversed;
	}

	/// <summary>
	/// Transform a 3D point using an homography matrix.
	/// </summary>
	/// <returns>3D transformed point in cartesian coordinates.</returns>
	/// <param name="point">3D point in cartesian coordinates.</param>
	/// <param name="transformation">Homography 4x4 transformation matrix.</param>
	public static double[] TransformH(this double[][] transformation, double[] point)
	{
		double[] homographic = transformation.Multiply(point.Concatenate(1));
		return new double[3] {homographic[0] / homographic[3],
		                      homographic[1] / homographic[3],
		                      homographic[2] / homographic[3]};
	}

	/// <summary>
	/// Create a look-at camera from its spherical coordinates
	/// relative to the target.
	/// </summary>
	/// <param name="target">Target position.</param>
	/// <param name="ground">Ground angle, in [0, 2pi].</param>
	/// <param name="elevation">Elevation angle, in [-pi/2, pi/2].</param>
	/// <param name="zoom">Zoom factor.</param>
	/// <returns>Camera matrix.</returns>
	public static double[][] AngleDistanceCamera(double[] target, double ground, double elevation, double zoom)
	{
		double[][] camera;

		camera       = CreateTranslation(new double[3] {-target[0], -target[1], -target[2]});
		camera       = CreateRotationY(ground).Multiply(camera);
		camera       = CreateRotationX(elevation).Multiply(camera);
		camera[3][3] = zoom;

		return camera;
	}

	/// <summary>
	/// Creates a 4x4 transformation matrix for translating every point
	/// in the same amount.
	/// </summary>
	/// <param name="target">New zero position.</param>
	/// <returns>Transformation matrix.</returns>
	public static double[][] CreateTranslation(double[] target)
	{
		return new double[4][] {new double[4] {1, 0, 0, target[0]},
		                        new double[4] {0, 1, 0, target[1]},
		                        new double[4] {0, 0, 1, target[2]},
		                        new double[4] {0, 0, 0, 1}};
	}

	/// <summary>
	/// Creates a 4x4 rotation matrix for rotating in the yz plane
	/// with a given angle.
	/// </summary>
	/// <param name="angle">Angle measured in radians, counterclockwise starting from (0, 0, 1).</param>
	/// <returns>Transformation matrix.</returns>
	public static double[][] CreateRotationX(double angle)
	{
		double ct = Math.Cos(angle);
		double st = Math.Sin(angle);

		return new double[4][] {new double[4] {1,  0,   0, 0},
		                        new double[4] {0, ct, -st, 0},
		                        new double[4] {0, st,  ct, 0},
		                        new double[4] {0,  0,   0, 1}};
	}

	/// <summary>
	/// Creates a 4x4 rotation matrix for rotating in the xz plane
	/// with a given angle.
	/// </summary>
	/// <param name="angle">Angle measured in radians, counterclockwise starting from (1, 0, 0).</param>
	/// <returns>Transformation matrix.</returns>
	public static double[][] CreateRotationY(double angle)
	{
		double ct = Math.Cos(angle);
		double st = Math.Sin(angle);
		
		return new double[4][] {new double[4] { ct, 0, st, 0},
		                        new double[4] {  0, 1,  0, 0},
		                        new double[4] {-st, 0, ct, 0},
		                        new double[4] {  0, 0,  0, 1}};
		}

	/// <summary>
	/// Creates a 4x4 rotation matrix for rotating in the xy plane
	/// with a given angle.
	/// </summary>
	/// <param name="angle">Angle measured in radians, counterclockwise starting from (0, 1, 0).</param>
	/// <returns>Transformation matrix.</returns>
	public static double[][] CreateRotationZ(double angle)
	{
		double ct = Math.Cos(angle);
		double st = Math.Sin(angle);

		return new double[4][] {new double[4] {ct, -st, 0, 0},
		                        new double[4] {st,  ct, 0, 0},
		                        new double[4] { 0,   0, 1, 0},
		                        new double[4] { 0,   0, 0, 1}};
	}

	/// <summary>
	/// Convert a 3D vector from a double array representation to a Vector3 object.
	/// </summary>
	/// <param name="x">Double array representation.</param>
	/// <returns>Vector3 object.</returns>
	public static Vector3 ToVector3(this double[] x)
	{
		return new Vector3((float) x[0], (float) x[1], (float) x[2]);
	}

	/// <summary>
	/// Transform a MonoGame matrix to a double jagged array.
	/// </summary>
	/// <param name="matrix">MonoGame matrix.</param>
	/// <returns>Jagged array.</returns>
	public static double[][] ToJagged(this Microsoft.Xna.Framework.Matrix matrix)
	{
		return new double[4][] { new double[4] {matrix.M11, matrix.M12, matrix.M13, matrix.M14},
		                         new double[4] {matrix.M21, matrix.M22, matrix.M23, matrix.M24},
		                         new double[4] {matrix.M31, matrix.M32, matrix.M33, matrix.M34},
		                         new double[4] {matrix.M41, matrix.M42, matrix.M43, matrix.M44} };
	}

	/// <summary>
	/// Transform a double jagged array to a MonoGame matrix.
	/// </summary>
	/// <param name="matrix">Jagged array.</param>
	/// <returns>MonoGame matrix.</returns>
	public static Microsoft.Xna.Framework.Matrix ToXnaMatrix(this double[][] matrix)
	{
		return new Microsoft.Xna.Framework.Matrix((float) matrix[0][0], (float) matrix[0][1], (float) matrix[0][2], (float) matrix[0][3],
		                                          (float) matrix[1][0], (float) matrix[1][1], (float) matrix[1][2], (float) matrix[1][3],
		                                          (float) matrix[2][0], (float) matrix[2][1], (float) matrix[2][2], (float) matrix[2][3],
		                                          (float) matrix[3][0], (float) matrix[3][1], (float) matrix[3][2], (float) matrix[3][3]);
	}

	/// <summary>
	/// Calculate the transpose of a vector (a horizontal matrix).
	/// </summary>
	/// <param name="vector">Vector to be transposed.</param>
	/// <param name="shallow">If true, use the same vector for the matrix,
	/// but changing values in one will be reflected on the other.</param>
	/// <returns>Transposed vector.</returns>
	public static double[][] Transpose(this double[] vector, bool shallow = false)
	{
		double[][] result = new double[1][];

		if (shallow) {
			result[0] = vector;
		}
		else {
			result[0] = Util.SClone(vector);
		}

		return result;
	}

	/// <summary>
	/// Multiply a scalar coefficient with a matrix (element-wise), possibly in-place.
	/// </summary>
	/// <param name="coeff">Scalar coefficient.</param>
	/// <param name="matrix">Original matrix.</param>
	/// <param name="inplace">Don't create a new matrix, use the original one. Lower memory footprint but breaks
	/// immutability (useful for thread-safety).</param>
	/// <returns>Scaled matrix.</returns>
	public static double[][] Multiply(this double coeff, double[][] matrix, bool inplace = false)
	{
		double[][] result;

		if (inplace) {
			result = matrix;
		}
		else {
			result = new double[matrix.Length][];

			for (int i = 0; i < result.Length; i++) {
				result[i] = new double[matrix[i].Length];
			}
		}

		for (int i = 0; i < matrix   .Length; i++) {
		for (int k = 0; k < matrix[i].Length; k++) {
			result[i][k] = coeff * matrix[i][k];
		}
		}

		return result;
	}

	/// <summary>
	/// Divide a matrix by a scalar coefficient (element-wise), possibly in-place.
	/// </summary>
	/// <param name="coeff">Scalar coefficient.</param>
	/// <param name="matrix">Original matrix.</param>
	/// <param name="inplace">Don't create a new matrix, use the original one. Lower memory footprint but breaks
	/// immutability (useful for thread-safety).</param>
	/// <returns>Scaled matrix.</returns>
	public static double[][] Divide(this double[][] matrix, double coeff, bool inplace = false)
	{
		return (1/coeff).Multiply(matrix, inplace);
	}

	/// <summary>
	/// Multiply a jagged matrix by the transpose of another.
	/// </summary>
	/// <remarks>Jagged version of the Accord.Net function.</remarks>
	/// <param name="a">First matrix.</param>
	/// <param name="b">Second matrix.</param>
	/// <returns>Multiplication result.</returns>
	public static double[][] MultiplyByTranspose(this double[][] a, double[][] b)
	{
		return a.Multiply(b.Transpose());
	}

	/// <summary>
	/// Calculate the determinant of a jagged matrix.
	/// </summary>
	/// <remarks>Jagged version of the Accord.Net function.</remarks>
	/// <param name="matrix">Original matrix.</param>
	/// <returns>Matrix determinant.</returns>
	public static double Determinant(this double[][] matrix)
	{
		return matrix.ToMatrix().Determinant();
	}

	/// <summary>
	/// Invert a matrix and try to deal with singular matrices by
	/// introducing a little offset; since singular matrices live in
	/// a very thin subspace, the offset will almost surely make the
	/// matrix invertible. Naturally, this matrix may be ill-conditioned
	/// so the inverse may be inaccurate. Expect to have large components,
	/// since the zero eigenvalues would give infinite inverse.
	/// </summary>
	/// <remarks>
	/// The matrix must be square for this method to apply.
	/// This method is in a way the complement of PseudoInverse(),
	/// since it tries to explode infinite dimensions with "big" numbers,
	/// instead of passing them as zero.
	/// </remarks>
	/// <param name="matrix">Matrix to invert.</param>
	/// <returns>Matrix inverse.</returns>
	public static double[][] SingularInverse(this double[][] matrix)
	{
		const double eps = 1e-10;
		double[][] variation = matrix.MemberwiseClone();

		if (variation.Length == 0 || variation.Length != variation[0].Length) {
			throw new FormatException("Input matrix must be square");
		}

		for (int i = 0; i < variation.Length; i++) {
			variation[i][i] += eps;
		}

		return variation.Inverse();
	}

	/// <summary>
	/// Force every entry in the vector to be in the given range.
	/// </summary>
	/// <param name="vector">Vector to clamp.</param>
	/// <param name="min">Minimum value for output.</param>
	/// <param name="max">Maximum value for output.</param>
	/// <returns>Clamped vector.</returns>
	public static double[] Clamp(this double[] vector, double min, double max)
	{
		double[] clamped = new double[vector.Length];
		
		for (int i = 0; i <clamped.Length; i++) {
			clamped[i] = Math.Max(min, Math.Min(max, vector[i]));
		}

		return clamped;
	}

	/// <summary>
	/// Calculate log(sum(exp(vector))) as numerically stable as possible.
	/// This corresponds to a soft version of the max() operation.
	/// </summary>
	/// <param name="vector">Array of entries.</param>
	/// <param name="begin">First index to consider.</param>
	/// <param name="end">Last+1 index to consider.</param>
	/// <returns>Log-sum-exp value.</returns>
	public static double LogSumExp(this double[] vector, int begin, int end)
	{
		double max   = double.NegativeInfinity;
		double value = 0;

		if (begin < 0) {
			throw new ArgumentException("Begin index must be greater or equal to zero");
		}

		if (end > vector.Length) {
			throw new ArgumentException("End index must be less or equal to the vector size");
		}

		for (int i = begin; i < end; i++) {
			max = Math.Max(max, vector[i]);
		}

		if (double.IsNegativeInfinity(max)) {
			return double.NegativeInfinity;
		}

		for (int i = begin; i < end; i++) {
			value += Math.Exp(vector[i] - max);
		}

		value = max + Math.Log(value);

		return value;
	}

	/// <summary>
	/// Calculate an weighted average of the form Sum(Exp(weight) * vector) / Sum(Exp(weight))
	/// as numerically stable as possible.
	/// </summary>
	/// <param name="vectors">Array of vectors.</param>
	/// <param name="weights">Log-weights for the average.</param>
	/// <param name="begin">First index to consider.</param>
	/// <param name="end">Last+1 index to consider.</param>
	/// <returns>Tempered average of the vectors.</returns>
	public static double[] TemperedAverage(this double[][] vectors, double[] weights, int begin, int end)
	{
		if (vectors.Length != weights.Length) {
			throw new ArgumentException("There must be exactly one weight per vector");
		}

		if (begin < 0) {
			throw new ArgumentException("Begin index must be greater or equal to zero");
		}

		if (end > vectors.Length) {
			throw new ArgumentException("End index must be less or equal to the vector size");
		}

		if (vectors.Length == 0) {
			return new double[0];
		}

		double   max   = double.NegativeInfinity;
		double[] value = new double[vectors[0].Length];

		for (int i = begin; i < end; i++) {
			max = Math.Max(max, weights[i]);
		}

		if (double.IsNegativeInfinity(max)) {
			return new double[6];
		}

		for (int i = begin; i < end; i++) {
			weights[i] = Math.Exp(weights[i] - max);
		}

		weights = weights.Normalize();

		for (int i = begin; i < end; i++) {
			value = value.Add(weights[i].Multiply(vectors[i]));
		}

		return value;
	}

	/// <summary>
	/// Concatenate two matrices vertically.
	/// </summary>
	/// <param name="first">First matrix.</param>
	/// <param name="second">Second matrix.</param>
	/// <returns>Concatenated matrix.</returns>
	public static double[][] VConcatenate(this double[][] first, double[][] second)
	{
		if (first.Columns() != second.Columns()) {
			throw new ArgumentException("matrices must have the same number of columns.");
		}
		
		double[][] concat = Accord.Math.Matrix.Jagged<double>(first.Rows() + second.Rows(), first.Columns(), 0);

		for (int i = 0; i < first.Rows(); i++) {
		for (int k = 0; k < first.Columns(); k++) {
			concat[i][k] = first[i][k];
		}
		}
		for (int i = 0, h = first.Rows(); i < second.Rows(); i++, h++) {
		for (int k = 0; k < second.Columns(); k++) {
			concat[h][k] = second[i][k];
		}
		}

		return concat;
	}

	/// <summary>
	/// Create a new square matrix of a given size filled with zeros.
	/// </summary>
	/// <param name="size">Number of rows.</param>
	public static double[][] Zero(int size)
	{
		double[][] zero = new double[size][];

		for (int i = 0; i < size; i++) {
			zero[i] = new double[size];
		}

		return zero;
	}

	/// <summary>
	/// Compare two vectors by relatively comparing each entry.
	/// </summary>
	/// <param name="a">First matrix.</param>
	/// <param name="b">Second matrix.</param>
	/// <param name="threshold">Acceptable relative difference between each entry (fraction of the average vector)./param>
	/// <param name="zerothreshold">Acceptable absolute difference between each entry in the case of zero average./param>
	/// <returns>True if both vectors are equal.</returns>
	public static bool IsEqualRelative(this double[] a, double[] b, double threshold, double zerothreshold=1e-5)
	{
		if (a.Length != b.Length) {
			return false;
		}

		for (int i = 0; i < a.Length; i++) {
			double diff     = Math.Abs( a[i] - b[i]);
			double avg      = Math.Abs((a[i] + b[i]) / 2);
			double relative = diff / avg;
			if (!Double.IsNaN(relative)) {
				if (avg > zerothreshold) {
					if (relative > threshold) {
						return false;
					}
				}
				else {
					if (diff > zerothreshold) {
						return false;
					}
				}
			}
		}

		return true;
	}

	/// <summary>
	/// Compare two matrices by relatively comparing each entry.
	/// </summary>
	/// <param name="a">First matrix.</param>
	/// <param name="b">Second matrix.</param>
	/// <param name="threshold">Acceptable relative difference between each entry (fraction of the average matrix)./param>
	/// <param name="zerothreshold">Acceptable absolute difference between each entry in the case of zero average./param>
	/// <returns>True if both matrices are equal.</returns>
	public static bool IsEqualRelative(this double[][] a, double[][] b, double threshold, double zerothreshold=1e-5)
	{
		if (a.Length != b.Length) {
			return false;
		}

		for (int i = 0; i < a   .Length; i++) {
		for (int k = 0; k < a[i].Length; k++) {
			double diff     = Math.Abs( a[i][k] - b[i][k]);
			double avg      = Math.Abs((a[i][k] + b[i][k]) / 2);
			double relative = diff / avg;
			if (!Double.IsNaN(relative)) {
				if (avg > zerothreshold) {
					if (relative > threshold) {
						return false;
					}
				}
				else {
					if (diff > zerothreshold) {
						return false;
					}
				}
			}
		}
		}

		return true;
	}

	/// <summary>
	/// Compare two matrices by relatively comparing each entry.
	/// </summary>
	/// <param name="a">First matrix.</param>
	/// <param name="b">Second matrix.</param>
	/// <param name="threshold">Acceptable relative difference between each entry (fraction of the average matrix)./param>
	/// <param name="absthreshold">Acceptable absolute difference between each entry in the case of zero average./param>
	/// <returns>True if both matrices are equal.</returns>
	public static bool IsEqualRelative(this double[,] a, double[,] b, double threshold, double absthreshold=1e-10)
	{
		return a.ToArray().IsEqualRelative(b.ToArray(), threshold, absthreshold);
	}

	/// <summary>
	/// Compare two matrices by comparing each entry in their SVD decomposition.
	/// </summary>
	/// <param name="a">First matrix.</param>
	/// <param name="b">Second matrix.</param>
	/// <param name="threshold">Acceptable relative difference between each entry (fraction of the first matrix)./param>
	/// <returns>True if both matrices are equal.</returns>
	public static bool SVDEquals(this double[][] a, double[][] b, double threshold)
	{
		var adecomp = new SingularValueDecomposition(a.ToMatrix());
		var bdecomp = new SingularValueDecomposition(b.ToMatrix());

		return adecomp.Diagonal.IsEqualRelative(bdecomp.Diagonal, threshold) &&
		       adecomp.LeftSingularVectors .IsEqualRelative(bdecomp.LeftSingularVectors,  threshold) &&
		       adecomp.RightSingularVectors.IsEqualRelative(bdecomp.RightSingularVectors, threshold);
	}
}
}
