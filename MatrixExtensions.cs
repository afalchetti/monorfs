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
	/// Obtain the corresponding rotation matriz from a quaternion.
	/// </summary>
	/// <param name="q">Original rotation quaternion.</param>
	/// <returns>Rotation matrix.</returns>
	public static double[][] MatrixFromQuaternion(Quaternion q)
	{
		double xx = q.X * q.X;
		double yy = q.Y * q.Y;
		double zz = q.Z * q.Z;
		double xy = q.X * q.Y;
		double xz = q.X * q.Z;
		double xw = q.X * q.W;
		double yz = q.Y * q.Z;
		double yw = q.Y * q.W;
		double zw = q.Z * q.W;

		return new double[3][] {new double[3] {1 - 2 * (yy + zz), 2 * (xy + zw),     2 * (xz - yw)},
		                        new double[3] {2 * (xy - zw),     1 - 2 * (xx + zz), 2 * (yz + xw)},
		                        new double[3] {2 * (xz + yw),     2 * (yz - xw),     1 - 2 * (xx + yy)}};
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
	/// Calculate the inverse of a jagged matrix.
	/// </summary>
	/// <remarks>Jagged version of the Accord.Net function.</remarks>
	/// <param name="matrix">Original matrix.</param>
	/// <returns>Matrix inverse.</returns>
	public static double[][] Inverse(this double[][] matrix)
	{
		return matrix.ToMatrix().Inverse().ToArray();
	}
}
}
