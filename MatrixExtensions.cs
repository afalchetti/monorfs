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
using System.Collections.Generic;

using Accord;
using Accord.Math;
using Microsoft.Xna.Framework;

namespace monorfs
{
/// <summary>
/// Utility extensions to the matrix and vector classes (double[][] and double[])
/// </summary>
public static class MatrixExtensions
{
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
	/// Get a 2x2 rotation matrix of a specified angle.
	/// </summary>
	/// <param name="angle">Angle measured in radians, counterclockwise starting from (0, 1).</param>
	/// <returns>The rotation matrix.</returns>
	public static double[][] Rotation2(double angle)
	{
		double ct = Math.Cos(angle);
		double st = Math.Sin(angle);
		return new double[2][] {new double[2] {ct, -st}, new double[2] {st, ct}};
	}
	/*
	/// <summary>
	/// Write a 2x2 rotation matrix of a specified angle inside another matrix.
	/// </summary>
	/// <param name="angle">Angle measured in radians, counterclockwise starting from (0, 1).</param>
	/// <returns>The rotation matrix.</returns>
	/// */

	/// <summary>
	/// Write a 2x2 rotation matrix of a specified angle inside another matrix.
	/// </summary>
	/// <param name="angle">Angle measured in radians, counterclockwise starting from (0, 1).</param>
	/// <param name="matrix">Output matrix.Must have enough room to accomodate the 2x2 rotation.</param>
	/// <param name="offx">X-index offset to start writing.</param>
	/// <param name="offy">Y-index offset to start writing.</param>
	public static void Rotation2(double angle, ref double[][] matrix, int offx, int offy)
	{
		double ct = Math.Cos(angle);
		double st = Math.Sin(angle);
		matrix[offx    ][offy]     =  ct;
		matrix[offx + 1][offy]     = -st;
		matrix[offx    ][offy + 1] =  st;
		matrix[offx + 1][offy + 1] =  ct;
	}

	/// <summary>
	/// Get a 3x3 homographic rotation matrix of a specified angle.
	/// </summary>
	/// <param name="angle">Angle measured in radians, counterclockwise starting from (0, 1).</param>
	/// <returns>The homographic rotation matrix.</returns>
	public static double[][] Rotation2H(double angle)
	{
		double ct = Math.Cos(angle);
		double st = Math.Sin(angle);
		return new double[3][] {new double[3] {ct, -st, 0}, new double[3] {st, ct, 0}, new double[3] {0, 0, 1}};
	}

	/// <summary>
	/// Get a 3x3 translation matrix of a specified movement.
	/// </summary>
	/// <param name="move">Translation offset in xy-coordinates.</param>
	/// <returns>The homographic translation matrix.</returns>
	public static double[][] Translation2H(double[] move)
	{
		return new double[3][] {new double[3] {0, 0, move[0]}, new double[3] {0, 0, move[1]}, new double[3] {0, 0, 1}};
	}

	/// <summary>
	/// Get a complete 3x3 homographic transform matrix.
	/// </summary>
	/// <param name="translation">Translation offset in xy-coordinates.</param>
	/// <param name="angle">Angle measured in radians, counterclockwise starting from (0, 1).</param>
	/// <returns>The homographic transformation matrix.</returns>
	public static double[][] TransformH(double[] translation, double angle)
	{
		double ct = Math.Cos(angle);
		double st = Math.Sin(angle);
		return new double[3][] {new double[3] {ct, -st, translation[0]}, new double[3] {st, ct, translation[1]}, new double[3] {0, 0, 1}};
	}

	/// <summary>
	/// Creates a 3x3 rotation matrix for rotating in the yz plane
	/// with a given angle.
	/// </summary>
	/// <param name="angle">Angle measured in radians, counterclockwise starting from (0, 0, 1).</param>
	/// <returns>The rotation matrix.</returns>
	public static double[][] CreateRotationX(double angle)
	{
		double ct = Math.Cos(angle);
		double st = Math.Sin(angle);
		return new double[3][] {new double[3] {1, 0, 0}, new double[3] {0, ct, -st}, new double[3] {0, st, ct}};
	}

	/// <summary>
	/// Creates a 3x3 rotation matrix for rotating in the xz plane
	/// with a given angle.
	/// </summary>
	/// <param name="angle">Angle measured in radians, counterclockwise starting from (1, 0, 0).</param>
	/// <returns>The rotation matrix.</returns>
	public static double[][] CreateRotationY(double angle)
	{
		double ct = Math.Cos(angle);
		double st = Math.Sin(angle);
		return new double[3][] {new double[3] {st, 0, ct}, new double[3] {0, 1, 0}, new double[3] {ct, 0, -st}};
	}

	/// <summary>
	/// Creates a 3x3 rotation matrix for rotating in the xy plane
	/// with a given angle.
	/// </summary>
	/// <param name="angle">Angle measured in radians, counterclockwise starting from (0, 1, 0).</param>
	/// <returns>The rotation matrix.</returns>
	public static double[][] CreateRotationZ(double angle)
	{
		double ct = Math.Cos(angle);
		double st = Math.Sin(angle);
		return new double[3][] {new double[3] {ct, -st, 0}, new double[3] {st, ct, 0}, new double[3] {0, 0, 1}};
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
	/// <param name="a">Original matrix.</param>
	/// <returns>Matrix determinant.</returns>
	public static double Determinant(this double[][] matrix)
	{
		return matrix.ToMatrix().Determinant();
	}
	
	/// <summary>
	/// Calculate the inverse of a jagged matrix.
	/// </summary>
	/// <remarks>Jagged version of the Accord.Net function.</remarks>
	/// <param name="a">Original matrix.</param>
	/// <returns>Matrix inverse.</returns>
	public static double[][] Inverse(this double[][] matrix)
	{
		return matrix.ToMatrix().Inverse().ToArray();
	}
}
}
