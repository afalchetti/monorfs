// MatrixEntensions.cs
// Utility extensions to matrix class (double[,])
// Part of MonoRFS
//
// Copyright (C) 2015 Angelo Falchetti
// All rights reserved.
// Any use, reproduction, distribution or copy of this
// work via any medium is strictly forbidden without
// express written consent from the author.

using System;
using System.Collections.Generic;

using Accord;
using Accord.Math;

namespace monorfs
{
/// <summary>
/// Utility extensions to the matrix and vector classes (double[,] and double[])
/// </summary>
public static class MatrixExtensions
{
	/// <summary>
	/// Get a 2x2 rotation matrix of a specified angle.
	/// </summary>
	/// <param name="angle">Angle measured in radians, counterclockwise starting from (0, 1).</param>
	/// <returns>The rotation matrix.</returns>
	public static double[,] Rotation2(double angle)
	{
		double ct = Math.Cos(angle);
		double st = Math.Sin(angle);
		return new double[2, 2] {{ct, -st}, {st, ct}};
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
	public static void Rotation2(double angle, ref double[,] matrix, int offx, int offy)
	{
		double ct = Math.Cos(angle);
		double st = Math.Sin(angle);
		matrix[offx,     offy]     =  ct;
		matrix[offx + 1, offy]     = -st;
		matrix[offx,     offy + 1] =  st;
		matrix[offx + 1, offy + 1] =  ct;
	}

	/// <summary>
	/// Get a 3x3 homographic rotation matrix of a specified angle.
	/// </summary>
	/// <param name="angle">Angle measured in radians, counterclockwise starting from (0, 1).</param>
	/// <returns>The homographic rotation matrix.</returns>
	public static double[,] Rotation2H(double angle)
	{
		double ct = Math.Cos(angle);
		double st = Math.Sin(angle);
		return new double[3, 3] {{ct, -st, 0}, {st, ct, 0}, {0, 0, 1}};
	}

	/// <summary>
	/// Get a 3x3 translation matrix of a specified movement.
	/// </summary>
	/// <param name="move">Translation offset in xy-coordinates.</param>
	/// <returns>The homographic translation matrix.</returns>
	public static double[,] Translation2H(double[] move)
	{
		return new double[3, 3] {{0, 0, move[0]}, {0, 0, move[1]}, {0, 0, 1}};
	}

	/// <summary>
	/// Get a complete 3x3 homographic transform matrix.
	/// </summary>
	/// <param name="translation">Translation offset in xy-coordinates.</param>
	/// <param name="angle">Angle measured in radians, counterclockwise starting from (0, 1).</param>
	/// <returns>The homographic transformation matrix.</returns>
	public static double[,] TransformH(double[] translation, double angle)
	{
		double ct = Math.Cos(angle);
		double st = Math.Sin(angle);
		return new double[3, 3] {{ct, -st, translation[0]}, {st, ct, translation[1]}, {0, 0, 1}};
	}

	/*
	 * TODO this, when it is necessary
	public static Rotation3(double theta, double phi, double psi)
	{
		double ct = Math.Cos(angle);
		double st = Math.Sin(angle);
		return new double[,] {{ct, -st}, {st, ct}};
	}

	private static Rotation3(double theta, int axis)
	{
		double ct = Math.Cos(angle);
		double st = Math.Sin(angle);
		return new double[,] {{ct, -st}, {st, ct}};
	}

	public static Rotation3H(double theta)
	{
	}*/
}
}
