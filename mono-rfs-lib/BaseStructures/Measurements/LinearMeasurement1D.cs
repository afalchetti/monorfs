// LinearMeasurement1D.cs
// Measurement structure consisting in 1D coordinate differences
// Part of MonoRFS
//
// Copyright (c) 2015-2016, Angelo Falchetti
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

namespace monorfs
{
/// <summary>
/// Measurement structure consisting in a dx coordinate difference.
/// </summary>
public class LinearMeasurement1D : IMeasurement<LinearMeasurement1D>
{
	public static LinearMeasurement1D Zero { get; private set; }

	/// <summary>
	/// Pixel x-coordinate difference.
	/// </summary>
	public double X { get; set; }

	/// <summary>
	/// Number of measurement coordinates.
	/// </summary>
	public int Size { get { return 1; } }

	/// <summary>
	/// Global constants initialization.
	/// </summary>
	static LinearMeasurement1D()
	{
		Zero = new LinearMeasurement1D(0);
	}

	/// <summary>
	/// Construct a new LinearMeasurement1D from its scalar components.
	/// </summary>
	/// <param name="dx">Pixel x coordinate difference.</param>
	public LinearMeasurement1D(double dx)
	{
		X = dx;
	}

	/// <summary>
	/// Construct a new default (zero) LinearMeasurement1D.
	/// </summary>
	public LinearMeasurement1D()
	{
		X = 0;
	}

	/// <summary>
	/// Obtain a local linear representation for the measurement.
	/// </summary>
	/// <returns>Local representation.</returns>
	public double[] ToLinear()
	{
		return new double[1] {X};
	}

	/// <summary>
	/// Retrieve the measurement from a linear representation.
	/// </summary>
	/// <param name="linear">Linear representation.</param>
	/// <returns>Measurement.</returns>
	public LinearMeasurement1D FromLinear(double[] linear)
	{
		if (linear == null || linear.Length != 1) {
			throw new ArgumentException("Linear representation must contain " +
			                            "exactly one value: (dx).");
		}

		return new LinearMeasurement1D(linear[0]);
	}

	/// <summary>
	/// Get a zero measurement, with dx = 0.
	/// </summary>
	/// <returns>Zero measurement.</returns>
	public LinearMeasurement1D ZeroM()
	{
		return Zero;
	}

	/// <summary>
	/// Create a vehicle descriptor string.
	/// </summary>
	/// <returns>Simulated vehicle descriptor string.</returns>
	public override string ToString()
	{
		return ToString("g6");
	}

	/// <summary>
	/// Create a vehicle descriptor string.
	/// </summary>
	/// <param name="format">String format for double values.</param>
	/// <returns>Simulated vehicle descriptor string.</returns>
	public string ToString(string format)
	{
		return "(" + X.ToString(format) + ")";
	}
}
}
