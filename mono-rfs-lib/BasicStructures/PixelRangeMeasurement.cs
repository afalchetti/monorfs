// PixelRangeMeasurement.cs
// Measurement structure consisting in two pixel coordinates and a range
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
using OpenTK.Graphics.ES11;

namespace monorfs
{
/// <summary>
/// Measurement structure consisting in two pixel coordinates and a range.
/// </summary>
public class PixelRangeMeasurement : IMeasurement<PixelRangeMeasurement>
{
	public static PixelRangeMeasurement Zero { get; private set; }

	/// <summary>
	/// Pixel x-coordinate.
	/// </summary>
	public double X { get; set; }

	/// <summary>
	/// Pixel y-coordinate.
	/// </summary>
	public double Y { get; set; }

	/// <summary>
	/// Distance between the sensor and the measured object.
	/// </summary>
	public double Range { get; set; }

	/// <summary>
	/// Number of measurement coordinates.
	/// </summary>
	public int Size { get { return 3; } }

	/// <summary>
	/// Global constants initialization.
	/// </summary>
	static PixelRangeMeasurement()
	{
		Zero = new PixelRangeMeasurement(0, 0, 0);
	}

	/// <summary>
	/// Construct a new PixelRangeMeasurement from its scalar components.
	/// </summary>
	/// <param name="x">Pixel x coordinate.</param>
	/// <param name="y">Pixel y coordinate.</param>
	/// <param name="range">Distance to the sensor.</param>
	public PixelRangeMeasurement(double x, double y, double range)
	{
		X     = x;
		Y     = y;
		Range = range;
	}

	/// <summary>
	/// Construct a new default (zero) PixelRangeMeasurement.
	/// </summary>
	public PixelRangeMeasurement()
	{
		X     = 0;
		Y     = 0;
		Range = 0;
	}

	/// <summary>
	/// Obtain a local linear representation for the measurement.
	/// </summary>
	/// <returns>Local representation.</returns>
	public double[] ToLinear()
	{
		return new double[3] {X, Y, Range};
	}

	/// <summary>
	/// Retrieve the measurement from a linear representation.
	/// </summary>
	/// <param name="linear">Linear representation.</param>
	/// <returns>Measurement.</returns>
	public PixelRangeMeasurement FromLinear(double[] linear)
	{
		if (linear == null || linear.Length != 3) {
			throw new ArgumentException("Linear representation must contain " +
			                            "exactly three values: (px, py, range).");
		}

		return new PixelRangeMeasurement(linear[0], linear[1], linear[2]);
	}

	/// <summary>
	/// Get a zero measurement, with px = py = range = 0.
	/// </summary>
	/// <returns>Zero measurement.</returns>
	public PixelRangeMeasurement ZeroM()
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
		return "(" + X.ToString(format) + ", " + Y.ToString(format) + ", " + Range.ToString(format) + ")";
	}
}
}
