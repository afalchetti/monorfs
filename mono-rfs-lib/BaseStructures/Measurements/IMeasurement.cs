// IMeasurement.cs
// Measurement structure interface
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
using System.Collections;
using System.Collections.Generic;

using Accord.Math;
using Accord.MachineLearning.Structures;

namespace monorfs
{
/// <summary>
/// Measurement structure.
/// </summary>
public interface IMeasurement<Measurement>
{
	/// <summary>
	/// Obtain a linear representation for the measurement.
	/// </summary>
	/// <returns>Linear representation.</returns>
	double[] ToLinear();

	/// <summary>
	/// Retrieve the measurement from a linear representation.
	/// </summary>
	/// <param name="linear">Linear representation.</param>
	/// <returns>Measurement.</returns>
	Measurement FromLinear(double[] linear);

	/// <summary>
	/// Number of measurement coordinates.
	/// </summary>
	int Size { get; }

	/// <summary>
	/// Get a zero measurement, however it may be defined.
	/// </summary>
	/// <remarks>Usually this will be defined as a static property inside the classes;
	/// this method works if polymorphism is an issue.</remarks>
	/// <returns>Zero measurement.</returns>
	Measurement ZeroM();

	/// <summary>
	/// Create a vehicle descriptor string.
	/// </summary>
	/// <param name="format">String format for double values.</param>
	/// <returns>Simulated vehicle descriptor string.</returns>
	string ToString(string format);
}
}
