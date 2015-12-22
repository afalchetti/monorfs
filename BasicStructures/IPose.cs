// IPose.cs
// Pose structure interface
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
/// 3D pose structure.
/// </summary>
public interface IPose<Pose>
{
	/// <summary>
	/// Get the internal state representation.
	/// </summary>
	double[] State { get; }

	/// <summary>
	/// Obtain a local linear representation for the pose around unity.
	/// </summary>
	double[] ToLinear();

	/// <summary>
	/// Retrieve the pose from a linear representation around unity.
	/// </summary>
	/// <param name="linear">Linear representation.</param>
	Pose FromLinear(double[] linear);

	/// <summary>
	/// Move the pose by an odometry delta.
	/// </summary>
	/// <param name="delta">Odometry delta.</param>
	Pose Add(double[] delta);

	/// <summary>
	/// Find the odometry delta that transforms another pose into this.
	/// </summary>
	/// <param name="origin">Origin pose.</param>
	double[] Subtract(Pose origin);
}
}
