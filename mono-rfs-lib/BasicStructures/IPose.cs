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
	/// Location coordinates. May be zero if not appropriate.
	/// </summary>
	double[] Location { get; }

	/// <summary>
	/// Orientation coordinates. May be identity if not appropriate.
	/// </summary>
	Quaternion Orientation { get; }

	/// <summary>
	/// Obtain a local odometry representation for the pose around unity.
	/// </summary>
	/// <returns>Local odometry representation.</returns>
	double[] ToOdometry();

	/// <summary>
	/// Retrieve the pose from an odometry representation around unity.
	/// </summary>
	/// <param name="odometry">Linear representation.</param>
	/// <returns>Pose.</returns>
	Pose FromOdometry(double[] odometry);

	/// <summary>
	/// Obtain a local linear representation for the pose around unity.
	/// </summary>
	/// <returns>Local representation.</returns>
	double[] ToLinear();

	/// <summary>
	/// Retrieve the pose from a linear representation around unity.
	/// </summary>
	/// <param name="linear">Linear representation.</param>
	/// <returns>Pose.</returns>
	Pose FromLinear(double[] linear);

	/// <summary>
	/// Move the pose by an odometry delta.
	/// </summary>
	/// <param name="delta">Odometry delta.</param>
	/// <returns>Moved pose.</returns>
	Pose AddOdometry(double[] delta);

	/// <summary>
	/// Find the odometry delta that transforms another pose into this one.
	/// </summary>
	/// <param name="origin">Origin pose.</param>
	/// <returns>Pose difference in local odometry representation.</returns>
	double[] DiffOdometry(Pose origin);

	/// <summary>
	/// Add a linear vector (e.g. in Lie space) to the the pose around its pose manifold.
	/// </summary>
	/// <param name="delta">Odometry delta.</param>
	/// <returns>Moved pose.</returns>
	Pose Add(double[] delta);

	/// <summary>
	/// Find the linear vector (e.g. in Lie space) that transforms another pose into this one.
	/// </summary>
	/// <param name="origin">Origin pose.</param>
	/// <returns>Pose difference in local representation.</returns>
	double[] Subtract(Pose origin);

	/// <summary>
	/// Obtain a linearization for the Add operation around the given delta.
	/// </summary>
	/// <param name="delta">Odometry reading.</param>
	/// <returns>Jacobian.</returns>
	double[][] AddJacobian(double[] delta);

	/// <summary>
	/// Obtain a linearization for the Subtract operation around this pose.
	/// </summary>
	/// <param name="origin">Odometry reading.</param>
	/// <returns>Jacobian.</returns>
	double[][] SubtractJacobian(Pose origin);

	/// <summary>
	/// Obtain a linearization for the AddOdometry operation around this pose.
	/// It follows the form
	/// f(x[k-1], u) - x[k] ~ F dx[k-1] + G dx[k] - a.
	/// </summary>
	/// <param name="delta">Odometry reading.</param>
	/// <returns>Jacobian (F).</returns>
	double[][] AddOdometryJacobian(double[] delta);

	/// <summary>
	/// Number of state coordinates.
	/// </summary>
	int StateSize { get; }


	/// <summary>
	/// Number of odometry coordinates.
	/// </summary>
	int OdometrySize { get; }

	/// <summary>
	/// Create a pose from its state.
	/// </summary>
	/// <param name="state">State.</param>
	/// <returns>Pose.</returns>
	Pose FromState(double[] state);

	/// <summary>
	/// Deep clone.
	/// </summary>
	/// <returns>Pose clone.</returns>
	Pose DClone();

	/// <summary>
	/// Add the keyboard input to an odometry reading.
	/// </summary>
	/// <param name="odometry">Original odometry reading.</param>
	/// <param name="keyboard">Keyboard input in canonical 6-axis representation:
	/// (dlocx, dlocy, dlocz, dpitch, dyaw, droll). It may be used however makes
	/// sense in the particular pose space.</param>
	/// <returns>Modified odometry reading.</returns>
	double[] AddKeyboardInput(double[] odometry, double[] keyboard);

	/// <summary>
	/// Get the identity pose, however it may be defined.
	/// </summary>
	/// <remarks>Usually this will be defined as a static property inside the classes;
	/// this method works if polymorphism is an issue.</remarks>
	/// <returns>Identity pose.</returns>
	Pose IdentityP();

	/// <summary>
	/// Create a vehicle descriptor string.
	/// </summary>
	/// <param name="format">String format for double values.</param>
	/// <returns>Simulated vehicle descriptor string.</returns>
	string ToString(string format);
}
}
