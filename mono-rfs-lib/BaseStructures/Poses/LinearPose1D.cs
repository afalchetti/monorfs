// LinearPose1D.cs
// 1D pose structure, consisting only of location
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

using Accord.Math;

namespace monorfs
{
/// <summary>
/// 1D pose structure, consisting only of location.
/// </summary>
public class LinearPose1D : IPose<LinearPose1D>
{
	/// <summary>
	/// Identity pose.
	/// </summary>
	public static LinearPose1D Identity { get; private set; }

	/// <summary>
	/// Location x-axis coordinate.
	/// </summary>
	public double X { get; set; }

	/// <summary>
	/// Space location coordinates.
	/// </summary>
	public double[] Location
	{
		get { return new double[3] { X, 0, 0 }; }
	}

	/// <summary>
	/// Orientation of the vehicle.
	/// </summary>
	public Quaternion Orientation { get { return Quaternion.Identity; } }

	/// <summary>
	/// Get the complete state as an array.
	/// </summary>
	public double[] State
	{
		get { return new double[1] {X}; }
	}

	/// <summary>
	/// Number of state coordinates.
	/// </summary>
	public int StateSize { get { return 1; } }

	/// <summary>
	/// Number of odometry coordinates.
	/// </summary>
	public int OdometrySize { get { return 1; } }

	/// <summary>
	/// Global constants initialization.
	/// </summary>
	static LinearPose1D() {
		Identity = new LinearPose1D();
	}

	/// <summary>
	/// Construct an identity pose.
	/// </summary>
	public LinearPose1D()
		: this(new double[1] {0}) {}

	/// <summary>
	/// Construct a pose given its internal state (x).
	/// </summary>
	/// <param name="state">Internal state.</param>
	public LinearPose1D(double[] state)
	{
		for (int i = 0; i < state.Length; i++) {
			if (double.IsNaN(state[i])) {
				throw new ArgumentException("State can't be NaN");
			}
		}

		X = state[0];
	}

	/// <summary>
	/// Construct a pose copying it from another one.
	/// </summary>
	/// <param name="that">Copied pose.</param>
	public LinearPose1D(LinearPose1D that)
		: this(that.State) {}


	/// <summary>
	/// Create a pose from its state.
	/// </summary>
	/// <param name="state">State.</param>
	/// <returns>Pose.</returns>
	public LinearPose1D FromState(double[] state)
	{
		if (state.Length != 1) {
			throw new ArgumentException("State should have exactly one parameter. It has " + state.Length + ".");
		}

		return new LinearPose1D(state);
	}

	/// <summary>
	/// Deep clone.
	/// </summary>
	/// <returns>Pose clone.</returns>
	public LinearPose1D DClone()
	{
		return new LinearPose1D(this);
	}

	/// <summary>
	/// Get the identity pose, with loc = 0 and rot = I.
	/// </summary>
	/// <returns>Identity pose.</returns>
	public LinearPose1D IdentityP()
	{
		return Identity;
	}

	/// <summary>
	/// Obtain a local odometry representation for the pose around unity.
	/// </summary>
	public double[] ToOdometry()
	{
		return this.DiffOdometry(Identity);
	}

	/// <summary>
	/// Retrieve the pose from an odometry representation around unity.
	/// </summary>
	/// <param name="linear">Linear representation.</param>
	public LinearPose1D FromOdometry(double[] linear)
	{
		return Identity.AddOdometry(linear);
	}

	/// <summary>
	/// Obtain a local lie linear representation for the pose around unity.
	/// </summary>
	public double[] ToLinear()
	{
		return this.Subtract(Identity);
	}

	/// <summary>
	/// Retrieve the pose from a lie linear representation around unity.
	/// </summary>
	/// <param name="linear">Linear representation.</param>
	public LinearPose1D FromLinear(double[] linear)
	{
		return Identity.Add(linear);
	}

	/// <summary>
	/// Add a linear vector in Lie space to the the pose around its pose manifold.
	/// It uses global coordinates, where translation and rotation are independent.
	/// </summary>
	/// <param name="delta">Odometry delta.</param>
	public LinearPose1D AddGlobal(double[] delta)
	{
		return new LinearPose1D(new double[1] {X + delta[0]});
	}

	/// <summary>
	/// Find the linear vector in semi-Lie space that transforms another pose into this one.
	/// It uses global coordinates, where translation and rotation are independent.
	/// </summary>
	/// <param name="origin">Origin pose.</param>
	public double[] SubtractGlobal(LinearPose1D origin)
	{
		return new double[1] {this.X - origin.X};
	}

	/// <summary>
	/// Add a linear vector in semi-Lie space to the the pose around its pose manifold.
	/// </summary>
	/// <param name="delta">Odometry delta.</param>
	public LinearPose1D Add(double[] delta)
	{
		return AddGlobal(delta);
	}

	/// <summary>
	/// Find the linear vector in semi-Lie space that transforms another pose into this one.
	/// </summary>
	/// <param name="origin">Origin pose.</param>
	public double[] Subtract(LinearPose1D origin)
	{
		return SubtractGlobal(origin);
	}

	/// <summary>
	/// Move the pose by an odometry delta.
	/// </summary>
	/// <param name="delta">Odometry delta.</param>
	public LinearPose1D AddOdometry(double[] delta)
	{
		return AddGlobal(delta);
	}

	/// <summary>
	/// Find the odometry delta that transforms another pose into this.
	/// </summary>
	/// <param name="origin">Origin pose.</param>
	public double[] DiffOdometry(LinearPose1D origin)
	{
		return SubtractGlobal(origin);
	}

	/// <summary>
	/// Obtain a linearization for the Add operation around the given delta.
	/// </summary>
	/// <param name="delta">Odometry reading.</param>
	/// <returns>Jacobian.</returns>
	public double[][] AddJacobian(double[] delta)
	{
		double[][] identity = new double[1][] { new double[1] {1} };

		return identity;
	}

	/// <summary>
	/// Obtain a linearization for the Subtract operation around this pose.
	/// </summary>
	/// <param name="origin">Odometry reading.</param>
	/// <returns>Jacobian.</returns>
	public double[][] SubtractJacobian(LinearPose1D origin)
	{
		double[][] identity = new double[1][] { new double[1] {1}};

		return identity;
	}

	/// <summary>
	/// Obtain a linearization for the AddOdometry operation around this pose.
	/// It follows the form
	/// f(x[k-1], u) - x[k] ~ F dx[k-1] + G dx[k] - a.
	/// </summary>
	/// <param name="delta">Odometry reading.</param>
	/// <returns>Jacobian (F).</returns>
	public double[][] AddOdometryJacobian(double[] delta)
	{
		return AddJacobian(delta);
	}

	/// <summary>
	/// Add the keyboard input to an odometry reading.
	/// </summary>
	/// <param name="odometry">Original odometry reading.</param>
	/// <param name="keyboard">Keyboard input in canonical 6-axis representation:
	/// (dlocx, dlocy, dlocz, dpitch, dyaw, droll). Only the first two coordinates are used;
	/// the rest is disregarded.</param>
	/// <returns>Modified odometry reading.</returns>
	public double[] AddKeyboardInput(double[] odometry, double[] keyboard)
	{
		return new double[1] {odometry[0] + 0.01 * keyboard[4]};
	}

	/// <summary>
	/// Get the transformation matrix representation of this pose.
	/// </summary>
	/// <returns>Pose matrix.</returns>
	public double[][] ToMatrix()
	{
		return new double[4][]
		{
			new double[4] {1, 0, 0, X},
			new double[4] {0, 1, 0, 0},
			new double[4] {0, 0, 1, 0},
			new double[4] {0, 0, 0, 1}
		};
	}

	/// <param name="a">First pose.</param>
	/// <param name="b">Second pose.</param>
	public static bool operator ==(LinearPose1D a, LinearPose1D b)
	{
		return a.Equals(b);
	}

	/// <param name="a">First pose.</param>
	/// <param name="b">Second pose.</param>
	public static bool operator !=(LinearPose1D a, LinearPose1D b)
	{
		return !(a == b);
	}

	/// <summary>
	/// Efficient equality comparer with another linear 1D pose.
	/// </summary>
	/// <param name="that">Compared pose.</param>
	/// <param name="epsilon">Maximum deviation between components.</param>
	/// <returns>True if both poses are the same.</returns>
	public bool Equals(LinearPose1D that, double epsilon = 0)
	{
		return this.Location.IsEqual(that.Location, epsilon);
	}

	/// <summary>
	/// Compares this object with another.
	/// </summary>
	/// <param name="that">Compared object.</param>
	/// <returns>True if the objects are the same.</returns>
	public override bool Equals(object that)
	{
		return that is LinearPose1D && this.Equals(that as LinearPose1D);
	}

	/// <summary>
	/// Get a unique code that is equal for any two equal 1D poses.
	/// </summary>
	/// <returns>Hash code.</returns>
	public override int GetHashCode()
	{
		int hash = 17;

		hash = unchecked(37 * hash + X.GetHashCode());

		return hash;
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

	/// <summary>
	/// Create a vehicle descriptor string.
	/// </summary>
	/// <returns>Simulated vehicle descriptor string.</returns>
	public override string ToString()
	{
		return ToString("f3");
	}
}
}
