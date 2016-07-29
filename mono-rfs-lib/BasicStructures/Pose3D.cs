// Pose3D.cs
// 3D pose structure, consisting of location and orientation
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
/// 3D pose structure, consisting of location and orientation.
/// </summary>
public class Pose3D : IPose<Pose3D>
{
	/// <summary>
	/// Identity pose.
	/// </summary>
	public static Pose3D Identity { get; private set; }

	/// <summary>
	/// Location x-axis coordinate.
	/// </summary>
	public double X { get; set; }

	/// <summary>
	/// Location y-axis coordinate.
	/// </summary>
	public double Y { get; set; }

	/// <summary>
	/// Location z-axis coordinate.
	/// </summary>
	public double Z { get; set; }

	/// <summary>
	/// Space location coordinates.
	/// </summary>
	public double[] Location
	{
		get { return new double[3] { X, Y, Z }; }
		//set { X = value[0]; Y = value[1]; Z = value[2]; }
	}

	/// <summary>
	/// Orientation of the vehicle.
	/// </summary>
	public Quaternion Orientation { get; private set; }

	/// <summary>
	/// Rotation quaternion scalar component.
	/// </summary>
	public double W {
		get { return Orientation.W; }
		//set { Orientation = new Quaternion(value, Orientation.X, Orientation.Y, Orientation.Z); }
	}

	/// <summary>
	/// Rotation quaternion vector component.
	/// </summary>
	public double[] K {
		get { return new double[3] { Orientation.X, Orientation.Y, Orientation.Z }; }
		//set { Orientation = new Quaternion(Orientation.W, value[0], value[1], value[2]); }
	}

	/// <summary>
	/// Angle in angle-axis representation.
	/// </summary>
	public double Angle
	{
		get { return 2 * Math.Acos(Orientation.W); }
	}

	/// <summary>
	/// Axis in angle-axis representation.
	/// </summary>
	public double[] Axis
	{
		get
		{
			double normalize = Math.Sqrt(1 - Orientation.W * Orientation.W);
			return (normalize > 1e-8) ? K.Divide(normalize) : new double[3] {1, 0, 0};
		}
	}

	/// <summary>
	/// Get the complete state as an array.
	/// </summary>
	public double[] State
	{
		get { return new double[7] {X, Y, Z, Orientation.W, Orientation.X, Orientation.Y, Orientation.Z}; }
	}

	static Pose3D() {
		Identity = new Pose3D();
	}

	/// <summary>
	/// Construct an identity pose.
	/// </summary>
	public Pose3D()
		: this(new double[7] {0, 0, 0, 1, 0, 0, 0}) {}

	/// <summary>
	/// Construct a pose given its internal state (x, y, z, qw, qx, qy, qz).
	/// </summary>
	/// <param name="state">Internal state.</param>
	public Pose3D(double[] state)
	{
		for (int i = 0; i < state.Length; i++) {
			if (double.IsNaN(state[i])) {
				throw new ArgumentException("State can't be NaN");
			}
		}

		X = state[0];
		Y = state[1];
		Z = state[2];

		Orientation = new Quaternion(state[3], state[4], state[5], state[6]);
	}

	/// <summary>
	/// Construct a pose from its location and orientation.
	/// </summary>
	/// <param name="location">Location.</param>
	/// <param name="orientation">Orientation.</param>
	public Pose3D(double[] location, Quaternion orientation)
	{
		/*for (int i = 0; i < location.Length; i++) {
			if (double.IsNaN(location[i])) {
				throw new ArgumentException("State can't be NaN");
			}
		}
		if (double.IsNaN(orientation.W)) {
			throw new ArgumentException("State can't be NaN");
		}
		if (double.IsNaN(orientation.X)) {
			throw new ArgumentException("State can't be NaN");
		}
		if (double.IsNaN(orientation.Y)) {
			throw new ArgumentException("State can't be NaN");
		}
		if (double.IsNaN(orientation.Z)) {
			throw new ArgumentException("State can't be NaN");
		}*/

		X = location[0];
		Y = location[1];
		Z = location[2];

		Orientation = new Quaternion(orientation);
	}

	/// <summary>
	/// Construct a pose copying it from another one.
	/// </summary>
	/// <param name="that">Copied pose.</param>
	public Pose3D(Pose3D that)
		: this(that.State) {}
	
	/// <summary>
	/// Obtain a local linear representation for the pose around unity.
	/// </summary>
	public double[] ToLinear()
	{
		return this.Subtract(Identity);
	}

	/// <summary>
	/// Retrieve the pose from a linear representation around unity.
	/// </summary>
	/// <param name="linear">Linear representation.</param>
	public Pose3D FromLinear(double[] linear)
	{
		return Identity.Add(linear);
	}

	/// <summary>
	/// Move the pose by an odometry delta.
	/// </summary>
	/// <param name="delta">Odometry delta.</param>
	public Pose3D Add(double[] delta)
	{
		Quaternion dorientation   = Quaternion.Identity.Add(new double[3] {delta[3], delta[4], delta[5]});
		Quaternion neworientation = Orientation * dorientation;
		Quaternion middelta       = dorientation.Sqrt();
		Quaternion midrotation    = Orientation * middelta;
		Quaternion dlocation      = midrotation * new Quaternion(0, delta[0], delta[1], delta[2]) * midrotation.Conjugate();

		double[]   location    = new double[3] {X + dlocation.X, Y + dlocation.Y, Z + dlocation.Z};
		Quaternion orientation = neworientation.Normalize();

		return new Pose3D(location, orientation);
	}

	/// <summary>
	/// Find the odometry delta that transforms another pose into this.
	/// </summary>
	/// <param name="origin">Origin pose.</param>
	public double[] Subtract(Pose3D origin)
	{
		Quaternion dq          = origin.Orientation.Conjugate() * this.Orientation;
		Quaternion middelta    = dq.Sqrt();
		Quaternion midrotation = origin.Orientation * middelta;
		double[]   dxglobal    = this.Location.Subtract(origin.Location);
		Quaternion dx          = midrotation.Conjugate() *
		                             new Quaternion(0, dxglobal[0], dxglobal[1], dxglobal[2]) *
		                             midrotation;
		
		double[] lie = 2.Multiply(Quaternion.Log(dq));

		return new double[6] {dx.X, dx.Y, dx.Z, lie[0], lie[1], lie[2]};
	}

	/// <summary>
	/// Obtain a linearization for the Add operation around this pose.
	/// It follows the form
	/// f(x[k-1], u) - x[k] ~ F dx[k-1] + G dx[k] - a.
	/// </summary>
	/// <param name="delta">Odometry reading.</param>
	/// <returns>Jacobian (F).</returns>
	public double[][] AddJacobian(double[] delta)
	{
		Quaternion dorientation = Quaternion.Identity.Add(new double[3] {delta[3], delta[4], delta[5]});
		Quaternion middelta     = dorientation.Sqrt();

		double[][] Cdelta       = Util.Quat2Matrix(dorientation);
		double[][] Cmiddelta    = Util.Quat2Matrix(middelta);

		double[][] crossdX  = Util.CrossProductMatrix(new double[3] {delta[0], delta[1], delta[2]});

		double[][] dxdq = (-1.0).Multiply(Cmiddelta.Transpose())
		                        .Multiply(crossdX);

		return Cdelta.Transpose()      .Concatenate(dxdq).VConcatenate(
		       MatrixExtensions.Zero(3).Concatenate(Cdelta.Transpose()));
	}

	/// <param name="a">First quaternion.</param>
	/// <param name="b">Second quaternion.</param>
	public static bool operator ==(Pose3D a, Pose3D b)
	{
		return a.Equals(b);
	}

	/// <param name="a">First quaternion.</param>
	/// <param name="b">Second quaternion.</param>
	public static bool operator !=(Pose3D a, Pose3D b)
	{
		return !(a == b);
	}

	/// <summary>
	/// Efficient equality comparer with another 3d pose.
	/// </summary>
	/// <param name="that">Compared pose.</param>
	/// <param name="epsilon">Maximum deviation between components.</param>
	/// <returns>True if both poses are the same.</returns>
	public bool Equals(Pose3D that, double epsilon = 0)
	{
		return this.Location.IsEqual(that.Location, epsilon) &&
		       this.Orientation.Equals(that.Orientation, epsilon);
	}

	/// <summary>
	/// Compares this object with another.
	/// </summary>
	/// <param name="that">Compared object.</param>
	/// <returns>True if the objects are the same.</returns>
	public override bool Equals(object that)
	{
		return that is Pose3D && this.Equals(that as Pose3D);
	}

	/// <summary>
	/// Get a unique code that is equal for any two equal 3d poses.
	/// </summary>
	/// <returns>Hash code.</returns>
	public override int GetHashCode()
	{
		int hash = 17;

		hash = unchecked(37 * hash + Location.GetHashCode());
		hash = unchecked(37 * hash + Orientation.GetHashCode());

		return hash;
	}

	/// <summary>
	/// Get a string representation of the pose.
	/// </summary>
	/// <param name="format">Double formatting descriptor.</param>
	/// <returns>String representation.</returns>
	public string ToString(string format)
	{
		return "[(" + X.ToString(format) + ", " + Y.ToString(format) + ", " + Z.ToString(format) + "); " +
		       Orientation.ToString(format) + "]";
	}

	/// <summary>
	/// Get a string representation of the pose.
	/// </summary>
	/// <returns>String representation.</returns>
	public override string ToString()
	{
		return ToString("f3");
	}
}
}
