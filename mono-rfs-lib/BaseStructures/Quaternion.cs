// Quaternion.cs
// Quaternion structure
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

namespace monorfs
{
/// <summary>
/// Quaternion structure.
/// </summary>
public class Quaternion
{
	/// <summary>
	/// Scalar component.
	/// </summary>
	public double W { get; private set; }

	/// <summary>
	/// X coordinate.
	/// </summary>
	public double X { get; private set; }

	/// <summary>
	/// Y coordinate.
	/// </summary>
	public double Y { get; private set; }

	/// <summary>
	/// Z coordinate.
	/// </summary>
	public double Z { get; private set; }

	/// <summary>
	/// Get the magnitude (absolute value) of this quaternion.
	/// </summary>
	public double Magnitude
	{
		get { return Math.Sqrt(W * W + X * X + Y * Y + Z * Z); }
	}

	/// <summary>
	/// Angle in angle-axis representation.
	/// </summary>
	public double Angle
	{
		get { return 2 * Math.Acos(W); }
	}

	/// <summary>
	/// Gets the identity.
	/// </summary>
	/// <value>The identity.</value>
	public static Quaternion Identity { get; private set; }

	static Quaternion()
	{
		Identity = new Quaternion(1, 0, 0, 0);
	}

	/// <summary>
	/// Default constructor.
	/// </summary>
	private Quaternion() {}

	/// <summary>
	/// Construct a new quaternion from its components.
	/// </summary>
	/// <param name="w">Scalar component.</param>
	/// <param name="x">X coordinate.</param>
	/// <param name="y">Y coordinate.</param>
	/// <param name="z">Z coordinate.</param>
	public Quaternion(double w, double x, double y, double z)
	{
		W = w;
		X = x;
		Y = y;
		Z = z;
	}

	/// <summary>
	/// Construct a new quaternion from its components.
	/// </summary>
	/// <param name="w">Scalar component.</param>
	/// <param name="xyz">Vector coordinate.</param>
	public Quaternion(double w, double[] xyz)
	{
		W = w;
		X = xyz[0];
		Y = xyz[1];
		Z = xyz[2];
	}

	/// <summary>
	/// Construct a Quaternion copying its components from another.
	/// </summary>
	/// <param name="that">Copied quaternion.</param>
	public Quaternion(Quaternion that)
	{
		this.W = that.W;
		this.X = that.X;
		this.Y = that.Y;
		this.Z = that.Z;
	}

	/// <summary>
	/// Obtain a local linear representation in Lie space for the quaternion around unity.
	/// </summary>
	public double[] ToLinear()
	{
		//return this.Subtract(Identity);
		return 2.Multiply(Log(this));
	}

	/// <summary>
	/// Retrieve the quaternion from a linear representation in Lie space around unity.
	/// </summary>
	/// <param name="linear">Linear representation.</param>
	public Quaternion FromLinear(double[] linear)
	{
		//return Identity.Add(linear);
		return Exp(0.5.Multiply(linear));
	}

	/// <summary>
	/// Obtain the conjugate of this quaternion.
	/// </summary>
	/// <returns>Conjugate.</returns>
	public Quaternion Conjugate()
	{
		return new Quaternion(W, -X, -Y, -Z);
	}

	/// <summary>
	/// Add a Lie delta element to this quaternion.
	/// </summary>
	/// <param name="b">Lie delta element.</param>
	/// <returns>Sum.</returns>
	public Quaternion Add(double[] b)
	{
		return this * Exp(0.5.Multiply(b));
	}

	/// <summary>
	/// Subtract a quaternion to another quaternion to obtain a Lie delta element.
	/// </summary>
	/// <param name="b">Subtracted quaternion.</param>
	/// <returns>Lie delta element.</returns>
	public double[] Subtract(Quaternion b)
	{
		return 2.Multiply(Log(b.Conjugate() * this));
	}

	/// <summary>
	/// Exponential of a lie algebra element.
	/// </summary>
	/// <param name="lie">Lie element.</param>
	/// <returns>Associated quaternion.</returns>
	public static Quaternion Exp(double[] lie)
	{
		double phi = lie.Euclidean();

		if (phi < 1e-12) {
			return Identity;
		}

		double[] vector = Math.Sin(phi).Multiply(lie.Normalize());

		return new Quaternion(Math.Cos(phi), vector[0], vector[1], vector[2]);
	}

	/// <summary>
	/// Logarithm of a quaternion.
	/// </summary>
	/// <param name="q">Quaternion.</param>
	/// <returns>Associated lie element.</returns>
	public static double[] Log(Quaternion q)
	{
		q = q.Normalize();

		double   phi    = Math.Acos(q.W);
		double[] vector = new double[3] {q.X, q.Y, q.Z};
		double   mag    = vector.Euclidean();

		if (mag < 1e-12) {
			return new double[3] {0, 0, 0};
		}

		double[] unit = vector.Normalize();
		return phi.Multiply(unit);
	}

	/// <summary>
	/// Calculate the positive square root of a quaternion
	/// distinct to the identity. If it is the identity,
	/// the result is undefined.
	/// </summary>
	/// <returns>Square root.</returns>
	public Quaternion Sqrt()
	{
		if (Math.Abs(W - -1.0) < 1e-8) {
			return Identity;
		}

		double rw    = Math.Sqrt(0.5 * (1 + W));
		double alpha = 1 / (2 * rw);

		return new Quaternion(rw, alpha * X, alpha * Y, alpha * Z);
	}

	/// <summary>
	/// Get a normalized version of this quaternion.
	/// </summary>
	public Quaternion Normalize()
	{
		double alpha = 1 / Magnitude;

		return alpha * this;
	}

	/// <summary>
	/// Create a new quaternion from its yaw-pitch-roll representation.
	/// </summary>
	/// <returns>Equivalent quaternion.</returns>
	/// <param name="yaw">Yaw coordinate.</param>
	/// <param name="pitch">Pitch coordinate.</param>
	/// <param name="roll">Roll coordinate.</param>
	public static Quaternion CreateFromYawPitchRoll(double yaw, double pitch, double roll)
	{
		double y2 = 0.5 * yaw;
		double p2 = 0.5 * pitch;
		double r2 = 0.5 * roll;

		double sy = Math.Sin(y2);
		double cy = Math.Cos(y2);

		double sp = Math.Sin(p2);
		double cp = Math.Cos(p2);

		double sr = Math.Sin(r2);
		double cr = Math.Cos(r2);

		return new Quaternion(cy * cp * cr + sy * sp * sr,
		                      cy * sp * cr + sy * cp * sr,
		                      sy * cp * cr - cy * sp * sr,
		                      cy * cp * sr - sy * sp * cr);
	}

	/// <summary>
	/// Create a quaternion that rotates a specified unitary vector into another.
	/// </summary>
	/// <param name="from">Input vector.</param>
	/// <param name="to">Output vector.</param>
	/// <returns>Quaternion that rotates 'from' into 'to'.</returns>
	public static Quaternion VectorRotator(double[] from, double[] to)
	{
		return new Quaternion(1 + from.InnerProduct(to), Matrix.VectorProduct(from, to)).Normalize();
	}

	/// <param name="a">First quaternion.</param>
	/// <param name="b">Second quaternion.</param>
	public static Quaternion operator +(Quaternion a, Quaternion b)
	{
		return new Quaternion(a.W + b.W, a.X + b.X, a.Y + b.Y, a.Z + b.Z);
	}

	/// <param name="a">First quaternion.</param>
	/// <param name="b">Second quaternion.</param>
	public static Quaternion operator *(Quaternion a, Quaternion b)
	{
		return new Quaternion(a.W * b.W - (a.X * b.X + a.Y * b.Y + a.Z * b.Z),
		                      a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
		                      a.W * b.Y + a.Y * b.W + a.Z * b.X - a.X * b.Z,
		                      a.W * b.Z + a.Z * b.W + a.X * b.Y - a.Y * b.X);
	}

	/// <param name="alpha">Scalar value.</param>
	/// <param name="q">First quaternion.</param>
	public static Quaternion operator *(double alpha, Quaternion q)
	{
		return new Quaternion(alpha * q.W, alpha * q.X, alpha * q.Y, alpha * q.Z);
	}

	/// <param name="a">First quaternion.</param>
	/// <param name="b">Second quaternion.</param>
	public static Quaternion operator -(Quaternion a, Quaternion b)
	{
		return new Quaternion(a.W - b.W, a.X - b.X, a.Y - b.Y, a.Z - b.Z);
	}

	/// <param name="q">Quaternion.</param>
	public static Quaternion operator -(Quaternion q)
	{
		return new Quaternion(-q.W, -q.X, -q.Y, -q.Z);
	}

	/// <summary>
	/// Get the unitary matrix representation of this quaternion.
	/// </summary>
	/// <returns>Rotation matrix.</returns>
	public double[][] ToMatrix()
	{
		double xx = X * X;
		double yy = Y * Y;
		double zz = Z * Z;
		double xy = X * Y;
		double xz = X * Z;
		double xw = X * W;
		double yz = Y * Z;
		double yw = Y * W;
		double zw = Z * W;

		return new double[3][] { new double[3] {1 - 2 * (yy + zz), 2 * (xy - zw),     2 * (xz + yw)},
		                         new double[3] {2 * (xy + zw),     1 - 2 * (xx + zz), 2 * (yz - xw)},
		                         new double[3] {2 * (xz - yw),     2 * (yz + xw),     1 - 2 * (xx + yy)} };
	}

	/// <param name="a">First quaternion.</param>
	/// <param name="b">Second quaternion.</param>
	public static bool operator ==(Quaternion a, Quaternion b)
	{
		return a.Equals(b);
	}

	/// <param name="a">First quaternion.</param>
	/// <param name="b">Second quaternion.</param>
	public static bool operator !=(Quaternion a, Quaternion b)
	{
		return !(a == b);
	}

	/// <summary>
	/// Efficient equality comparer with another quaternion.
	/// </summary>
	/// <param name="that">Compared quaternion.</param>
	/// <param name="epsilon">Maximum deviation between components.</param>
	/// <returns>True if both quaternion are the same.</returns>
	public bool Equals(Quaternion that, double epsilon = 0)
	{
		return Math.Abs(this.W - that.W) < epsilon &&
		       Math.Abs(this.X - that.X) < epsilon &&
		       Math.Abs(this.Y - that.Y) < epsilon &&
		       Math.Abs(this.Z - that.Z) < epsilon;
	}

	/// <summary>
	/// Compares this object with another.
	/// </summary>
	/// <param name="that">Compared object.</param>
	/// <returns>True if the objects are the same.</returns>
	public override bool Equals(object that)
	{
		return that is Quaternion && this.Equals(that as Quaternion);
	}

	/// <summary>
	/// Get a unique code that is equal for any two equal Quaternions.
	/// </summary>
	/// <returns>Hash code.</returns>
	public override int GetHashCode()
	{
		int hash = 17;

		hash = unchecked(37 * hash + W.GetHashCode());
		hash = unchecked(37 * hash + X.GetHashCode());
		hash = unchecked(37 * hash + Y.GetHashCode());
		hash = unchecked(37 * hash + Z.GetHashCode());

		return hash;
	}

	/// <summary>
	/// Get a string representation of this quaternion.
	/// </summary>
	/// <param name="format">Double formatting descriptor.</param>
	public string ToString(string format)
	{
		return "(" + W.ToString(format) + ", " + X.ToString(format) + ", " +
		             Y.ToString(format) + ", " + Z.ToString(format) + ")";
	}

	/// <summary>
	/// Get a string representation of this quaternion.
	/// </summary>
	public override string ToString()
	{
		return ToString("f3");
	}
}
}
