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

using Accord.Math;

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
	}

	/// <summary>
	/// Rotation quaternion vector component.
	/// </summary>
	public double[] K {
		get { return new double[3] { Orientation.X, Orientation.Y, Orientation.Z }; }
	}

	/// <summary>
	/// Angle in angle-axis representation.
	/// </summary>
	public double Angle
	{
		get { return Orientation.Angle; }
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

	/// <summary>
	/// Number of state coordinates.
	/// </summary>
	public int StateSize { get { return 7; } }

	/// <summary>
	/// Number of odometry coordinates.
	/// </summary>
	public int OdometrySize { get { return 6; } }

	/// <summary>
	/// Global constants initialization.
	/// </summary>
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

		double w = state[3];
		double x = state[4];
		double y = state[5];
		double z = state[6];

		double a = 1.0 / Math.Sqrt(w * w + x * x + y * y + z  *z);

		Orientation = new Quaternion(a * w, a * x, a * y, a * z);
	}

	/// <summary>
	/// Construct a pose from its location and orientation.
	/// </summary>
	/// <param name="location">Location.</param>
	/// <param name="orientation">Orientation.</param>
	public Pose3D(double[] location, Quaternion orientation)
	{
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
	/// Create a pose from its state.
	/// </summary>
	/// <param name="state">State.</param>
	/// <returns>Pose.</returns>
	public Pose3D FromState(double[] state)
	{
		if (state.Length != 7) {
			throw new ArgumentException("State should have exactly seven parameters.");
		}

		return new Pose3D(state);
	}

	/// <summary>
	/// Deep clone.
	/// </summary>
	/// <returns>Pose clone.</returns>
	public Pose3D DClone()
	{
		return new Pose3D(this);
	}

	/// <summary>
	/// Get the identity pose, with loc = 0 and rot = I.
	/// </summary>
	/// <returns>Identity pose.</returns>
	public Pose3D IdentityP()
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
	public Pose3D FromOdometry(double[] linear)
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
	public Pose3D FromLinear(double[] linear)
	{
		return Identity.Add(linear);
	}

	/// <summary>
	/// Add a linear vector in Lie space to the the pose around its pose manifold.
	/// It uses global coordinates, where translation and rotation are independent.
	/// </summary>
	/// <param name="delta">Odometry delta.</param>
	public Pose3D AddGlobal(double[] delta)
	{
		Quaternion neworientation = Orientation.Add(new double[3] {delta[3], delta[4], delta[5]});
		double[]   location       = new double[3] {X + delta[0], Y + delta[1], Z + delta[2]};

		return new Pose3D(location, neworientation.Normalize());
	}

	/// <summary>
	/// Find the linear vector in semi-Lie space that transforms another pose into this one.
	/// It uses global coordinates, where translation and rotation are independent.
	/// </summary>
	/// <param name="origin">Origin pose.</param>
	public double[] SubtractGlobal(Pose3D origin)
	{
		double[] dq = this.Orientation.Subtract(origin.Orientation);
		double[] dx = this.Location.Subtract(origin.Location);

		return new double[6] {dx[0], dx[1], dx[2], dq[0], dq[1], dq[2]};
	}

	/// <summary>
	/// Add a linear vector in semi-Lie space to the the pose around its pose manifold.
	/// </summary>
	/// <param name="delta">Odometry delta.</param>
	public Pose3D Add(double[] delta)
	{
		Quaternion neworientation = Orientation.Add(new double[3] {delta[3], delta[4], delta[5]});
		Quaternion dlocation      = Orientation * new Quaternion(0, delta[0], delta[1], delta[2]) * Orientation.Conjugate();

		double[]   location    = new double[3] {X + dlocation.X, Y + dlocation.Y, Z + dlocation.Z};
		Quaternion orientation = neworientation.Normalize();

		return new Pose3D(location, orientation);
	}

	/// <summary>
	/// Find the linear vector in semi-Lie space that transforms another pose into this one.
	/// </summary>
	/// <param name="origin">Origin pose.</param>
	public double[] Subtract(Pose3D origin)
	{
		Quaternion dq       = origin.Orientation.Conjugate() * this.Orientation;
		double[]   dxglobal = this.Location.Subtract(origin.Location);
		Quaternion dx       = origin.Orientation.Conjugate() *
		                          new Quaternion(0, dxglobal[0], dxglobal[1], dxglobal[2]) *
		                          origin.Orientation;

		double[] lie = dq.ToLinear();

		return new double[6] {dx.X, dx.Y, dx.Z, lie[0], lie[1], lie[2]};
	}

	/// <summary>
	/// Move the pose by an odometry delta.
	/// </summary>
	/// <param name="delta">Odometry delta.</param>
	public Pose3D AddOdometry(double[] delta)
	{
		// equivalent to: (inlined and simplified for performance)
		// double[] lie = Quaternion.Identity.FromLinear(new double[3] {delta[3], delta[4], delta[5]}).Sqrt().ToLinear();
		// double[] dq  = new double[6] {0, 0, 0, lie[0], lie[1], lie[2]};
		// double[] dx  = new double[6] {delta[0], delta[1], delta[2], 0, 0, 0};
		// 
		// return this.Add(dq).Add(dx).Add(dq);

		Quaternion dorientation   = Quaternion.Identity.FromLinear(new double[3] {delta[3], delta[4], delta[5]});
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
	public double[] DiffOdometry(Pose3D origin)
	{
		// equivalent to: (inlined and simplified for performance)
		// double[] lie  = (origin.Orientation.Conjugate() * this.Orientation).ToLinear();
		// double[] sqrt = new double[6] {0, 0, 0, lie[0]/2, lie[1]/2, lie[2]/2};
		// double[] dx   = (this.Add((-1.0).Multiply(sqrt))).Subtract(origin.Add(sqrt));
		// 
		// return new double[6] {dx[0], dx[1], dx[2], lie[0], lie[1], lie[2]};

		Quaternion dq          = origin.Orientation.Conjugate() * this.Orientation;
		Quaternion middelta    = dq.Sqrt();
		Quaternion midrotation = origin.Orientation * middelta;
		double[]   dxglobal    = this.Location.Subtract(origin.Location);
		Quaternion dx          = midrotation.Conjugate() *
		                             new Quaternion(0, dxglobal[0], dxglobal[1], dxglobal[2]) *
		                             midrotation;
		
		double[] lie = dq.ToLinear();
		
		return new double[6] {dx.X, dx.Y, dx.Z, lie[0], lie[1], lie[2]};
	}

	/// <summary>
	/// Obtain a linearization for the Add operation around the given delta.
	/// </summary>
	/// <param name="delta">Odometry reading.</param>
	/// <returns>Jacobian.</returns>
	public double[][] AddJacobian(double[] delta)
	{
		double[][] Crot   = Orientation.ToMatrix();

		double[][] zero     = MatrixExtensions.Zero(3);
		double[][] identity = new double[3][] { new double[3] {1, 0, 0},
		                                        new double[3] {0, 1, 0},
		                                        new double[3] {0, 0, 1} };

		return Crot.Concatenate(zero).VConcatenate(
		       zero.Concatenate(identity));
	}

	/// <summary>
	/// Obtain a linearization for the Subtract operation around this pose.
	/// </summary>
	/// <param name="origin">Odometry reading.</param>
	/// <returns>Jacobian.</returns>
	public double[][] SubtractJacobian(Pose3D origin)
	{
		double[][] Crot   = origin.Orientation.ToMatrix();

		double[][] zero     = MatrixExtensions.Zero(3);
		double[][] identity = new double[3][] { new double[3] {1, 0, 0},
		                                        new double[3] {0, 1, 0},
		                                        new double[3] {0, 0, 1} };

		return Crot.Transpose().Concatenate(zero).VConcatenate(
		       zero            .Concatenate(identity));
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
		Quaternion dorientation = Quaternion.Identity.Add(new double[3] {delta[3], delta[4], delta[5]});
		Quaternion sqrt         = dorientation.Sqrt();

		double[][] Cmid   = (Orientation * sqrt).ToMatrix();
		double[][] Cdelta = dorientation.ToMatrix();
		double[][] Csqrt  = sqrt.ToMatrix();

		double[][] crossdX  = Util.CrossProductMatrix(new double[3] {delta[0], delta[1], delta[2]});

		double[][] identity = new double[3][] { new double[3] {1, 0, 0},
		                                        new double[3] {0, 1, 0},
		                                        new double[3] {0, 0, 1} };

		double[][] dxdq = (-1.0).Multiply(Cmid).Multiply(crossdX).Multiply(Csqrt.Transpose());

		return identity                .Concatenate(dxdq).VConcatenate(
		       MatrixExtensions.Zero(3).Concatenate(Cdelta.Transpose()));
	}

	/// <summary>
	/// Add the keyboard input to an odometry reading.
	/// </summary>
	/// <param name="odometry">Original odometry reading.</param>
	/// <param name="keyboard">Keyboard input in canonical 6-axis representation:
	/// (dlocx, dlocy, dlocz, dpitch, dyaw, droll).</param>
	/// <returns>Modified odometry reading.</returns>
	public double[] AddKeyboardInput(double[] odometry, double[] keyboard)
	{
		return new double[6] {odometry[0] + 0.02 * keyboard[0],
		                      odometry[1] + 0.02 * keyboard[1],
		                      odometry[2] + 0.02 * keyboard[2],
		                      odometry[3] - 0.1  * keyboard[3],
		                      odometry[4] - 0.1  * keyboard[4],
		                      odometry[5] + 0.1  * keyboard[5]};
	}

	/// <summary>
	/// Get the transformation matrix representation of this pose.
	/// </summary>
	/// <returns>Pose matrix.</returns>
	public double[][] ToMatrix()
	{
		double[][] Crot = Orientation.ToMatrix();

		return new double[4][]
		{
			new double[4] {Crot[0][0], Crot[0][1], Crot[0][2], X},
			new double[4] {Crot[1][0], Crot[1][1], Crot[1][2], Y},
			new double[4] {Crot[2][0], Crot[2][1], Crot[2][2], Z},
			new double[4] {         0,          0,          0, 1}
		};
	}

	/// <param name="a">First pose.</param>
	/// <param name="b">Second pose.</param>
	public static bool operator ==(Pose3D a, Pose3D b)
	{
		return a.Equals(b);
	}

	/// <param name="a">First pose.</param>
	/// <param name="b">Second pose.</param>
	public static bool operator !=(Pose3D a, Pose3D b)
	{
		return !(a == b);
	}

	/// <summary>
	/// Efficient equality comparer with another 3D pose.
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
	/// Get a unique code that is equal for any two equal 3D poses.
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
	/// Create a vehicle descriptor string.
	/// </summary>
	/// <param name="format">String format for double values.</param>
	/// <returns>Simulated vehicle descriptor string.</returns>
	public string ToString(string format)
	{
		return "[(" + X.ToString(format) + ", " + Y.ToString(format) + ", " + Z.ToString(format) + "); " +
		       Orientation.ToString(format) + "]";
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
