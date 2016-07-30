// Pose3DTest.cs
// Pose3D unit tests
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

using NUnit.Framework;

namespace monorfs.Test
{
/// <summary>
/// Pose3D unit tests.
/// </summary>
[TestFixture]
class Pose3DTest
{
	Pose3D a;
	Pose3D b;
	double[] odometry;
	double[] odometry2;

	[SetUp]
	public void setup() {
		Quaternion arot = Quaternion.CreateFromYawPitchRoll(0.4, 1.6, 0.1).Normalize();
		Quaternion brot = Quaternion.CreateFromYawPitchRoll(0.4, 0.6, 0.5).Normalize();

		a = new Pose3D(new double[3] {0.1,  0.3, 0.2}, arot);
		b = new Pose3D(new double[3] {0.5, -0.4, 0.7}, brot);

		odometry  = new double[6] {0.12, 2.17, 1.03, 0.21, 0.05, 1.05};
		odometry2 = new double[6] {0.13, 0.09, 0.05, 0.02, 1.20, 0.20};
	}

	[TearDown]
	public void teardown()
	{
	}

	[Test]
	public void AddSubtract()
	{
		double[] recovered = a.AddOdometry(odometry).DiffOdometry(a);

		Assert.AreEqual(odometry[0], recovered[0], 1e-3);
		Assert.AreEqual(odometry[1], recovered[1], 1e-3);
		Assert.AreEqual(odometry[2], recovered[2], 1e-3);
		Assert.AreEqual(odometry[3], recovered[3], 1e-3);
		Assert.AreEqual(odometry[4], recovered[4], 1e-3);
		Assert.AreEqual(odometry[5], recovered[5], 1e-3);
	}

	[Test]
	public void SubtractAdd()
	{
		Pose3D recovered = b.AddOdometry(a.DiffOdometry(b));

		Assert.AreEqual(a.State[0], recovered.State[0], 1e-3);
		Assert.AreEqual(a.State[1], recovered.State[1], 1e-3);
		Assert.AreEqual(a.State[2], recovered.State[2], 1e-3);
		Assert.AreEqual(a.State[3], recovered.State[3], 1e-3);
		Assert.AreEqual(a.State[4], recovered.State[4], 1e-3);
		Assert.AreEqual(a.State[5], recovered.State[5], 1e-3);
		Assert.AreEqual(a.State[6], recovered.State[6], 1e-3);
	}

	[Test]
	public void Jacobian()
	{
		const double ds = 1e-5;
		Pose3D       f  = a.AddOdometry(odometry2);

		double[][] jacobian  = a.AddOdometryJacobian(odometry2);
		double[][] numerical = new double[6][];

		for (int i = 0; i < 6; i++) {
			double[] da = new double[6];
			da[i]       = ds;

			Pose3D   a2 = a.AddGlobal(da);
			//Pose3D   f2 = a.AddOdometry(Pose3D.Identity.FromLinear(odometry2).AddGlobal(da).ToLinear());
			Pose3D   f2 = a2.AddOdometry(odometry2);
			double[] df = f2.SubtractGlobal(f);

			numerical[i] = df.Divide(ds);
		}

		numerical = numerical.Transpose();

		Console.WriteLine(jacobian.ToString("f6"));
		Console.WriteLine("---");
		Console.WriteLine(numerical.ToString("f6"));

		Assert.IsTrue(numerical.IsEqual(jacobian, 1e-4));
	}
}
}
