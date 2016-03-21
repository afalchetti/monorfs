// UtilTest.cs
// Utility routines unit tests
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

using NUnit.Framework;

namespace monorfs.Test
{
/// <summary>
/// Utility routines unit tests.
/// </summary>
[TestFixture]
class UtilTest
{
	[SetUp]
	public void setup() {
	}

	[TearDown]
	public void teardown()
	{
	}

	[Test]
	public void Lie2Quat2Lie()
	{
		double[]   lie         = new double[6] {0.2, 0.4, 1.0, 2.1, 0.9, 0.1};
		Pose3D     linearpoint = new Pose3D(new double[7] {0.2, 0.3, 0.1, 0.5, 0.5, 0.5, 0.5});
		double[][] covariance  = Config.MotionCovarianceL;
		Gaussian   glie        = new Gaussian(lie, covariance, 1.0);
		Gaussian   recovered   = Util.Quat2Lie(Util.Lie2Quat(glie, linearpoint), linearpoint);

		for (int i = 0; i < 6; i++) {
			Assert.AreEqual(lie[i], recovered.Mean[i], 1e-3);
		}

		for (int i = 0; i < 6; i++) {
		for (int k = 0; k < 6; k++) {
			Assert.AreEqual(covariance[i][k], recovered.Covariance[i][k], 1e-3);
		}
		}
	}

	[Test]
	public void Quat2Lie2Quat()
	{
		double[]   quaternion  = new double[7] {0.7, 0.1, 1.9, 0.932908456, 0.207911691, 0.207911691, 0.207911691};
		Pose3D     linearpoint = new Pose3D(new double[7] {0.2, 0.3, 0.1, 0.866025404, 0.5, 0.0, 0.0});
		double[][] covariance  = Config.MotionCovarianceQ;
		Gaussian   gquat        = new Gaussian(quaternion, covariance, 1.0);
		Gaussian   recovered   = Util.Lie2Quat(Util.Quat2Lie(gquat, linearpoint), linearpoint);

		for (int i = 0; i < 6; i++) {
			Assert.AreEqual(quaternion[i], recovered.Mean[i], 1e-3);
		}

		for (int i = 0; i < 6; i++) {
			for (int k = 0; k < 6; k++) {
				Assert.AreEqual(covariance[i][k], recovered.Covariance[i][k], 1e-3);
			}
		}
	}
}
}
