// UtilTest.cs
// Utility functions unit tests
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
using System.Linq;
using Microsoft.Xna.Framework;
using NUnit.Framework;

using Accord;
using Accord.Math;

namespace monorfs.Test
{
/// <summary>
/// Utility functions unit tests.
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
	public void ypr2qjacobian()
	{
		double[] yaw   = {0.0, Math.PI / 4, Math.PI / 3};
		double[] pitch = {0.0, Math.PI / 6, Math.PI / 3};
		double[] roll  = {0.0, Math.PI / 3, Math.PI / 4};

		double ds = 1e-3;

		for (int i = 0; i < yaw.Length; i++) {
			Quaternion q0         = Quaternion.CreateFromYawPitchRoll((float) yaw[i], (float) pitch[i], (float) roll[i]);
			double[][] jacobian   = Util.YPR2QJacobian(yaw[i], pitch[i], roll[i]);

			Quaternion dqy        = Quaternion.CreateFromYawPitchRoll((float) (yaw[i] + ds), (float) pitch[i], (float) roll[i]) - q0;
			Quaternion dqp        = Quaternion.CreateFromYawPitchRoll((float) yaw[i], (float) (pitch[i] + ds), (float) roll[i]) - q0;
			Quaternion dqr        = Quaternion.CreateFromYawPitchRoll((float) yaw[i], (float) pitch[i], (float) (roll[i] + ds)) - q0;

			double[][] numerical  = new double[4][] { new double[3] {dqy.W, dqp.W, dqr.W},
			                                          new double[3] {dqy.X, dqp.X, dqr.X},
			                                          new double[3] {dqy.Y, dqp.Y, dqr.Y},
			                                          new double[3] {dqy.Z, dqp.Z, dqr.Z}}.Divide(ds);

			for     (int x = 0; x < jacobian.Length; x++) {
				for (int y = 0; y < jacobian[0].Length; y++) {
					Assert.AreEqual(numerical[x][y], jacobian[x][y], 1e-3);
				}
			}


		}
	}
}
}
