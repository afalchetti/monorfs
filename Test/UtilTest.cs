// UtilTest.cs
// Utility functions unit tests.
// Part of MonoRFS
//
// Copyright (C) 2015 Angelo Falchetti
// All rights reserved.
// Any use, reproduction, distribution or copy of this
// work via any medium is strictly forbidden without
// express written consent from the author.

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