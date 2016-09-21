// PHDNavigatorTest.cs
// Unit tests for the PHD algorithms
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
using System.Collections.Generic;

using AForge;
using Accord.Math;

using Microsoft.Xna.Framework;

using NUnit.Framework;

using TimedState        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedArray        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedMapModel     = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<monorfs.Gaussian>>>;
using TimedGaussian     = System.Collections.Generic.List<System.Tuple<double, monorfs.Gaussian>>;
using TimedMeasurements = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<double[]>>>;
using PHDNavigator      = monorfs.PHDNavigator<monorfs.Linear2DMeasurer, monorfs.LinearPose2D, monorfs.LinearMeasurement2D>;
using SimulatedVehicle  = monorfs.SimulatedVehicle<monorfs.Linear2DMeasurer, monorfs.LinearPose2D, monorfs.LinearMeasurement2D>;

namespace monorfs.Test
{
/// <summary>
/// PHDNavigator unit tests.
/// </summary>
[TestFixture]
class PHDNavigator2DTest
{
	SimulatedVehicle vehicle;
	PHDNavigator navigator;

	[SetUp]
	public void Setup()
	{
		Config.SetLinear2DDefaults();

		LinearPose2D     pose      = new LinearPose2D();
		List<double[]>   landmarks = new List<double[]>();
		Linear2DMeasurer measurer  = new Linear2DMeasurer(6.5);

		pose.X = 1;
		pose.Y = 2;

		landmarks.Add(new double[3] {2, 2,   0});
		landmarks.Add(new double[3] {3, 4,   0});
		landmarks.Add(new double[3] {3, 3.5, 0});

		vehicle   = new SimulatedVehicle(pose, landmarks, measurer);
		navigator = new PHDNavigator(vehicle, 1, true);

	}

	[TearDown]
	public void Teardown()
	{
		Config.SetPRM3DDefaults();
	}

	[Test]
	public void PredictInitial()
	{
		List<LinearMeasurement2D> measurements = new List<LinearMeasurement2D>();
		Map                       model        = new Map(3);
		List<double[]>            unexplored   = new List<double[]>();

		measurements.Add(new LinearMeasurement2D(2, 3));
		unexplored.Add(new double[3] {3, 5, 0});

		Map predicted = navigator.PredictConditional(measurements, vehicle, model, unexplored);

		List<Gaussian> listed = predicted.ToList();

		Assert.AreEqual(1, listed.Count);

		Assert.IsTrue(new double[3] {3, 5, 0}.IsEqual(listed[0].Mean,       1e-5));
		Assert.IsTrue(Config.BirthCovariance .IsEqual(listed[0].Covariance, 1e-5));
		Assert.IsTrue(Config.BirthWeight     .IsEqual(listed[0].Weight,     1e-5));
	}

	[Test]
	public void PredictKnown()
	{
		List<LinearMeasurement2D> measurements = new List<LinearMeasurement2D>();
		Map                       model        = new Map(3);
		List<double[]>            unexplored   = new List<double[]>();

		measurements.Add(new LinearMeasurement2D(2, 3));
		model.Add(new Gaussian(new double[3] {3, 5, 0}, Accord.Math.Matrix.JaggedIdentity(3), 1.0));

		Map            predicted = navigator.PredictConditional(measurements, vehicle, model, unexplored);
		List<Gaussian> mlist     = model.ToList();
		List<Gaussian> plist     = predicted.ToList();

		Assert.AreEqual(1, plist.Count);

		// should have done nothing (already explored)
		Assert.IsTrue(mlist[0].Mean      .IsEqual(plist[0].Mean,       1e-5));
		Assert.IsTrue(mlist[0].Covariance.IsEqual(plist[0].Covariance, 1e-5));
		Assert.IsTrue(mlist[0].Weight    .IsEqual(plist[0].Weight,     1e-5));
	}

	[Test]
	public void Correct()
	{
		List<LinearMeasurement2D> measurements = new List<LinearMeasurement2D>();
		Map                       model        = new Map(3);

		double[][] identity = Accord.Math.Matrix.JaggedIdentity(3);
		double     PD       = vehicle.PD;

		Gaussian comp1 = new Gaussian(new double[3] {3, 5, 0}, identity,                 0.8);
		Gaussian comp2 = new Gaussian(new double[3] {7, 5, 0}, (4.0).Multiply(identity), 1.4);

		measurements.Add(new LinearMeasurement2D(2, 3));
		measurements.Add(new LinearMeasurement2D(5, 3));
		model.Add(comp1);
		model.Add(comp2);

		// in Linear2D, the measurement covariance in measurement space
		// is the same as in the map space
		double[][] mcov = MatrixExtensions.Zero(3);

		mcov[0][0] = vehicle.MeasurementCovariance[0][0];
		mcov[0][1] = vehicle.MeasurementCovariance[0][1];
		mcov[1][0] = vehicle.MeasurementCovariance[1][0];
		mcov[1][1] = vehicle.MeasurementCovariance[1][1];

		Gaussian gz1 = new Gaussian(new double[3] {1 + 2, 2 + 3, 0}, mcov, 1.0);
		Gaussian gz2 = new Gaussian(new double[3] {1 + 5, 2 + 3, 0}, mcov, 1.0);

		Map            corrected = navigator.CorrectConditional(measurements, vehicle, model);
		List<Gaussian> clist     = corrected.ToList();
		List<Gaussian> explist   = new List<Gaussian>();

		Gaussian z11     = Gaussian.Multiply(gz1, comp1);
		Gaussian z12     = Gaussian.Multiply(gz1, comp2);
		Gaussian z21     = Gaussian.Multiply(gz2, comp1);
		Gaussian z22     = Gaussian.Multiply(gz2, comp2);
		double   sumw1   = z11.Weight +z12.Weight;
		double   sumw2   = z21.Weight + z22.Weight;
		double   clutter = vehicle.ClutterDensity;

		explist.Add(comp1.Reweight(0.8 * (1 - PD)));
		explist.Add(comp2.Reweight(1.4 * (1 - PD)));
		explist.Add(z11.Reweight(z11.Weight * PD / (clutter + PD * sumw1)));
		explist.Add(z12.Reweight(z12.Weight * PD / (clutter + PD * sumw1)));
		explist.Add(z21.Reweight(z21.Weight * PD / (clutter + PD * sumw2)));
		explist.Add(z22.Reweight(z22.Weight * PD / (clutter + PD * sumw2)));

		Assert.AreEqual(6, clist.Count);

		foreach (Gaussian expected in explist) {
			bool found = false;
			for (int i = 0; i < clist.Count; i++) {
				if (expected.Equals(clist[i], 1e-5)) {
					found = true;

					clist.RemoveAt(i);
					break;
				}
			}

			if (!found) {
				Assert.Fail("Component not found: " + expected);
			}
		}
	}

	[Test]
	public void Prune()
	{
		Map model      = new Map(3);
		Map bigones    = new Map(3);
		Map irrelevant = new Map(3);
		Map mergeable1 = new Map(3);
		Map mergeable2 = new Map(3);

		List<Gaussian> explist  = new List<Gaussian>();
		double[][]     identity = Accord.Math.Matrix.JaggedIdentity(3);
		double         mergedist = PHDNavigator.MergeThreshold;

		bigones.Add(new Gaussian(new double[3] {-12, -24, -54}, identity,                23.0));
		bigones.Add(new Gaussian(new double[3] {-80, -22, -12}, (4.0).Multiply(identity), 1.0));
		bigones.Add(new Gaussian(new double[3] {-63, -11, -95}, (0.1).Multiply(identity), 6.0));

		irrelevant.Add(new Gaussian(new double[3] {12, 24, 54}, identity,                 0.3  * PHDNavigator.MinWeight));
		irrelevant.Add(new Gaussian(new double[3] {80, 22, 12}, (4.0).Multiply(identity), 0.8  * PHDNavigator.MinWeight));
		irrelevant.Add(new Gaussian(new double[3] {63, 11, 95}, (0.1).Multiply(identity), 0.99 * PHDNavigator.MinWeight));
		irrelevant.Add(new Gaussian(new double[3] {23, 19, 73}, identity,                 0.0  * PHDNavigator.MinWeight));

		mergeable1.Add(new Gaussian(new double[3] {0, 0,             0}, identity, 1.0));
		mergeable1.Add(new Gaussian(new double[3] {0, mergedist,     0}, identity, 0.6));
		mergeable1.Add(new Gaussian(new double[3] {0, mergedist / 2, 0}, identity, 1.2));

		mergeable2.Add(new Gaussian(new double[3] {99 - mergedist / 6, 99, 99}, identity, 0.9));
		mergeable2.Add(new Gaussian(new double[3] {99, 99 - mergedist / 6, 99}, identity, 0.5));
		mergeable2.Add(new Gaussian(new double[3] {99, 99, 99 - mergedist / 6}, identity, 1.1));

		foreach (Gaussian component in bigones) {
			model.Add(component);
			explist.Add(component);
		}

		foreach (Gaussian component in irrelevant) {
			model.Add(component);
		}

		foreach (Gaussian component in mergeable1) {
			model.Add(component);
		}

		foreach (Gaussian component in mergeable2) {
			model.Add(component);
		}

		explist.Add(Gaussian.Merge(mergeable1.ToList()));
		explist.Add(Gaussian.Merge(mergeable2.ToList()));

		Map            pruned = navigator.PruneModel(model);
		List<Gaussian> plist  = pruned.ToList();

		Assert.AreEqual(5, plist.Count);

		foreach (Gaussian expected in explist) {
			bool found = false;
			for (int i = 0; i < plist.Count; i++) {
				if (expected.Equals(plist[i], 1e-5)) {
					found = true;

					plist.RemoveAt(i);
					break;
				}
			}

			if (!found) {
				Assert.Fail("Component not found: " + expected);
			}
		}
	}
}
}
