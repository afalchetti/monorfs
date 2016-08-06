// LoopyPHDNavigatorTest.cs
// Unit tests for the loopy PHD algorithms
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

using Accord.Math;

using NUnit.Framework;

using TimedState        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedArray        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedMapModel     = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<monorfs.Gaussian>>>;
using TimedGaussian     = System.Collections.Generic.List<System.Tuple<double, monorfs.Gaussian>>;
using TimedMeasurements = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<double[]>>>;
using PHDNavigator      = monorfs.PHDNavigator<monorfs.PRM3DMeasurer, monorfs.Pose3D, monorfs.PixelRangeMeasurement>;
using LoopyPHDNavigator = monorfs.LoopyPHDNavigator<monorfs.PRM3DMeasurer, monorfs.Pose3D, monorfs.PixelRangeMeasurement>;
using SimulatedVehicle  = monorfs.SimulatedVehicle<monorfs.PRM3DMeasurer, monorfs.Pose3D, monorfs.PixelRangeMeasurement>;

namespace monorfs.Test
{
/// <summary>
/// GraphCombinatorics unit tests.
/// </summary>
[TestFixture]
class LoopyPHDNavigatorTest
{
	// LoopyPHDNavigator nav;
	double[][] identitycov;

	[SetUp]
	public void Setup()
	{
		// Vehicle           vehicle      = new SimulatedVehicle();
		// TimedState        trajectory   = new TimedState();
		// TimedArray        odometry     = new TimedArray();
		// TimedMeasurements measurements = new TimedMeasurements();

		// nav = new LoopyPHDNavigator(vehicle, trajectory, odometry, measurements);

		identitycov = new double[3][] { new double[3] {1, 0, 0},
		                                new double[3] {0, 1, 0},
		                                new double[3] {0, 0, 1} };
	}

	[Test]
	public void FitGaussianNoMap()
	{
		Pose3D   linearpoint  = new Pose3D();
		double[] estimate     = new double[6];
		Map      map          = new Map();
		var      measurements = new List<PixelRangeMeasurement>();

		double[][] covariance = MatrixExtensions.Zero(6).SingularInverse();
		Gaussian   expected   = new Gaussian(new double[6] {0, 0, 0, 0, 0, 0}, covariance, 1.0);

		Gaussian result = LoopyPHDNavigator.FitGaussian(estimate, measurements, map, linearpoint);

		Assert.IsTrue(expected.Equals(result, 1e-5));
	}

	[Test]
	public void FitGaussian()
	{
		Pose3D   linearpoint  = new Pose3D();
		double[] estimate     = new double[6];
		Map      map          = new Map();
		var      measurements = new List<PixelRangeMeasurement>();

		measurements.Add(new PixelRangeMeasurement(0, 0, 1));

		map.Add(new Gaussian(new double[3] {0, 0, 1}, identitycov, 1.0));

		// this expectation assumes a diagonal measurement error and the identity pose
		SimulatedVehicle dummy = new SimulatedVehicle();
		double a  = 1.0/dummy.MeasurementCovariance[0][0];
		double b  = 1.0/dummy.MeasurementCovariance[1][1];
		double c  = 1.0/dummy.MeasurementCovariance[2][2];
		double f2 = dummy.Measurer.VisionFocal * dummy.Measurer.VisionFocal;

		double[][] expinfo = new double[6][] { new double[6] {a*f2,     0, 0,     0, a*f2, 0},
		                                       new double[6] {   0,  b*f2, 0, -b*f2,    0, 0},
		                                       new double[6] {   0,     0, c,     0,    0, 0},
		                                       new double[6] {   0, -b*f2, 0,  b*f2,    0, 0},
		                                       new double[6] {a*f2,     0, 0,     0, a*f2, 0},
		                                       new double[6] {   0,     0, 0,     0,    0, 0} };

		double[][] expcov =  expinfo.SingularInverse();
		Gaussian   result = LoopyPHDNavigator.FitGaussian(estimate, measurements, map, linearpoint);

		Assert.IsTrue(new double[6] {0, 0, 0, 0, 0, 0}.IsEqual(result.Mean, 1e-5));
		Assert.IsTrue(expcov.IsEqualRelative(result.Covariance, 0.001));
	}

	[Test]
	public void FitGaussianWrongLocation()
	{
		Pose3D   linearpoint  = new Pose3D();
		double[] estimate     = new double[6];
		Map      map          = new Map();
		var      measurements = new List<PixelRangeMeasurement>();

		estimate[2] = -0.05;
		measurements.Add(new PixelRangeMeasurement(0, 0, 1));

		map.Add(new Gaussian(new double[3] {0, 0, 1.05}, identitycov, 1.0));

		// this expectation assumes a diagonal measurement error and the identity pose
		SimulatedVehicle dummy = new SimulatedVehicle();
		double a  = 1.0/dummy.MeasurementCovariance[0][0];
		double b  = 1.0/dummy.MeasurementCovariance[1][1];
		double c  = 1.0/dummy.MeasurementCovariance[2][2];
		double f2 = dummy.Measurer.VisionFocal * dummy.Measurer.VisionFocal;
		
		double[]   expstate = new double[7] {0, 0, 0.05, 1, 0, 0, 0};
		double[][] expinfo  = new double[6][] { new double[6] {a*f2,     0, 0,     0, a*f2, 0},
		                                        new double[6] {   0,  b*f2, 0, -b*f2,    0, 0},
		                                        new double[6] {   0,     0, c,     0,    0, 0},
		                                        new double[6] {   0, -b*f2, 0,  b*f2,    0, 0},
		                                        new double[6] {a*f2,     0, 0,     0, a*f2, 0},
		                                        new double[6] {   0,     0, 0,     0,    0, 0} };

		double[][] expcov = expinfo.SingularInverse();
		Gaussian   result = LoopyPHDNavigator.FitGaussian(estimate, measurements, map, linearpoint);

		Console.WriteLine(result.Covariance.ToString("e5"));
		Console.WriteLine(result.Mean.ToString("e5"));

		Assert.IsTrue(expstate.IsEqual(result.Mean, 1e-4));
		Assert.IsTrue(expcov.IsEqualRelative(result.Covariance, 0.001));
	}

	[Test]
	public void FitGaussianMultipleLandmarks()
	{
		Pose3D   linearpoint  = new Pose3D();
		double[] estimate     = new double[6] {-0.002, 0.003, 0.001, 0, 0, 0};
		Map      map          = new Map();
		var      measurements = new List<PixelRangeMeasurement>();

		measurements.Add(new PixelRangeMeasurement(115.16312, 0, 1.019803903));
		measurements.Add(new PixelRangeMeasurement(0,  57.58156, 1.004987562));
		measurements.Add(new PixelRangeMeasurement(28.79078,  0, 2.002498439));

		map.Add(new Gaussian(new double[3] {0.2, 0, 1}, identitycov, 1.0));
		map.Add(new Gaussian(new double[3] {0, 0.1, 1}, identitycov, 1.0));
		map.Add(new Gaussian(new double[3] {0.1, 0, 2}, identitycov,  1.0));

		Gaussian expected = new Gaussian(new double[7] {0, 0, 0, 1, 0, 0, 0},
		                                 new double[7][] { new double[7] {1, 0, 0, 0, 0, 0, 0},
		                                                   new double[7] {0, 1, 0, 0, 0, 0, 0},
		                                                   new double[7] {0, 0, 1, 0, 0, 0, 0},
		                                                   new double[7] {0, 0, 0, 1, 0, 0, 0},
		                                                   new double[7] {0, 0, 0, 0, 1, 0, 0},
		                                                   new double[7] {0, 0, 0, 0, 0, 1, 0},
		                                                   new double[7] {0, 0, 0, 0, 0, 0, 1} },
		                                 1.0);

		Gaussian result = LoopyPHDNavigator.FitGaussian(estimate, measurements, map, linearpoint);

		Console.WriteLine(result);

		Assert.AreEqual(expected, result);
	}

	[Test]
	public void FitMeasurementAlreadyFine()
	{
		PRM3DMeasurer measurer = new PRM3DMeasurer();
		Pose3D        pose0    = Pose3D.Identity;

		var      measurement = new PixelRangeMeasurement(0, 0, 1);
		double[] landmark    = new double[3] {0, 0, 1};

		Pose3D fitted = measurer.FitToMeasurement(pose0, measurement, landmark);

		Pose3D expected = new Pose3D(new double[3] {0, 0, 0}, Quaternion.Identity);

		Assert.IsTrue(expected.Equals(fitted, 1e-5));
	}

	[Test]
	public void FitMeasurementOnlyTranslation()
	{
		PRM3DMeasurer measurer = new PRM3DMeasurer();
		Pose3D        pose0    = Pose3D.Identity;

		var      measurement = new PixelRangeMeasurement(0, 0, 1);
		double[] landmark    = new double[3] {0, 0, 2.5};

		Pose3D fitted = measurer.FitToMeasurement(pose0, measurement, landmark);

		Pose3D expected = new Pose3D(new double[3] {0, 0, 1.5}, Quaternion.Identity);

		Assert.IsTrue(expected.Equals(fitted, 1e-5));
	}

	[Test]
	public void FitMeasurementOnlyRotation()
	{
		PRM3DMeasurer measurer = new PRM3DMeasurer();
		Pose3D        pose0    = Pose3D.Identity;

		var      measurement = new PixelRangeMeasurement(0, 0, 1);
		double[] landmark    = new double[3] {0, 1/Math.Sqrt(2), 1/Math.Sqrt(2)};

		Pose3D fitted = measurer.FitToMeasurement(pose0, measurement, landmark);

		Quaternion rotation = new Quaternion(Math.Cos(Math.PI / 4 / 2),
		                                     (-Math.Sin(Math.PI / 4 / 2)).Multiply(new double[3] {1, 0, 0}));

		Pose3D expected = new Pose3D(new double[3] {0, 0, 0}, rotation);

		Assert.IsTrue(expected.Equals(fitted, 1e-5));
	}

	[Test]
	public void FitMeasurementUnmeasurableLandmark()
	{
		PRM3DMeasurer measurer = new PRM3DMeasurer();
		Pose3D        pose0    = Pose3D.Identity;

		var      measurement = new PixelRangeMeasurement(0, 0, 1);
		double[] landmark    = new double[3] {0, 1, 0};

		Pose3D fitted   = measurer.FitToMeasurement(pose0, measurement, landmark);

		Quaternion rotation = new Quaternion(Math.Cos(Math.PI / 2 / 2),
		                                     (-Math.Sin(Math.PI / 2 / 2)).Multiply(new double[3] {1, 0, 0}));

		Pose3D expected = new Pose3D(new double[3] {0, 0, 0}, rotation);

		Assert.IsTrue(expected.Equals(fitted, 1e-5));
	}

	[Test]
	public void FitMeasurementGeneral()
	{
		PRM3DMeasurer measurer = new PRM3DMeasurer();
		Pose3D        pose0    = new Pose3D(new double[3] {2.0, 0.2, 0.1},
		                                    new Quaternion(1, 2, 3, 4).Normalize());

		var      measurement = new PixelRangeMeasurement(-120, 50, 1.3);
		double[] landmark    = new double[3] {0.1, -1.0, 1.2};

		Pose3D fpose = measurer.FitToMeasurement(pose0, measurement, landmark);

		var measured = measurer.MeasurePerfect(fpose, landmark);

		Assert.IsTrue(measurement.ToLinear().IsEqual(measured.ToLinear(), 1e-5));
	}

	public void LogLike()
	{
		SimulatedVehicle estimate     = new SimulatedVehicle();
		Map              map          = new Map();
		var              measurements = new List<PixelRangeMeasurement>();

		measurements.Add(new PixelRangeMeasurement(0, 0, 1));

		map.Add(new Gaussian(new double[3] {0, 0, 1}, identitycov, 1.0));
		
		double[]   x = new double[101];
		double[]   y = new double[101];
		double[][] loglike = new double[101][];

		for (int i = 0 ; i < x.Length; i++) {
			x[i] = (i / 100.0 - 0.5) / 100;
			Console.Write(x[i] + ", ");
		}

		Console.WriteLine();

		for (int k = 0 ; k < x.Length; k++) {
			y[k] = (k / 100.0 - 0.5) / 100;
			Console.Write(y[k] + ", ");
		}

		Console.WriteLine();

		for (int i = 0 ; i < x.Length; i++) {
			loglike[i] = new double[101];
			for (int k = 0 ; k < y.Length; k++) {
				double[] ds = new double[6];
				ds[0] = x[i];
				ds[4] = y[k];

				estimate.Pose = Pose3D.Identity.Add(ds);
				loglike[i][k] = Math.Log(PHDNavigator.SetLikelihood(measurements, map, estimate));
				Console.Write(loglike[i][k] + ", ");
			}
			Console.WriteLine();
		}

		Assert.True(false);
	}
}
}
