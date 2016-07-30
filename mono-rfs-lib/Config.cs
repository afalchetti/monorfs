// Config.cs
// Global configurations
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
using System.Reflection;

using Accord.Math;

namespace monorfs
{
/// <summary>
/// Global configuration constants.
/// </summary>
public static class Config
{
	// General
	public static int NParallel = 8;  // number of permitted threads


	// Manipulator
	public static double AxisLimit = 10.0;

	// Simulation
	public static TimeSpan MeasureElapsed = new TimeSpan(10000000/30);
	public static float[]  MapClip        = new float[4] {-6, 6, -3, 3};
	public static bool     UseOdometry    = true;

	// Vehicle
	public static double[][] MotionCovariance = new double[6][] { new double[6] {5e-3, 0, 0, 0, 0, 0},
	                                                              new double[6] {0, 5e-3, 0, 0, 0, 0},
	                                                              new double[6] {0, 0, 5e-3, 0, 0, 0},
	                                                              new double[6] {0, 0, 0, 2e-4, 0, 0},
	                                                              new double[6] {0, 0, 0, 0, 2e-4, 0},
	                                                              new double[6] {0, 0, 0, 0, 0, 2e-4} };
	
	public static double[][] MotionCovarianceQ;
	public static double[][] MotionCovarianceL;
	
	public static double[][] MeasurementCovariance = new double[3][] { new double[3] {2e-0, 0, 0},
	                                                                   new double[3] {0, 2e-0, 0},
	                                                                   new double[3] {0, 0, 1e-3} };
	
	// SimulatedVehicle
	public static double   DetectionProbability = 0.9;
	public static double   ClutterDensity       = 3e-7;
	public static bool     PerfectStill         = false;
	public static double[] VisibilityRamp       = new double[3] {3 * MeasurementCovariance[0][0], 
	                                                             3 * MeasurementCovariance[1][1],
	                                                             3 * MeasurementCovariance[2][2]};

	// KinectVehicle
	public static int  KinectDelta    = 2;
	public static bool KeypointFilter = true;

	// Map
	public static double DensityDistanceThreshold = 1;//10*Math.Sqrt(BirthCovariance[0][0]);

	// Navigator
	public static bool ShowVisible = false;

	// PHDNavigator
	public static double[][] BirthCovariance = new double[3][] { new double[3] {1e-3, 0, 0},
	                                                             new double[3] {0, 1e-3, 0},
	                                                             new double[3] {0, 0, 1e-3} };
	public static double BirthWeight          = 0.05;
	public static double MinWeight            = 1e-2;
	public static double MinEffectiveParticle = 0.2;
	public static int    MaxQuantity          = 200;
	public static double MergeThreshold       = 3e0;
	public static double ExplorationThreshold = 1e-5;
	public static bool   RenderAllParticles   = true;

	public static double MotionCovarianceMultiplier      = 1.0;
	public static double MeasurementCovarianceMultiplier = 1.0;
	public static double NavigatorPD                     = DetectionProbability;
	public static double NavigatorClutterDensity         = ClutterDensity;

	// LoopyPHDNavigator
	public static double GradientAscentRate = 0.03;

	// ISAM2Navigator
	public static double MatchThreshold       = 3.0;
	public static int    NewLandmarkThreshold = 3;
	public static DataAssociationAlgorithm DAAlgorithm = DataAssociationAlgorithm.Mahalanobis;

	// OdometryNavigator
	public static double OdometryMergeThreshold = 1e-2;

	/// <summary>
	/// Static constructor. Fill missing fields.
	/// </summary>
	static Config() {
		Util.GetMotionCovariances(MotionCovariance, out MotionCovarianceQ, out MotionCovarianceL);
	}

	/// <summary>
	/// Read configuration instructions from file.
	/// Any missing parameter will be left as is.
	/// </summary>
	/// <param name="filename">File name.</param>
	public static void FromFile(string filename)
	{
		FieldInfo[] fields = typeof (Config).GetFields(BindingFlags.Static | BindingFlags.Public);
		string[]    lines  = System.IO.File.ReadAllLines(filename);

		foreach (string line in lines)
		{
			string[] keypair = line.Split(new char[] {':'}, 2);

			if (keypair.Length != 2) {
				Console.WriteLine("Skipped malformed configuration line:\n'" + line + "'");
				continue;
			}

			string fieldname = keypair[0].Trim();
			string strvalue  = keypair[1].Trim();

			foreach (FieldInfo field in fields) {
				if (fieldname == field.Name) {
					if (field.FieldType == typeof (double[][])) {
						field.SetValue(null, Matrix.ParseJagged(strvalue,
							new OctaveMatrixFormatProvider(DefaultMatrixFormatProvider.CurrentCulture)));
					}
					else if (field.FieldType == typeof (TimeSpan)) {
						double seconds = double.Parse(strvalue);
						field.SetValue(null, new TimeSpan((long) (10000000 * seconds)));
					}
					else if (field.FieldType == typeof (DataAssociationAlgorithm)) {
						field.SetValue(null, Enum.Parse(typeof (DataAssociationAlgorithm), strvalue));
					}
					else if (field.FieldType == typeof (int)) {
						field.SetValue(null, int.Parse(strvalue));
					}
					else if (field.FieldType == typeof (double)) {
						field.SetValue(null, double.Parse(strvalue));
					}
					else if (field.FieldType == typeof (bool)) {
						field.SetValue(null, bool.Parse(strvalue));
					}
					else {
						Console.WriteLine("Skipped field of unknown type: " + field.Name);
						continue;
					}

					break;
				}
			}
		}

		// fill missing fields
		Util.GetMotionCovariances(MotionCovariance, out MotionCovarianceQ, out MotionCovarianceL);
	}

	/// <summary>
	/// Get a string representation for the entire configuration.
	/// </summary>
	public static new string ToString()
	{
		FieldInfo[] fields = typeof (Config).GetFields(BindingFlags.Static | BindingFlags.Public);
		string[]    lines  = new string[fields.Length];
		int         i      = 0;

		foreach (FieldInfo field in fields)
		{
			string repr = "";

			if (field.FieldType == typeof (double[][])) {
				double[][] value = (double[][]) field.GetValue(null);

				repr = value.ToString(new OctaveMatrixFormatProvider(
				                              DefaultMatrixFormatProvider.CurrentCulture));
			}
			else if (field.FieldType == typeof (double[])) {
				double[] value = (double[]) field.GetValue(null);

				repr = value.ToString(new OctaveMatrixFormatProvider(
				                              DefaultMatrixFormatProvider.CurrentCulture));
			}
			else if (field.FieldType == typeof (float[])) {
				float[] value = (float[]) field.GetValue(null);

				repr = value.ToString(new OctaveMatrixFormatProvider(
				                              DefaultMatrixFormatProvider.CurrentCulture));
			}
			else if (field.FieldType == typeof (TimeSpan)) {
				repr = ((TimeSpan) field.GetValue(null)).TotalSeconds.ToString();
			}
			else {
				repr = field.GetValue(null).ToString().Replace('\n', ' ');
			}

			lines[i] = field.Name + ": " + repr;

			i++;
		}

		return string.Join("\n", lines);
	}
}
}