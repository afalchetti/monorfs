// FileParser.cs
// File to structure utility definitions
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
using System.IO;
using System.IO.Compression;
using System.Collections.Generic;

using AForge;
using Accord.Math;

using Microsoft.Xna.Framework;

using TimedState        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedArray        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedTrajectory   = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<System.Tuple<double, double[]>>>>;
using TimedMapModel     = System.Collections.Generic.List<System.Tuple<double, monorfs.Map>>;
using TimedMeasurements = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<double[]>>>;
using TimedMessage      = System.Collections.Generic.List<System.Tuple<double, string>>;

namespace monorfs
{
/// <summary>
/// Reads structures from files.
/// </summary>
public static class FileParser
{
	/// <summary>
	/// Get a trajectory history from a formatted string descriptor.
	/// </summary>
	/// <param name="descriptor">Formatted trajectory vectors moving through time.</param>
	/// <param name="dim">Expected state dimension.</param>
	/// <param name="filterhistory">If true, show the filtered trajectory history, otherwise, the smooth one,
	/// i.e. the smooth history may change retroactively from future knowledge. Note however that the
	/// recorded vehicle may not support smoothing and may perform the same in both modes.
	/// Note also that this is not the algorithm mode, only the visualization; the algorithm
	/// could still take past information into account, but it won't be visualized.</param> 
	/// <returns>The trajectory, list of vectors representing state as a function of discrete time
	/// (time is delivered as the first entry of each tuple).</returns>
	public static TimedTrajectory TrajectoryHistoryFromDescriptor(string descriptor, int dim, bool filterhistory = false)
	{
		string[]        frames   = descriptor.Split(new string[] {"\n|\n"}, StringSplitOptions.RemoveEmptyEntries);
		TimedTrajectory history  = new TimedTrajectory();
		TimedState      filtered = new TimedState();

		foreach (string frame in frames) {
			string[] lines = frame.Split(new char[] {'\n'}, StringSplitOptions.RemoveEmptyEntries);
			double time    = -1;

			try {
				time = double.Parse(lines[0]);
			}
			catch (FormatException) {
				throw new FormatException("bad trajectory format: missing time");
			}

			TimedState trajectory = TimedArrayFromDescriptor(lines.Submatrix(1, lines.Length - 1), dim);

			// filtering, disregard history updates (i.e. future affecting the past)
			if (filterhistory) {
				filtered.Add(trajectory[trajectory.Count - 1]);
				history.Add(Tuple.Create(time, new TimedState(filtered)));
			}
			else {
				history.Add(Tuple.Create(time, trajectory));
			}
		}

		return history;
	}

	/// <summary>
	/// Get a timed array structure (e.g. the trajectory) from a formatted string descriptor.
	/// </summary>
	/// <param name="lines">Array of formatted arrays moving through time.</param>
	/// <param name="dim">Expected array dimension.</param>
	/// <returns>The trajectory, list of arrays as a function of discrete time
	/// (time is delivered as the first entry of each tuple).</returns>
	public static TimedArray TimedArrayFromDescriptor(string[] lines, int dim)
	{
		TimedArray array = new TimedArray();

		foreach (string line in lines) {
			double[] values = ParseDoubleList(line);

			if (values.Length != dim + 1) {  // state + time
				throw new FormatException("wrong state dimension");
			}

			array.Add(Tuple.Create(values[0], values.Submatrix(1, values.Length - 1)));
		}

		return array;
	}

	/// <summary>
	/// Get a map trajectory (or history) from a formatted string descriptor.
	/// </summary>
	/// <param name="descriptor">Formatted map descriptor moving through time.</param>
	/// <param name="dim">Expected world dimension.</param>
	/// <returns>The map history, list of vectors representing map model components as a function of discrete time
	/// (time is delivered as the first entry of each tuple).</returns>
	public static TimedMapModel MapHistoryFromDescriptor(string descriptor, int dim)
	{
		string[]      frames  = descriptor.Split(new string[] {"\n|\n"}, StringSplitOptions.RemoveEmptyEntries);
		TimedMapModel history = new TimedMapModel();

		foreach (string frame in frames) {
			string[] lines = frame.Split(new char[] {'\n'}, StringSplitOptions.RemoveEmptyEntries);
			double time    = -1;

			try {
				time = double.Parse(lines[0]);
			}
			catch (FormatException) {
				throw new FormatException("bad map format: missing time");
			}

			history.Add(Tuple.Create(time, MapFromDescriptor(lines.Submatrix(1, lines.Length - 1), dim)));
		}

		return history;
	}

	/// <summary>
	/// Get a map model from a formatted string descriptor.
	/// </summary>
	/// <param name="lines">Formatted map descriptor as an array of lines.</param>
	/// <param name="dim">Expected world dimension.</param>
	/// <returns>The map model as a vector of gaussian components.</returns>
	public static Map MapFromDescriptor(string[] lines, int dim)
	{
		Map map = new Map(dim);

		for (int i = 0; i < lines.Length; i++) {
			Gaussian component = ParseGaussianDescriptor(lines[i]);
			map.Add(component);

			if (component.Mean.Length != dim) {
				throw new FormatException("wrong gaussian dimension");
			}
		}

		return map;
	}

	/// <summary>
	/// Get a measurement history from a formatted string descriptor.
	/// </summary>
	/// <param name="descriptor">Formatted measurement history descriptor through time.</param>
	/// <param name="dim">Expected measurement dimension.</param>
	/// <returns>The measurement history, list of vectors representing point measurements as a function of discrete time
	/// (time is delivered as the first entry of each tuple).</returns>
	public static TimedMeasurements MeasurementsFromDescriptor(string descriptor, int dim)
	{
		string[]          frames  = descriptor.Split('\n');
		TimedMeasurements history = new TimedMeasurements();

		foreach (string frame in frames) {
			string[] parts = frame.Split(':');
			double time    = -1;

			if (parts.Length != 2) {
				throw new FormatException("bad measurement format: no ':' delimiter found");
			}

			try {
				time = double.Parse(parts[0]);
			}
			catch (FormatException) {
				throw new FormatException("bad measurement format: missing time");
			}

			string[]       points       = parts[1].Split(';');
			List<double[]> measurements = new List<double[]>();

			foreach (string point in points) {
				if (point == "") {
					continue;
				}

				string[] strcomps   = point.Split(' ');
				double[] components = new double[strcomps.Length];

				if (components.Length != dim) {
					throw new FormatException("wrong measurement dimension");
				}

				for (int i = 0; i < strcomps.Length; i++) {
					try {
						components[i] = double.Parse(strcomps[i]);
					}
					catch (FormatException) {
						throw new FormatException("bad measurement format: invalid point");
					}
				}

				measurements.Add(components);
			}

			history.Add(Tuple.Create(time, measurements));
		}

		return history;
	}

	/// <summary>
	/// Get a timed message list from a formatted string descriptor.
	/// </summary>
	/// <param name="lines">Array of messages moving through time.</param>
	/// <returns>Timed message list.</returns>
	public static TimedMessage TimedMessageFromDescriptor(string[] lines)
	{
		TimedMessage array = new TimedMessage();

		foreach (string line in lines) {
			string[] values = line.Split(new char[] {' '}, 2);
			double time = 0;

			try {
				time = double.Parse(values[0]);
			}
			catch (FormatException) {
				throw new FormatException("the TimedMessage descriptor '" + line + "' is malformed");
			}

			array.Add(Tuple.Create(time, values[1]));
		}

		return array;
	}

	/// <summary>
	/// Create a command list from a descriptor string array.
	/// </summary>
	/// <param name="commandstr">Command descriptor array.</param>
	/// <returns>Command list.</returns>
	public static List<double[]> CommandsFromDescriptor(string[] commandstr)
	{
		List<double[]> commands = new List<double[]>(commandstr.Length);

		for (int i = 0; i < commandstr.Length; i++) {
			commands.Add(ParseDoubleList(commandstr[i]));
			// the item structure is {d1, d2, d3.... (odometry To/FromLinear()) Slam/Mapping Switch}
			// Last item: -1 => start mapping; 0 => nothing; 1 => start SLAM
		}

		return commands;
	}

	/// <summary>
	/// Get an array of doubles from a space-separated string descriptor.
	/// </summary>
	/// <param name="descriptor">String representing the list. Separated by spaces.</param>
	/// <returns>The double array.</returns>
	public static double[] ParseDoubleList(string descriptor) {
		string[] values = descriptor.Split(' ');
		double[] point  = new double[values.Length];
		
		try {
			for (int i = 0; i < point.Length; i++) {
				point[i] = double.Parse(values[i]);
			}
		}
		catch (FormatException) {
			throw new FormatException("the double descriptor '" + descriptor + "' is malformed");
		}

		return point;
	}

	/// <summary>
	/// Get an gaussian component from a formatted string descriptor.
	/// </summary>
	/// <param name="descriptor">String representing the gaussian in linearized form.</param>
	/// <returns>The gaussian object.</returns>
	public static Gaussian ParseGaussianDescriptor(string descriptor) {
		string[]   parts = descriptor.Split(';');
		double[]   mean;
		double[][] covariance;
		double     weight;

		try {
			weight = double.Parse(parts[0]);
			
			string[] meanvals = parts[1].Split(' ');
			string[] covvals  = parts[2].Split(' ');

			if (covvals.Length != meanvals.Length * meanvals.Length) {
				throw new FormatException("covariance has the wrong size");
			}

			mean       = new double[meanvals.Length];
			covariance = new double[meanvals.Length][];
			
			for (int i = 0; i < mean.Length; i++) {
				mean[i] = double.Parse(meanvals[i]);
			}
			
			int h = 0;
			for (int i = 0; i < mean.Length; i++) {
				covariance[i] = new double[meanvals.Length];

				for (int k = 0; k < mean.Length; k++) {
					covariance[i][k] = double.Parse(covvals[h++]);
				}
			}
		}
		catch (FormatException) {
			throw new FormatException("the double descriptor '" + descriptor + "' is malformed");
		}

		return new Gaussian(mean, covariance, weight);
	}
}
}
