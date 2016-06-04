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
	public static TimedTrajectory TrajectoryHistoryFromDescriptor(string descriptor, int dim = 7, bool filterhistory = false)
	{
		string[]        frames  = descriptor.Split(new string[] {"\n|\n"}, StringSplitOptions.RemoveEmptyEntries);
		TimedTrajectory history = new TimedTrajectory();
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
	public static TimedArray TimedArrayFromDescriptor(string[] lines, int dim = 7)
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
	public static TimedMapModel MapHistoryFromDescriptor(string descriptor, int dim = 3)
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
	public static Map MapFromDescriptor(string[] lines, int dim = 3)
	{
		Map map = new Map();

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
	public static TimedMeasurements MeasurementsFromDescriptor(string descriptor, int dim = 3)
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
			// the item structure is {dlocx, dlocy, dlocz, dyaw, dpitch, droll}
		}

		return commands;
	}

	/// <summary>
	/// Create a vehicle from a simulation file.
	/// </summary>
	/// <param name="descriptor">Scene descriptor text.</param>
	/// <returns>Simulated vehicle parsed from file.</returns>
	public static SimulatedVehicle VehicleFromSimFile(string descriptor)
	{
		Dictionary<string, List<string>> dict = Util.ParseDictionary(descriptor);

		double[] vehiclepose = ParseDoubleList(dict["pose"][0]);

		double[] focaldata = null;

		if (dict.ContainsKey("focal")) {
			focaldata = ParseDoubleList(dict["focal"][0]);

			if (focaldata.Length != 7) {
				throw new FormatException("Focal description must have exactly seven arguments: focal distance, rangemin, rangemax, filmleft, filmtop, filmwidth, filmheight");
			}
		}

		if (vehiclepose.Length != 7) {
			throw new FormatException("Vehicle description must have exactly seven arguments: x, y, z (location), theta, ax, ay, az (rotation axis)");
		}

		Pose3D pose = new Pose3D(new double[7] {vehiclepose[0], vehiclepose[1], vehiclepose[2], vehiclepose[3], vehiclepose[4], vehiclepose[5], vehiclepose[6]});

		List<double[]> maploc      = new List<double[]>();
		List<string>   mapdescript = dict["landmarks"];

		for (int i = 0; i < mapdescript.Count; i++) {
			double[] landmark = ParseDoubleList(mapdescript[i]);

			if (landmark.Length != 3) {
				throw new FormatException("Map landmarks must be 3D");
			}

			maploc.Add(landmark);
		}

		if (focaldata != null) {
			double    focal = focaldata[0];
			Range     clip  = new Range((float) focaldata[1], (float) focaldata[2]);
			Rectangle film  = new Rectangle((int) focaldata[3], (int) focaldata[4], (int) focaldata[5], (int) focaldata[6]);

			return new SimulatedVehicle(pose, maploc, focal, film, clip);
		}
		else {
			return new SimulatedVehicle(pose, maploc);
		}
	}

	/// <summary>
	/// Create a vehicle descriptor string.
	/// </summary>
	/// <param name="vehicle">Vehicle to be .</param>
	/// <returns>Simulated vehicle descriptor string.</returns>
	public static string VehicleToDescriptor(Vehicle vehicle)
	{
		string descriptor = "";

		descriptor += "pose\n\t"
		            + string.Join(" ", vehicle.Pose.State.Convert(x => x.ToString("g6"))) + "\n";

		descriptor += "focal\n\t"
		            + vehicle.VisionFocal    .ToString("g6") + " "
		            + vehicle.RangeClip.Min  .ToString("g6") + " "
		            + vehicle.RangeClip.Max  .ToString("g6") + " "
		            + vehicle.FilmArea.Left  .ToString("g6") + " "
		            + vehicle.FilmArea.Top   .ToString("g6") + " "
		            + vehicle.FilmArea.Width .ToString("g6") + " "
		            + vehicle.FilmArea.Height.ToString("g6") + "\n";

		descriptor += "landmarks\n\t" + string.Join("\n\t",
		                                    vehicle.Landmarks.ConvertAll(l =>
		                                        string.Join(" ", l.Convert(x => x.ToString("g6"))))) + "\n";

		return descriptor;
	}

	/// <summary>
	/// Create a vehicle from a record file.
	/// </summary>
	/// <param name="datafile">Vehicle descriptor file.</param>
	/// <param name="extrainfo">If true, provide additional information through output parameters;
	/// otherwise, they output null.</param>
	/// <param name="estimate">Trajectory estimate.</param>
	/// <param name="xodometry">Odometry readings.</param>
	/// <param name="xmeasurements">Measurement readings.</param>
	/// <returns>Prerecorded vehicle parsed from file.</returns>
	public static RecordVehicle VehicleFromRecord(string datafile, bool extrainfo, out TimedState estimate,
	                                              out TimedArray xodometry, out TimedMeasurements xmeasurements)
	{
		string tmpdir  = Util.TemporaryDir();
		string datadir = Path.Combine(tmpdir, "data");

		ZipFile.ExtractToDirectory(datafile, datadir);

		string scenefile      = Path.Combine(datadir, "scene.world");
		string trajectoryfile = Path.Combine(datadir, "trajectory.out");
		string estimatefile   = Path.Combine(datadir, "estimate.out");
		string odometryfile   = Path.Combine(datadir, "odometry.out");
		string measurefile    = Path.Combine(datadir, "measurements.out");
		string tagfile        = Path.Combine(datadir, "tags.out");

		if (!File.Exists(scenefile)) {
			throw new ArgumentException("Missing scene file");
		}

		if (extrainfo && !File.Exists(estimatefile)) {
			throw new ArgumentException("Missing estimate file");
		}

		if (!File.Exists(trajectoryfile)) {
			throw new ArgumentException("Missing trajectory file");
		}

		if (!File.Exists(odometryfile)) {
			throw new ArgumentException("Missing odometry file");
		}

		if (!File.Exists(measurefile)) {
			throw new ArgumentException("Missing measurement file");
		}

		if (!File.Exists(tagfile)) {
			tagfile = "";
		}

		RecordVehicle     explorer;
		TimedState        trajectory;
		TimedArray        odometry;
		TimedMeasurements measurements;
		TimedMessage      tags;

		SimulatedVehicle template = VehicleFromSimFile(File.ReadAllText(scenefile));

		trajectory   = TimedArrayFromDescriptor  (File.ReadAllLines(trajectoryfile), 7);
		odometry     = TimedArrayFromDescriptor  (File.ReadAllLines(odometryfile), 6);
		measurements = MeasurementsFromDescriptor(File.ReadAllText(measurefile), 3);

		if (!string.IsNullOrEmpty(tagfile)) {
			tags = TimedMessageFromDescriptor(File.ReadAllLines(tagfile));
		}
		else {
			tags = new TimedMessage();
		}

		explorer = new RecordVehicle(trajectory, odometry, measurements, tags, template.Landmarks,
		                             template.VisionFocal, template.FilmArea, template.RangeClip);

		if (extrainfo) {
			TimedTrajectory fullestimate = TrajectoryHistoryFromDescriptor(File.ReadAllText(estimatefile), 7, false);

			estimate      = fullestimate[fullestimate.Count - 1].Item2;
			xodometry     = odometry;
			xmeasurements = measurements;
		}
		else {
			estimate      = null;
			xodometry     = null;
			xmeasurements = null;
		}

		Directory.Delete(tmpdir, true);

		return explorer;
	}
	
	/// <summary>
	/// Create a vehicle from a real sensor device.
	/// </summary>
	/// <param name="sensor">Device (or recorded file) path.</param>
	/// <param name="sidebar">True to show a sidebar with image processing details.</param>
	/// <returns>Vehicle linked to sensor.</returns>
	public static KinectVehicle VehicleFromSensor(string sensor, bool sidebar)
	{
		return new KinectVehicle(sensor, sidebar);
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
