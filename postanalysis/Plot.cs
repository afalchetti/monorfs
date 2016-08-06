﻿// Plot.cs
// Plot handler
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

using Accord.Math;

using monorfs;

using FP = monorfs.FileParser;

using TimedValue        = System.Collections.Generic.List<System.Tuple<double, double>>;
using TimedArray        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedState        = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedTrajectory   = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<System.Tuple<double, double[]>>>>;
using TimedMapModel     = System.Collections.Generic.List<System.Tuple<double, monorfs.Map>>;
using TimedMeasurements = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<double[]>>>;
using TimedMessage      = System.Collections.Generic.List<System.Tuple<double, string>>;

namespace postanalysis
{

/// <summary>
/// Plot handler.
/// </summary>
public class Plot
{
	public TimedState      Trajectory { get; set; }
	public TimedTrajectory Estimate { get; set; }
	public TimedMapModel   Map { get; set; }
	public Map             Landmarks { get; set; }
	public TimedMapModel   VisibleLandmarks { get; set; }
	public TimedMessage    Tags { get; set; }
	public HistMode        PoseErrorMode { get; set; }

	public double C { get; set; }
	public double P { get; set; }

	/// <summary>
	/// Construct a visualization from its components.
	/// </summary>
	/// <param name="trajectory">Recorded vehicle trajectory.</param>
	/// <param name="estimate">Recorded estimated vehicle trajectory.</param>
	/// <param name="map">Recorded maximum-a-posteriori estimate for the map.</param>
	public Plot(TimedState trajectory, TimedTrajectory estimate, TimedMapModel map,
	            TimedMapModel visiblelandmarks, Map landmarks, double c, double p,
	            HistMode histmode, TimedMessage tags)
	{
		Trajectory       = trajectory;
		Estimate         = estimate;
		Map              = map;
		PoseErrorMode    = histmode;
		Landmarks        = landmarks;
		VisibleLandmarks = visiblelandmarks;
		Tags             = tags;
		C                = c;
		P                = p;
	}

	/// <summary>
	/// Create a visualization object from a pair of formatted description files.
	/// </summary>
	/// <param name="datafile">Compressed prerecorded data file.</param>
	/// <returns>Prepared visualization object.</returns>
	/// <remarks>All file must be previously sorted by time value. This property is assumed.</remarks>
	public static Plot FromFiles(string datafile, HistMode histmode, double c, double p)
	{
		string tmpdir  = Util.TemporaryDir();
		string datadir = Path.Combine(tmpdir, "data");

		ZipFile.ExtractToDirectory(datafile, datadir);

		string scenefile    = Path.Combine(datadir, "scene.world");
		string vehiclefile  = Path.Combine(datadir, "trajectory.out");
		string estimatefile = Path.Combine(datadir, "estimate.out");
		string mapfile      = Path.Combine(datadir, "maps.out");
		string vismapfile   = Path.Combine(datadir, "vismaps.out");
		string tagfile      = Path.Combine(datadir, "tags.out");


		if (!File.Exists(scenefile)) {
			scenefile = "";
		}

		if (!File.Exists(vismapfile)) {
			vismapfile = "";
		}

		if (!File.Exists(tagfile)) {
			tagfile = "";
		}

		TimedState      trajectory;
		TimedTrajectory estimate;
		TimedMapModel   map;
		TimedMapModel   vislandmarks;
		Map             landmarks = new Map();
		TimedMessage    tags;

		trajectory   = FP.TimedArrayFromDescriptor       (File.ReadAllLines(vehiclefile), 7);
		estimate     = FP.TrajectoryHistoryFromDescriptor(File.ReadAllText(estimatefile), 7);
		map          = FP.MapHistoryFromDescriptor       (File.ReadAllText(mapfile));

		if (!string.IsNullOrEmpty(vismapfile)) {
			vislandmarks = FP.MapHistoryFromDescriptor(File.ReadAllText(vismapfile));

			for (int i = vislandmarks.Count; i < map.Count; i++) {
				vislandmarks.Add(Tuple.Create(map[i].Item1, new Map()));
			}
		}
		else {
			vislandmarks = new TimedMapModel();

			foreach (var entry in map) {
				vislandmarks.Add(Tuple.Create(entry.Item1, new Map()));
			}
		}

		double[][] did = { new double[3] {1e-3, 0, 0},
		                   new double[3] {0, 1e-3, 0},
		                   new double[3] {0, 0, 1e-3} };

		if (!string.IsNullOrEmpty(scenefile)) {
			SimulatedVehicle explorer = FP.VehicleFromSimFile(File.ReadAllText(scenefile));

			foreach (var landmark in explorer.Landmarks) {
				landmarks.Add(new Gaussian(landmark, did, 1.0));
			}
		}

		if (!string.IsNullOrEmpty(tagfile)) {
			tags = FP.TimedMessageFromDescriptor(File.ReadAllLines(tagfile));
		}
		else {
			tags = new TimedMessage();
		}

		return new Plot(trajectory, estimate, map, vislandmarks, landmarks, c, p, histmode, tags);
	}

	/// <summary>
	/// Get a string representation of the tags in the timeline.
	/// </summary>
	public string SerializedTags
	{
		get
		{
			return string.Join("\n", Tags.ConvertAll(
				s => {double time    = s.Item1;
			          string message = s.Item2;
				      return time.ToString("g6") + " " + message; }
			));
		}
	}

	public TimedValue ExpectedSize
	{
		get
		{
			TimedValue size = new TimedValue();

			for (int i = 0; i < Map.Count; i++) {
				size.Add(Tuple.Create(Map[i].Item1, Map[i].Item2.ExpectedSize));
			}

			return size;
		}
	}

	public TimedValue CorrectSize
	{
		get
		{
			TimedValue size       = new TimedValue();
			Map        cumulative = new Map();

			for (int i = 0; i < Map.Count; i++) {
				foreach (Gaussian landmark in VisibleLandmarks[i].Item2) {
					if (cumulative.Near(landmark.Mean, 1e-5).Count == 0 && landmark.Weight > 0) {
						cumulative.Add(landmark);
					}
				}

				size.Add(Tuple.Create(Map[i].Item1, (double) cumulative.ExpectedSize));
			}

			return size;
		}
	}

	public TimedValue PathLength
	{
		get
		{
			TimedValue lengths   = new TimedValue();
			double     arclength = 0.0;

			lengths.Add(Tuple.Create(Trajectory[0].Item1, arclength));

			for (int i = 1; i < Trajectory.Count; i++) {
				arclength += diffloc(Trajectory[i].Item2, Trajectory[i - 1].Item2);
				lengths.Add(Tuple.Create(Trajectory[i].Item1, arclength));
			}

			return lengths;
		}
	}

	public TimedValue LocationError
	{
		get
		{
			return poseError(locationError0);
		}
	}

	public TimedValue RotationError
	{
		get
		{
			return poseError(rotationError0);
		}
	}

	public TimedValue OdoLocationError
	{
		get
		{
			return poseError(odolocationError0);
		}
	}

	public TimedValue OdoRotationError
	{
		get
		{
			return poseError(odorotationError0);
		}
	}

	private TimedValue poseError(Func<TimedState, TimedState, TimedValue> internalerror)
	{
		switch (PoseErrorMode) {
		case HistMode.Smooth:
			return internalerror(Trajectory, Estimate[Estimate.Count - 1].Item2);

		case HistMode.Filter:
			TimedState estimate = new TimedState();

			for (int i = 0; i < Estimate.Count; i++) {
				estimate.Add(Tuple.Create(Estimate[i].Item1, Estimate[i].Item2[i].Item2));
			}

			return internalerror(Trajectory, estimate);

		case HistMode.Timed:
			TimedValue error = new TimedValue();

			for (int i = 0; i < Estimate.Count; i++) {
				TimedValue localerror = internalerror(Trajectory, Estimate[i].Item2);
				double     mean       = 0;

				for (int k = 0; k < localerror.Count; k++) {
					mean += localerror[k].Item2;
				}

				mean /= localerror.Count;

				error.Add(Tuple.Create(Estimate[i].Item1, mean));
			}

			return error;
		}

		return null;
	}

	private TimedValue locationError0(TimedState groundtruth, TimedState estimate)
	{
		TimedValue error = new TimedValue();

		for (int i = 0; i < estimate.Count; i++) {
			error.Add(Tuple.Create(groundtruth[i].Item1, diffloc(groundtruth[i].Item2, estimate[i].Item2)));
		}

		return error;
	}

	private TimedValue rotationError0(TimedState groundtruth, TimedState estimate)
	{
		TimedValue error = new TimedValue();

		for (int i = 0; i < estimate.Count; i++) {
			error.Add(Tuple.Create(groundtruth[i].Item1, diffrot(groundtruth[i].Item2, estimate[i].Item2)));
		}

		return error;
	}

	private TimedValue odolocationError0(TimedState groundtruth, TimedState estimate)
	{
		TimedValue error = new TimedValue();

		error.Add(Tuple.Create(groundtruth[0].Item1, 0.0));

		for (int i = 1; i < estimate.Count; i++) {
			Pose3D godo = new Pose3D().FromLinear(groundtruth[i].Item2.Subtract(groundtruth[i-1].Item2));
			Pose3D eodo = new Pose3D().FromLinear(estimate[i].Item2.Subtract(estimate[i-1].Item2));

			error.Add(Tuple.Create(groundtruth[i].Item1, diffloc(godo.State, eodo.State)));
		}

		return error;
	}

	private TimedValue odorotationError0(TimedState groundtruth, TimedState estimate)
	{
		TimedValue error = new TimedValue();

		error.Add(Tuple.Create(groundtruth[0].Item1, 0.0));

		for (int i = 1; i < estimate.Count; i++) {
			Pose3D godo = new Pose3D().FromLinear(groundtruth[i].Item2.Subtract(groundtruth[i-1].Item2));
			Pose3D eodo = new Pose3D().FromLinear(estimate[i].Item2.Subtract(estimate[i-1].Item2));

			error.Add(Tuple.Create(groundtruth[i].Item1, diffrot(godo.State, eodo.State)));
		}

		return error;
	}

	private double diffloc(double[] a, double[] b)
	{
		Pose3D A = new Pose3D(a);
		Pose3D B = new Pose3D(b);

		return new Pose3D().FromLinear(A.Subtract(B)).Location.Euclidean();
	}

	private double diffrot(double[] a, double[] b)
	{
		Pose3D A = new Pose3D(a);
		Pose3D B = new Pose3D(b);

		return Math.Abs(new Pose3D().FromLinear(A.Subtract(B)).Angle);
	}

	private double difflandmark(double[] a, double[] b)
	{
		return a.Subtract(b).Euclidean();
	}

	private TimedValue spatialMapError;

	public TimedValue SpatialMapError
	{
		get
		{
			if (spatialMapError == null) {
				TimedValue dummy = MapError;
				// calling MapError computes the spatial error
			}

			return spatialMapError;
		}
	}

	public TimedValue MapError
	{
		get
		{
			TimedValue oerror = new TimedValue();
			TimedValue serror = new TimedValue();

			double ospaerror;
			double carderror;
			double spatialerror;

			for (int i = 0; i < Map.Count; i++) {
				Map jmap = BestMapEstimate(Map[i].Item2);
				ospaerror = OSPA(Landmarks, jmap, out carderror);
				spatialerror = Math.Pow(Math.Pow(ospaerror, P) - Math.Pow(carderror, P), 1.0/P);

				oerror.Add(Tuple.Create(Map[i].Item1, ospaerror));
				serror.Add(Tuple.Create(Map[i].Item1, spatialerror));
			}

			spatialMapError = serror;

			return oerror;
		}
	}

	public Map BestMapEstimate(Map map)
	{
		Map best = new Map();

		double[][] did = { new double[3] {1e-3, 0, 0},
		                   new double[3] {0, 1e-3, 0},
		                   new double[3] {0, 0, 1e-3} };

		foreach (Gaussian component in map) {
			if (component.Weight > 0.8) {
				best.Add(new Gaussian(component.Mean, did, 1.0));
			}
		}

		return best;
	}

	public double OSPA(Map a, Map b, out double cardinalityerror)
	{
		if (a.Count > b.Count) {
			Map temp = a;
			a        = b;
			b        = temp;
		}

		if (a.Count == 0) {
			cardinalityerror = (b.Count == 0) ? 0 : C;
			return cardinalityerror;
		}

		var alist = a.ToList();
		var blist = b.ToList();

		SparseMatrix transport = new SparseMatrix(b.Count, b.Count);

		double CP = Math.Pow(C, P);

		for (int k = 0; k < blist.Count; k++) {
		for (int i = 0; i < alist.Count; i++) {
			double distance = Math.Pow(LandmarkDistance(alist[i].Mean, blist[k].Mean), P);
			
			// usually would be:
			//     transport[k,i] = distance;
			// since we will be maximizing instead of minimizing, negative sign;
			// and to take advantage of the sparse nature of the problem,
			// difference against C^P (most of the entries will equal zero)

			if (CP - distance > 1e-5) {
				transport[k,i] = CP - distance;
			}
		}
		}

		// in the CP - x format, all these entries are zero :)
		//for (int k = 0; k < blist.Count; k++) {
		//for (int i = alist.Count; i < blist.Count; i++) {
		//	transport[k,i] = CP - CP;
		//}
		//}

		int[] best = GraphCombinatorics.LinearAssignment(transport);
		transport.Apply(x => CP - x);

		cardinalityerror = C * Math.Pow((double) (blist.Count - alist.Count) / blist.Count, 1.0/P);

		return Math.Pow(GraphCombinatorics.AssignmentValue(transport, best) / blist.Count, 1.0/P);
	}

	public double LandmarkDistance(double[] a, double[] b)
	{
		return Math.Min(C, difflandmark(a, b));
	}

	public string SerializedLocationError
	{
		get
		{
			return serializedTimedValue(LocationError);
		}
	}

	public string SerializedRotationError
	{
		get
		{
			return serializedTimedValue(RotationError);
		}
	}

	public string SerializedOdoLocationError
	{
		get
		{
			return serializedTimedValue(OdoLocationError);
		}
	}

	public string SerializedOdoRotationError
	{
		get
		{
			return serializedTimedValue(OdoRotationError);
		}
	}

	public string SerializedMapError
	{
		get
		{
			return serializedTimedValue(MapError);
		}
	}

	public string SerializedSpatialMapError
	{
		get
		{
			return serializedTimedValue(SpatialMapError);
		}
	}

	public string SerializedExpectedSize
	{
		get
		{
			return serializedTimedValue(ExpectedSize);
		}
	}

	public string SerializedCorrectSize
	{
		get
		{
			return serializedTimedValue(CorrectSize);
		}
	}

	public string SerializedPathLength
	{
		get
		{
			return serializedTimedValue(PathLength);
		}
	}

	private string serializedTimedValue(TimedValue series)
	{
		return string.Join("\n", series.ConvertAll(el => el.Item1 + " " + el.Item2));
	}
}

public enum HistMode
{
	Filter, Smooth, Timed
}
}
