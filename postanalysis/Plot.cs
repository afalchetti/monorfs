// Plot.cs
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
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;

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
using SparseMatrix      = monorfs.SparseMatrix<double>;
using SparseItem        = monorfs.SparseItem<double>;

namespace postanalysis
{

/// <summary>
/// Plot handler.
/// </summary>
public class Plot<MeasurerT, PoseT, MeasurementT>
	where PoseT        : IPose<PoseT>, new()
	where MeasurementT : IMeasurement<MeasurementT>, new()
	where MeasurerT    : IMeasurer<MeasurerT, PoseT, MeasurementT>, new()
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
	public int    RefTime { get; set; }

	/// <summary>
	/// Construct a visualization from its components.
	/// </summary>
	/// <param name="trajectory">Recorded vehicle trajectory.</param>
	/// <param name="estimate">Recorded estimated vehicle trajectory.</param>
	/// <param name="map">Recorded maximum-a-posteriori estimate for the map.</param>
	public Plot(TimedState trajectory, TimedTrajectory estimate, TimedMapModel map,
	            TimedMapModel visiblelandmarks, Map landmarks, double c, double p, double reftime,
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
		RefTime          = Trajectory.BinarySearch(Tuple.Create(reftime, new double[0]), new ComparisonComparer<Tuple<double, double[]>>(
		                                          (Tuple<double, double[]> a, Tuple<double, double[]> b) => Math.Sign(a.Item1 - b.Item1)));
		RefTime          = (RefTime >= 0) ? RefTime : (RefTime != ~Trajectory.Count) ? ~RefTime : 0;
	}

	/// <summary>
	/// Create a visualization object from a pair of formatted description files.
	/// </summary>
	/// <remarks>All file must be previously sorted by time value. This property is assumed.</remarks>
	/// <param name="datafile">Compressed prerecorded data file.</param>
	/// <returns>Prepared visualization object.</returns>
	public static Plot<MeasurerT, PoseT, MeasurementT>
	                  FromFiles(string datafile, HistMode histmode,
	                            double c, double p, double reftime)
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
		Map             landmarks = new Map(3);
		TimedMessage    tags;

		PoseT        dummyP = new PoseT();

		trajectory   = FP.TimedArrayFromDescriptor       (File.ReadAllLines(vehiclefile), dummyP.StateSize);
		estimate     = FP.TrajectoryHistoryFromDescriptor(File.ReadAllText(estimatefile), dummyP.StateSize);
		map          = FP.MapHistoryFromDescriptor       (File.ReadAllText(mapfile), 3);

		if (!string.IsNullOrEmpty(vismapfile)) {
			vislandmarks = FP.MapHistoryFromDescriptor(File.ReadAllText(vismapfile), 3);

			for (int i = vislandmarks.Count; i < map.Count; i++) {
				vislandmarks.Add(Tuple.Create(map[i].Item1, new Map(3)));
			}
		}
		else {
			vislandmarks = new TimedMapModel();

			foreach (var entry in map) {
				vislandmarks.Add(Tuple.Create(entry.Item1, new Map(3)));
			}
		}

		double[][] did = { new double[3] {1e-3, 0, 0},
		                   new double[3] {0, 1e-3, 0},
		                   new double[3] {0, 0, 1e-3} };

		if (!string.IsNullOrEmpty(scenefile)) {
			var explorer = SimulatedVehicle<MeasurerT, PoseT, MeasurementT>.
			                   FromFile(File.ReadAllText(scenefile));

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

		return new Plot<MeasurerT, PoseT, MeasurementT>(trajectory, estimate, map, vislandmarks,
		                                                landmarks, c, p, reftime, histmode, tags);
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

	public Map VisitedMap
	{
		get
		{
			Map cumulative = new Map(3);

			for (int i = 0; i < Map.Count; i++) {
				foreach (Gaussian landmark in VisibleLandmarks[i].Item2) {
					// forcing landmark.Weight to be bigger than zero disregards visible but
					// non-measured landmarks, which is probably the fairest thing to do
					if (cumulative.Near(landmark.Mean, 1e-5).Count == 0 && landmark.Weight > 0) {
						cumulative.Add(landmark);
					}
				}
			}

			return cumulative;
		}
	}

	public TimedValue CorrectSize
	{
		get
		{
			TimedValue size       = new TimedValue();
			Map        cumulative = new Map(3);

			for (int i = 0; i < Map.Count; i++) {
				foreach (Gaussian landmark in VisibleLandmarks[i].Item2) {
					// forcing landmark.Weight to be bigger than zero disregards visible but
					// non-measured landmarks, which is probably the fairest thing to do
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

			PoseT dummy = new PoseT();

			for (int i = 1; i < Trajectory.Count; i++) {
				arclength += diffloc(dummy.FromState(Trajectory[i].Item2), dummy.FromState(Trajectory[i - 1].Item2));
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
			TimedValue error      = new TimedValue();
			int        startindex = 0;
			var        slamtag    = this.Tags.Find(x => x.Item2.Contains("SLAM") && x.Item2.Contains("on"));
			double     slamtime   = (slamtag != null) ? slamtag.Item1 : 0;
			// note that if the tag is not present, Find will return the default Tuple<double, String>
			// and slamtime will be 0.0, which is correct

			for (int i = 0; i < Estimate.Count; i++) {
				TimedValue localerror = internalerror(Trajectory, Estimate[i].Item2);
				double     mean       = 0;

				for (int k = startindex; k < localerror.Count; k++) {
					mean += localerror[k].Item2;
				}

				mean /= localerror.Count - startindex;

				error.Add(Tuple.Create(Estimate[i].Item1, mean));

				if (Estimate[i].Item1 < slamtime) {
					startindex++;
				}
			}

			return error;
		}

		return null;
	}

	private TimedValue locationError0(TimedState groundtruth, TimedState estimate)
	{
		TimedValue error = new TimedValue();
		PoseT      dummy = new PoseT();

		for (int i = 0; i < estimate.Count; i++) {
			int   reftime = Math.Min(i, RefTime);
			PoseT gpose   = dummy.FromLinear(dummy.FromState(groundtruth[i].Item2).Subtract(
			                                 dummy.FromState(groundtruth[reftime].Item2)));
			PoseT epose   = dummy.FromLinear(dummy.FromState(estimate[i].Item2).Subtract(
			                                 dummy.FromState(estimate[reftime].Item2)));

			error.Add(Tuple.Create(groundtruth[i].Item1, diffloc(gpose, epose)));
		}

		return error;
	}

	private TimedValue rotationError0(TimedState groundtruth, TimedState estimate)
	{
		TimedValue error = new TimedValue();
		PoseT      dummy = new PoseT();

		for (int i = 0; i < estimate.Count; i++) {
			int   reftime = Math.Min(i, RefTime);
			PoseT gpose   = dummy.FromLinear(dummy.FromState(groundtruth[i].Item2).Subtract(
			                                 dummy.FromState(groundtruth[reftime].Item2)));
			PoseT epose   = dummy.FromLinear(dummy.FromState(estimate[i].Item2).Subtract(
			                                 dummy.FromState(estimate[reftime].Item2)));
			
			error.Add(Tuple.Create(groundtruth[i].Item1, diffrot(gpose, epose)));
		}

		return error;
	}

	private TimedValue odolocationError0(TimedState groundtruth, TimedState estimate)
	{
		TimedValue error = new TimedValue();
		PoseT      dummy = new PoseT();

		error.Add(Tuple.Create(groundtruth[0].Item1, 0.0));

		for (int i = 10; i < estimate.Count; i++) {
			PoseT godo = dummy.FromLinear(dummy.FromState(groundtruth[i].Item2).Subtract(
			                              dummy.FromState(groundtruth[i-10].Item2)));
			PoseT eodo = dummy.FromLinear(dummy.FromState(estimate[i].Item2).Subtract(
			                              dummy.FromState(estimate[i-10].Item2)));

			error.Add(Tuple.Create(groundtruth[i].Item1, diffloc(godo, eodo)));
		}

		return error;
	}

	private TimedValue odorotationError0(TimedState groundtruth, TimedState estimate)
	{
		TimedValue error = new TimedValue();
		PoseT      dummy = new PoseT();

		error.Add(Tuple.Create(groundtruth[0].Item1, 0.0));

		for (int i = 10; i < estimate.Count; i++) {
			PoseT godo = dummy.FromLinear(dummy.FromState(groundtruth[i].Item2).Subtract(
			                              dummy.FromState(groundtruth[i-10].Item2)));
			PoseT eodo = dummy.FromLinear(dummy.FromState(estimate[i].Item2).Subtract(
			                              dummy.FromState(estimate[i-10].Item2)));

			error.Add(Tuple.Create(groundtruth[i].Item1, diffrot(godo, eodo)));
		}

		return error;
	}

	private double diffloc(PoseT a, PoseT b)
	{
		return new PoseT().FromLinear(a.Subtract(b)).Location.Euclidean();
	}

	private double diffrot(PoseT a, PoseT b)
	{
		double rawangle  = new PoseT().FromLinear(a.Subtract(b)).Orientation.Angle;
		double canonical = Math.Abs(Util.NormalizeAngle(rawangle));

		return canonical;
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
				// reading MapError computes the spatial error
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

			Map   visited = VisitedMap;
			PoseT dummy   = new PoseT();

			double[][] identity = Matrix.JaggedIdentity(3);

			for (int i = 0; i < Map.Count; i++) {
				double[]   dtranslation = new double[3];
				double[]   rotcenter    = new double[3];
				double[][] drotation    = Matrix.JaggedIdentity(3);

				if (Estimate[i].Item2.Count > RefTime) {
					PoseT delta = dummy.FromLinear(dummy.FromState(Estimate[i].Item2[RefTime].Item2).Subtract(
					                  dummy.FromState(Trajectory[RefTime].Item2)));
					
					dtranslation = (-1.0).Multiply(delta.Location);
					drotation    = delta.Orientation.Conjugate().ToMatrix();
					rotcenter    = dummy.FromState(Estimate[i].Item2[RefTime].Item2).Location;
				}

				Map jmap   = Map[i].Item2.BestMapEstimate;
				Map refmap = new Map(3);

				foreach (var component in jmap) {
					double[] transformed = drotation.Multiply(component.Mean.Subtract(rotcenter)).Add(rotcenter).
					                           Add(dtranslation);

					refmap.Add(new Gaussian(transformed, identity, 1.0));
				}

				ospaerror    = OSPA(visited, refmap, out carderror);
				spatialerror = Math.Pow(Math.Pow(ospaerror, P) - Math.Pow(carderror, P), 1.0 / P);

				oerror.Add(Tuple.Create(Map[i].Item1, ospaerror));
				serror.Add(Tuple.Create(Map[i].Item1, spatialerror));
			}

			spatialMapError = serror;

			return oerror;
		}
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

		List<Gaussian> alist = a.ToList();
		List<Gaussian> blist = b.ToList();

		SparseMatrix transport = new SparseMatrix(b.Count, b.Count, 0.0);

		double CP = Math.Pow(C, P);

		for (int i = 0; i < alist.Count; i++) {
		for (int k = 0; k < blist.Count; k++) {
			double distance = Math.Pow(LandmarkDistance(alist[i].Mean, blist[k].Mean), P);
			
			// usually would be:
			//     transport[k,i] = distance;
			// since we will be maximizing instead of minimizing, negative sign;
			// and to take advantage of the sparse nature of the problem,
			// difference against C^P (most of the entries will equal zero)

			if (CP - distance > 1e-5) {
				transport[i, k] = CP - distance;
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

		cardinalityerror = C * Math.Pow((double) (blist.Count - alist.Count) / blist.Count, 1.0 / P);

		return Math.Pow(GraphCombinatorics.AssignmentValue(transport, best) / blist.Count, 1.0 / P);
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
