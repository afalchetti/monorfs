// ISAM2Navigator.cs
// SLAM solving navigator using the iSAM2 iterative algorithm
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
using System.Text;
using Accord.Math;
using Accord.Math.Decompositions;
using AForge;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using NUnit.Framework;
using System.Timers;
using System.Runtime.InteropServices;

using TimedState    = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedMapModel = System.Collections.Generic.List<System.Tuple<double, System.Collections.Generic.List<monorfs.Gaussian>>>;
using AForge.Math.Geometry;

namespace monorfs
{
/// <summary>
/// SLAM solver. It uses the iSAM2 iterative algorithm.
/// </summary>
public class ISAM2Navigator : Navigator
{
	/// <summary>
	/// Maximum distance at which a match is considered valid; otherwise a new landmark is generated.
	/// </summary>
	public static double MatchThreshold { get { return Config.MatchThreshold; } }

	/// <summary>
	/// Number of consecutive frames a landmark must be seen before
	/// it is considered real and not clutter.
	/// </summary>
	public static int NewLandmarkThreshold { get { return Config.NewLandmarkThreshold; } }

	/// <summary>
	/// Algorithm to decide data association.
	/// </summary>
	/// <remarks>
	/// Note that "perfect mode" isn't good for real applications.
	/// It's there to compare performance and gain insights as to how
	/// sensible is the algorithm to associations errors.
	/// </remarks>
	public static DataAssociationAlgorithm DAAlgorithm { get { return Config.DAAlgorithm; } }

	/// <summary>
	/// Obtains the data association from the tracked vehicle.
	/// </summary>
	/// <remarks>
	/// This delegate exposes a minimum amount of information
	/// about the tracked vehicle.
	/// Giving a full reference of the vehicle as a member field
	/// would be calling for generating interdependencies
	/// and could become problematic as an unintended source of
	/// direct information. It is better to separate both
	/// structures as much as possible.
	/// </remarks>
	private Func<List<int>> GetDataAssociation;

	/// <summary>
	/// Whether the tracked vehicle knows its data association.
	/// </summary>
	public bool HasDataAssociation { get; private set; }

	/// <summary>
	/// The handle.
	/// </summary>
	private HandleRef handle;

	/// <summary>
	/// Vehicle pose estimate.
	/// </summary>
	private SimulatedVehicle estimate;

	/// <summary>
	/// Map model estimation.
	/// </summary>
	private List<Gaussian> mapmodel;

	/// <summary>
	/// Previous SLAM step vehicle pose estimate.
	/// </summary>
	private SimulatedVehicle prevestimate;

	/// <summary>
	/// Most accurate estimate of the current vehicle pose.
	/// </summary>
	public override SimulatedVehicle BestEstimate
	{
		get { return estimate;  }
		set { estimate = value;	}
	}

	/// <summary>
	/// Most accurate estimate model of the map.
	/// </summary>
	public override List<Gaussian> BestMapModel
	{
		get { return mapmodel;  }
		set { mapmodel = value; }
	}

	/// <summary>
	/// Not-associated measurements map model.
	/// If enough similar measurements are found
	/// continuosly in time, the landmark is
	/// moved into the map model.
	/// </summary>
	/// <remarks>In this list, the weights are used to indicate
	/// in how many consecutive frames has the candidate been seen.</remarks>
	public List<Gaussian> CandidateMapModel { get; private set; }

	/// <summary>
	/// Previous SLAM step vehicle pose estimate.
	/// </summary>
	public SimulatedVehicle PreviousEstimate
	{
		get { return prevestimate;  }
		set { prevestimate = value; }
	}

	/// <summary>
	/// Next unused label id.
	/// </summary>
	private int nextlabel;

	/// <summary>
	/// Generate a new unused label id.
	/// </summary>
	private int NextLabel { get { return nextlabel++; } }

	/// <summary>
	/// Construct a PHDNavigator using the indicated vehicle as a reference.
	/// </summary>
	/// <param name="vehicle">Vehicle to track.</param>
	/// <param name="onlymapping">If true, don't do SLAM, but mapping
	/// (i.e. the localization is assumed known). Currently not used.</param>
	public ISAM2Navigator(Vehicle vehicle, bool onlymapping = false)
		: base(vehicle, onlymapping)
	{
		int      nmeasurement     = vehicle.MeasurementCovariance.Length;
		int      nmotion          = vehicle.MotionCovariance.Length;
		double[] measurementnoise = new double[nmeasurement];
		double[] motionnoise      = new double[nmotion];
		double   focal            = vehicle.VisionFocal;

		// NOTE the iSAM2 interface assumes independent noise in each component,
		//      discarding every entry that's not in the diagonal;
		//      although it is possible to define a complete matrix,
		//      it doesn't seem to be worth it when comparing performance
		for (int i = 0; i < nmeasurement; ++i) {
			measurementnoise[i] = Math.Sqrt(vehicle.MeasurementCovariance[i][i]);
		}

		for (int i = 0; i < nmotion; ++i) {
			motionnoise[i] = Math.Sqrt(vehicle.MotionCovariance[i][i]);
		}

		estimate          = new SimulatedVehicle();
		prevestimate      = new SimulatedVehicle();
		mapmodel          = new List<Gaussian>();
		CandidateMapModel = new List<Gaussian>();

		vehicle.State.CopyTo(estimate    .State, 0);
		vehicle.State.CopyTo(prevestimate.State, 0);

		HasDataAssociation = vehicle.HasDataAssociation;

		if (HasDataAssociation) {
			GetDataAssociation = () => vehicle.DataAssociation;
		}

		handle    = NewNavigator(vehicle.State, measurementnoise, motionnoise, focal);
		nextlabel = 0;
	}

	/// <summary>
	/// Change the mode of the navigator to solving full slam.
	/// Currently do nothing.
	/// </summary>
	public override void StartSlamInternal() {}

	/// <summary>
	/// Change the mode of the navigator to do only mapping.
	/// Currently do nothing.
	/// </summary>
	public override void StartMappingInternal() {}

	/// <summary>
	/// Reset the map model to an empty map.
	/// Currently do nothing.
	/// </summary>
	public override void ResetMapModel() {}

	/// <summary>
	/// Update the vehicle pose.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="dx">Moved distance from odometry in the local vertical movement-perpendicular direction since last timestep.</param>
	/// <param name="dy">Moved distance from odometry in the local horizontal movement-perpendicular direction since last timestep.</param>
	/// <param name="dz">Moved distance from odometry in the local depth movement-parallel direction since last timestep.</param>
	/// <param name="dyaw">Angle variation from odometry in the yaw coordinate since last timestep.</param>
	/// <param name="dpitch">Angle variation from odometry in the pitch coordinate since last timestep.</param>
	/// <param name="droll">Angle variation from odometry in the roll coordinate since last timestep.</param>
	public override void Update(GameTime time, double dx, double dy, double dz,
	                            double dyaw, double dpitch, double droll)
	{
		BestEstimate.Update(time, dx, dy, dz, dyaw, dpitch, droll);
	}

	/// <summary>
	/// Update both the estimated map and the localization.
	/// This means adding the appropiate graph factor and optimizing.
	/// This method is the core of the whole program.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="measurements">Sensor measurements in pixel-range form.</param>
	public override void SlamUpdate(GameTime time, List<double[]> measurements)
	{
		double[]  odometry = Vehicle.StateDiff(BestEstimate, prevestimate);
		List<int> labels   = FindLabels(measurements);

		for (int i = labels.Count - 1; i >= 0; i--) {
			// candidates are removed from the measurement list
			// until they've been seen enough times
			if (labels[i] < 0) {
				labels.RemoveAt(i);
				measurements.RemoveAt(i);
			}
		}

		double[] marshalmeasurements = new double[3 * measurements.Count];

		for (int i = 0, h = 0; i < measurements.Count; i++) {
			marshalmeasurements[h++] = measurements[i][0];
			marshalmeasurements[h++] = measurements[i][1];
			marshalmeasurements[h++] = measurements[i][2];
		}

		int status = Update(odometry, marshalmeasurements, labels.ToArray(), measurements.Count);

		// updates the estimated complete path to show the batch nature of the algorithm
		List<double[]> trajectory = GetTrajectory();
		BestEstimate.State        = trajectory[trajectory.Count - 1];
		BestMapModel              = GetMapModel();

		// copy the new estimated trajectory (given by iSAM2) into the BestEstimate
		// variable, using the previous timestamps, as they do not change
		for (int i = 0; i < trajectory.Count - 1; i++) {
			BestEstimate.WayPoints[i] = Tuple.Create(BestEstimate.WayPoints[i].Item1, trajectory[i]);
		}

		// the last one does not have a previous (valid) timestamp so it must be created
		BestEstimate.WayPoints[trajectory.Count - 1] =
			Tuple.Create(time.TotalGameTime.TotalSeconds, trajectory[trajectory.Count - 1]);

		// BestEstimate may have been updated several times since the last call to
		// this method, so it could contain a lot of unused data, remove it
		BestEstimate.WayPoints = BestEstimate.WayPoints.GetRange(0, trajectory.Count);

		BestEstimate.State.CopyTo(prevestimate.State, 0);
		UpdateTrajectory(time);
		UpdateMapHistory(time);

		if (status != 0) {
			string message = (status == 1) ? "IndeterminateLinearSystemException" :
			                 (status == 2) ? "ValuesKeyAlreadyExists":
			                                 "gtsam exception";
			throw new InvalidOperationException(message);
		}
	}

	/// <summary>
	/// Find the data association labels from the new valid measurements and
	/// the internal previous map model using Mahalanobis association.
	/// </summary>
	/// <param name="measurements">New measurements.</param>
	/// <returns>Association labels.</returns>
	public List<int> FindLabels(List<double[]> measurements)
	{
		if (DAAlgorithm == DataAssociationAlgorithm.Perfect) {
			if (!HasDataAssociation) {
				throw new InvalidOperationException("Tried to use perfect data association when none exists.");
			}

			return GetDataAssociation();
		}

		double[][] I             = 0.001.Multiply(Accord.Math.Matrix.Identity(3).ToArray());
		bool[]     keepcandidate = new bool[CandidateMapModel.Count];
		double[][] R;
		Gaussian[] q;
		Gaussian[] qcandidate;

		SimulatedVehicle pose             = BestEstimate;
		List<Gaussian>   visible          = new List<Gaussian>();
		List<int>        visibleLandmarks = new List<int>();

		for (int i = 0; i < mapmodel.Count; i++) {
			if (pose.Visible(mapmodel[i].Mean)) {
				visible         .Add(mapmodel[i]);
				visibleLandmarks.Add(i);
			}
		}

		double logPD      = Math.Log(pose.PD);
		double logclutter = Math.Log(pose.ClutterDensity);

		int n = visible.Count + CandidateMapModel.Count;
		int m = visible.Count + CandidateMapModel.Count + measurements.Count;

		// distances(i, k) = distance between landmark i and measurements k
		SparseMatrix distances  = new SparseMatrix(n, n, double.NegativeInfinity);

		int candidatecount = CandidateMapModel.Count;
		// candidate count at the beggining of the process
		// this is so the measurements aren't compared with other measurements
		// (if one gets promoted to candidate, the next one could think it's similar to it
		// but that isn't sensible: one landmark -> hopefully one measurement)

		R          = pose.MeasurementCovariance;
		q          = new Gaussian[visible.Count];
		qcandidate = new Gaussian[CandidateMapModel.Count];

		for (int i = 0; i < q.Length; i++) {
			Gaussian component = visible[i];

			if (DAAlgorithm == DataAssociationAlgorithm.Mahalanobis) {
				double[][] H = pose.MeasurementJacobian(component.Mean);
				q[i] = new Gaussian(pose.MeasurePerfect(component.Mean),
				                    H.Multiply(component.Covariance.MultiplyByTranspose(H)).Add(R),
				                    1.0);
			}
			else {
				q[i] = new Gaussian(component.Mean, I, 1.0);
			}
		}

		for (int i = 0; i < qcandidate.Length; i++) {
			Gaussian component = CandidateMapModel[i];

			if (DAAlgorithm == DataAssociationAlgorithm.Mahalanobis) {
				// assume the covariance is zero, since there's nothing better to assume here
				// note that this is more stringent on the unproven data, as they are given
				// less leeway for noise than the already associated landmark
				qcandidate[i] = new Gaussian(pose.MeasurePerfect(component.Mean), R, component.Weight);
			}
			else {
				qcandidate[i] = new Gaussian(component.Mean, I, 1.0);
			}
		}

		Gaussian[] vlandmarks = q.Concatenate(qcandidate);

		for (int i = 0; i < n;  i++) {
			for (int k = 0; k < measurements.Count; k++) {
				double distance2;

				if (DAAlgorithm == DataAssociationAlgorithm.Mahalanobis) {
					distance2 = vlandmarks[i].MahalanobisSquared(measurements[k]);
				}
				else {
					distance2 = vlandmarks[i].Mean.SquareEuclidean(pose.MeasureToMap(measurements[k]));
				}

				if (distance2 < MatchThreshold * MatchThreshold) {
					distances[i, k] = logPD + Math.Log(vlandmarks[i].Multiplier) - 0.5 * distance2;
				}
			}
		}

		for (int i = 0; i < vlandmarks.Length; i++) {
			distances[i, measurements.Count + i] = logPD;
		}

		for (int i = 0; i < measurements.Count; i++) {
			distances[vlandmarks.Length + i, i] = logclutter;
		}

		// fill the (Misdetection x Clutter) quadrant of the matrix with zeros (don't contribute)
		for (int i = vlandmarks.Length;  i < m; i++) {
		for (int k = measurements.Count; k < m; k++) {
			distances[i, k] = 0;
		}
		}

		int[] assignments = GraphCombinatorics.LinearAssignment(distances);

		// the assignment vector after removing all the clutter variables
		List<int> labels = new List<int>();

		for (int i = 0; i < measurements.Count; i++) {
			labels.Add(int.MinValue);
		}

		// proved landmark
		for (int i = 0; i < visible.Count; i++) {
			if (assignments[i] < measurements.Count) {
				labels[assignments[i]] = visibleLandmarks[i];
			}
		}

		// candidate landmark
		for (int i = visible.Count; i < vlandmarks.Length; i++) {
			if (assignments[i] < measurements.Count) {
				int k = i - visible.Count;

				labels[assignments[i]] = -k - 1;
				// negative labels are for candidates,
				// note that zero is already occupied by the associated landmarks

				// improve the estimated landmark by averaging with the new measurement
				double w = CandidateMapModel[k].Weight;
				CandidateMapModel[k] =
					new Gaussian((CandidateMapModel[k].Mean.Multiply(w).Add(
					                 BestEstimate.MeasureToMap(measurements[assignments[i]]))).Divide(w + 1),
					             I,
					             w + 1);
				
				// note the comparison between double and int
				// since the weight is only used with integer values (and addition by one)
				// there should never be any truncation error, at least while
				// the number has less than 23/52 bits (for float/double);
				// this amounts to 8388608 and 4.5e15 so it should be always ok.
				// In fact, the gtsam key system fails before that
				// (it uses three bytes for numbering, one for character)
				if (CandidateMapModel[k].Weight >= NewLandmarkThreshold) {
					labels[assignments[i]] = NextLabel;
				}
				else {
					keepcandidate[k] = true;
					// only keep candidates that haven't been added, but are still visible
				}
			}
		}

		// else: unmatched measurements, add to candidates
		for (int i = 0; i < measurements.Count; i++) {
			if (labels[i] == int.MinValue) {
				// if far from everything, generate a new candidate at the measured point
				// note that the covariance is assumed zero, though that would throw an exception,
				// as it doesn't have an inverse, so the identity is used instead (dummy value)
				CandidateMapModel.Add(new Gaussian(BestEstimate.MeasureToMap(measurements[i]), I, 1));

				if (NewLandmarkThreshold <= 1) {
					CandidateMapModel.RemoveAt(CandidateMapModel.Count - 1);
					labels[i] = NextLabel;
				}
				
			}
		}

		// anything that wasn't seen goes away, it was clutter
		for (int i = keepcandidate.Length - 1; i >= 0; i--) {
			if (!keepcandidate[i]) {
				CandidateMapModel.RemoveAt(i);
			}
		}

		return labels;
	}

	/// <summary>
	/// Dispose of any resources.
	/// </summary>
	public override void Dispose()
	{
		deletenavigator(handle);
	}

	/// <summary>
	/// Construct a new ISAM2Navigator from unmanaged code.
	/// </summary>
	/// <param name="initPose">Initial pose.</param>
	/// <param name="measurementNoise">Vectorized measurement noise.</param>
	/// <param name="motionNoise">Vectorized motion noise.</param>
	/// <param name="focal">Focal length.</param>
	/// <returns>New navigator.</returns>
	private unsafe HandleRef NewNavigator(double[] initPose, double[] measurementNoise,
	                                      double[] motionNoise, double focal)
	{
		IntPtr ptr = IntPtr.Zero;

		fixed(double* ptrInitPose         = initPose) {
		fixed(double* ptrMeasurementNoise = measurementNoise) {
		fixed(double* ptrMotionNoise      = motionNoise) {
			ptr = newnavigator((IntPtr) ptrInitPose, (IntPtr) ptrMeasurementNoise,
			                   (IntPtr) ptrMotionNoise, focal);
		}
		}
		}

		return new HandleRef(this, ptr);
	}

	/// <summary>
	/// Update the solver estimates from unmanaged code.
	/// </summary>
	/// <param name="odometry">Odometry readings as [dx, dy, dz, dyaw, dpitch, droll].</param>
	/// <param name="measurements">Vectorized measurement list as [x1, y1, z1, x2, y2, z2, ...].</param>
	/// <param name="labels">Data association labels, one per measurement.</param>
	/// <param name="nmeasurements">Number of measurements.</param>
	/// <returns>Error code: 0 = ok, 1 = divergence, 2 = duplicate var, -1 = general error.</returns>
	private unsafe int Update(double[] odometry, double[] measurements, int[] labels, int nmeasurements)
	{
		int retvalue = -1;
		fixed(double* ptrOdometry     = odometry) {
		fixed(double* ptrMeasurements = measurements) {
		fixed(int*    ptrLabels       = labels) {
			retvalue = update(handle, (IntPtr) ptrOdometry, (IntPtr) ptrMeasurements, (IntPtr) ptrLabels, nmeasurements);
		}
		}
		}

		return retvalue;
	}

	/// <summary>
	/// Get the trajectory estimate from unmanaged code.
	/// </summary>
	/// <returns>Trajectory estimate.</returns>
	private unsafe List<double[]> GetTrajectory()
	{
		int            length;
		List<double[]> trajectory = new List<double[]>();

		double* ptrtrajectory = (double*) gettrajectory(handle, out length);

		for (int i = 0, k = 0; i < length; i++, k += 7) {
			double[] point = new double[7];

			point[0] = ptrtrajectory[k + 0];
			point[1] = ptrtrajectory[k + 1];
			point[2] = ptrtrajectory[k + 2];
			point[3] = ptrtrajectory[k + 3];
			point[4] = ptrtrajectory[k + 4];
			point[5] = ptrtrajectory[k + 5];
			point[6] = ptrtrajectory[k + 6];

			trajectory.Add(point);
		}

		return trajectory;
	}

	/// <summary>
	/// Get the map estimate.
	/// </summary>
	/// <returns>Map estimate.</returns>
	private unsafe List<Gaussian> GetMapModel()
	{
		int            length;
		Gaussian       component;
		List<Gaussian> mapmodel   = new List<Gaussian>();

		double* ptrmapmodel = (double*) getmapmodel(handle, out length);
		double* ptrmapcov   = (double*) mapmarginals(handle, out length);

		for (int i = 0, k = 0, h = 0; i < length; i++, k += 3, h += 9) {
			double[]   mean       = new double[3];
			double[][] covariance = {new double[3], new double[3], new double[3]};

			mean[0]          = ptrmapmodel[k + 0];
			mean[1]          = ptrmapmodel[k + 1];
			mean[2]          = ptrmapmodel[k + 2];

			covariance[0][0] = ptrmapcov[h + 0];
			covariance[0][1] = ptrmapcov[h + 1];
			covariance[0][2] = ptrmapcov[h + 2];
			covariance[1][0] = ptrmapcov[h + 3];
			covariance[1][1] = ptrmapcov[h + 4];
			covariance[1][2] = ptrmapcov[h + 5];
			covariance[2][0] = ptrmapcov[h + 6];
			covariance[2][1] = ptrmapcov[h + 7];
			covariance[2][2] = ptrmapcov[h + 8];

			component = new Gaussian(mean, covariance, 1.0);
			mapmodel.Add(component);
		}

		return mapmodel;
	}

	[DllImport("libisam2.so")]
	private extern static IntPtr newnavigator(IntPtr initstate, IntPtr measurementnoise, IntPtr motionnoise, double focal);

	[DllImport("libisam2.so")]
	private extern static void deletenavigator(HandleRef navigator);

	[DllImport("libisam2.so")]
	private extern static int update(HandleRef navigator, IntPtr odometry, IntPtr measurements, IntPtr labels, int nmeasurements);

	[DllImport("libisam2.so")]
	private extern static IntPtr gettrajectory(HandleRef navigator, out int length);

	[DllImport("libisam2.so")]
	private extern static IntPtr getmapmodel(HandleRef navigator, out int length);

	[DllImport("libisam2.so")]
	private extern static IntPtr trajectorymarginals(HandleRef navigator, out int length);

	[DllImport("libisam2.so")]
	private extern static IntPtr mapmarginals(HandleRef navigator, out int length);
}

/// <summary>
/// Data association algorithm.
/// </summary>
public enum DataAssociationAlgorithm
{
	Perfect,
	NearestNeighbours,
	Mahalanobis
}
}
