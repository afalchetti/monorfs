// Vehicle.cs
// Abstract vehicle motion and measurement model
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
using AForge;

using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework;

using TimedState    = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedArray    = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedMapModel = System.Collections.Generic.List<System.Tuple<double, monorfs.Map>>;
using U             = monorfs.Util;

namespace monorfs
{
/// <summary>
/// Vehicle model.
/// </summary>
public abstract class Vehicle : IDisposable
{
	/// <summary>
	/// Internal motion model covariance matrix. Yaw-pitch-roll representation.
	/// </summary>
	protected double[][] motionCovariance = Config.MotionCovariance;

	/// <summary>
	/// Internal motion model covariance matrix. Quaternion representation.
	/// </summary>
	protected double[][] motionCovarianceQ = Config.MotionCovarianceQ;

	/// <summary>
	/// Internal motion model covariance matrix. Lie algebra representation.
	/// </summary>
	protected double[][] motionCovarianceL = Config.MotionCovarianceL;

	/// <summary>
	/// Motion model covariance matrix. Yaw-pitch-roll representation.
	/// </summary>
	public double[][] MotionCovariance
	{
		get {
			return motionCovariance;
		}

		set {
			motionCovariance = value;
			Util.GetMotionCovariances(motionCovariance, out motionCovarianceQ, out motionCovarianceL);
		}
	}

	/// <summary>
	/// Motion model covariance matrix. Quaternion representation.
	/// </summary>
	public double[][] MotionCovarianceQ { get { return motionCovarianceQ; } }

	/// <summary>
	/// Motion model covariance matrix. Lie algebra representation.
	/// </summary>
	public double[][] MotionCovarianceL { get { return motionCovarianceL; } }

	/// <summary>
	/// Measurement model covariance matrix.
	/// </summary>
	public double[][] MeasurementCovariance = Config.MeasurementCovariance;

	/// <summary>
	/// Internal vehicle pose state.
	/// </summary>
	public Pose3D Pose;

	/// <summary>
	/// State using the (noisy) odometry data.
	/// </summary>
	protected Pose3D OdometryPose;

	/// <summary>
	/// Odometry reference state; state at which the odometry started accumulating.
	/// Every time the system reads the odometry it resets to the correct state.
	/// </summary>
	private Pose3D refodometry;

	/// <summary>
	/// Vision camera focal length.
	/// </summary>
	public double VisionFocal { get; private set; }

	/// <summary>
	/// 2D film clipping area (i.e. sensor size and offset) in pixel units.
	/// </summary>
	public Rectangle FilmArea { get; protected set; }

	/// <summary>
	/// Range clipping planes in meters.
	/// </summary>
	public Range RangeClip { get; private set; }
	
	/// <summary>
	/// Render output.
	/// </summary>
	public virtual GraphicsDevice Graphics { get; set; }

	/// <summary>
	/// Render bitmap renderer.
	/// </summary>
	public virtual SpriteBatch Flip { get; set; }

	/// <summary>
	/// Trajectory through which the vehicle has moved.
	/// The first argument per item is a timestamp, the
	/// next three are 3D coordinates.
	/// </summary>
	public TimedState WayPoints { get; set; }

	/// <summary>
	/// Full odometry history.
	/// </summary>
	public TimedArray WayOdometry { get; private set; }

	/// <summary>
	/// History of landmark visibility.
	/// </summary>
	public TimedMapModel WayVisibleMaps { get; private set; }
	
	/// <summary>
	/// Cached measurements from the update process for rendering purposes.
	/// </summary>
	public List<double[]> MappedMeasurements { get; protected set; }

	/// <summary>
	/// Landmark 3d locations against which the measurements are performed, if known.
	/// </summary>
	public List<double[]> Landmarks { get; protected set; }

	/// <summary>
	/// Data association for the last measurement.
	/// </summary>
	public List<int> DataAssociation { get; protected set; }

	/// <summary>
	/// Whether the vehicle knows the data association in advance or not.
	/// </summary>
	public bool HasDataAssociation { get; protected set; }

	/// <summary>
	/// Groundtruth trajectory.
	/// </summary>
	public TimedState Groundtruth { get; protected set; }

	/// <summary>
	/// If true, the Groundtruth property contains the real trajectory;
	/// otherwise, it will be empty;
	/// </summary>
	public bool KnowsGroundtruth { get; protected set; }

	/// <summary>
	/// Whether the vehicle uses the sidebar or not.
	/// </summary>
	public bool HasSidebar { get; protected set; }

	/// <summary>
	/// Whether the vehicle is done with its movement and would like
	/// to be stopped.
	/// </summary>
	public bool WantsToStop { get; protected set; }

	/// <summary>
	/// If true, suggest other actors to use SLAM instead of mapping.
	/// </summary>
	public bool WantsSLAM { get; protected set; }

	/// <summary>
	/// If true, suggest other actors to use mapping instead of SLAM.
	/// </summary>
	public bool WantsMapping { get; protected set; }

	/// <summary>
	/// Construct a vehicle with default constants.
	/// </summary>
	protected Vehicle() : this(Pose3D.Identity) {}

	/// <summary>
	/// Construct a new Vehicle object from its initial state.
	/// </summary>
	/// <param name="initial">Initial pose.</param>
	protected Vehicle(Pose3D initial)
		: this(initial, 575.8156,
	           new Rectangle(-640 / 2, -480 / 2, 640, 480),
			   new Range(0.1f, 2f)) {}

	/// <summary>
	/// Construct a new Vehicle object from its initial state and appropiate constants.
	/// </summary>
	/// <param name="initial">Initial pose.</param>
	/// <param name="focal">Focal lenghth.</param>
	/// <param name="film">Film area.</param>
	/// <param name="clip">Range clipping area.</param>
	protected Vehicle(Pose3D initial, double focal, Rectangle film, Range clip)
	{
		Pose        = new Pose3D(initial);
		VisionFocal = focal;
		FilmArea    = film;
		RangeClip   = clip;
		
		OdometryPose = new Pose3D(Pose);
		refodometry  = new Pose3D(Pose);

		HasDataAssociation = false;
		DataAssociation    = new List<int>();
		Landmarks          = new List<double[]>();
		MappedMeasurements = new List<double[]>();

		Groundtruth      = new TimedState();
		KnowsGroundtruth = false;

		WayOdometry    = new TimedArray();
		WayPoints      = new TimedState();
		WayVisibleMaps = new TimedMapModel();

		WayPoints     .Add(Tuple.Create(0.0, Util.SClone(Pose.State)));
		WayVisibleMaps.Add(Tuple.Create(0.0, new Map()));

		HasSidebar   = false;
		WantsToStop  = false;
		WantsSLAM    = false;
		WantsMapping = false;
	}

	/// <summary>
	/// Perform a deep copy of another general vehicle.
	/// </summary>
	/// <param name="that">Copied general vehicle.</param>
	/// <param name="copytrajectory">If true, the vehicle historic trajectory is copied. Relatively heavy operation.</param>
	protected Vehicle(Vehicle that, bool copytrajectory = false)
		: this(that.Pose)
	{
		this.Pose               = new Pose3D(that.Pose);
		this.Graphics           = that.Graphics;
		this.MappedMeasurements = that.MappedMeasurements;
		this.Landmarks          = that.Landmarks;

		this.VisionFocal  = that.VisionFocal;
		this.FilmArea     = that.FilmArea;
		this.RangeClip    = that.RangeClip;

		this.motionCovariance      = that.motionCovariance.MemberwiseClone();
		this.motionCovarianceQ     = that.motionCovarianceQ.MemberwiseClone();
		this.motionCovarianceL     = that.motionCovarianceL.MemberwiseClone();
		this.MeasurementCovariance = that.MeasurementCovariance.MemberwiseClone();

		if (copytrajectory) {
			this.WayPoints      = new TimedState(that.WayPoints);
			this.WayOdometry    = new TimedArray(that.WayOdometry);
			this.WayVisibleMaps = new TimedMapModel(that.WayVisibleMaps);
		}
		else {
			this.WayOdometry    = new TimedArray();
			this.WayPoints      = new TimedState();
			this.WayVisibleMaps = new TimedMapModel();

			this.WayPoints     .Add(Tuple.Create(0.0, Util.SClone(Pose.State)));
			this.WayVisibleMaps.Add(Tuple.Create(0.0, new Map()));
		}
	}

	/// <summary>
	/// Clone a vehicle with an associated simulation particle.
	/// Polymorphism on the return value is allowed and encouraged
	/// to provide specific traits for particular reference vehicle.
	/// Assume default cloning parameters.
	/// </summary>
	/// <param name="vehicle">Vehicle to clone.</param>
	/// <param name="copytrajectory">If true, copy the whole trajectory history.</param>
	/// <returns>The clone.</returns>
	public virtual TrackVehicle TrackClone(TrackVehicle vehicle,
	                                       bool         copytrajectory = false)
	{
		return new TrackVehicle(vehicle, copytrajectory);
	}

	/// <summary>
	/// Clone a vehicle with an associated simulation particle.
	/// Polymorphism on the return value is allowed and encouraged
	/// to provide specific traits for particular reference vehicle.
	/// Specify all cloning parameters.
	/// </summary>
	/// <param name="motioncovmultiplier">Motion covariance multiplier.</param>
	/// <param name="measurecovmultiplier">Measurement covariance multiplier.</param>
	/// <param name="pdetection">Detection probability.</param>
	/// <param name="clutter">Clutter density.</param>
	/// <param name="copytrajectory">If true, copy the whole trajectory history.</param>
	/// <returns>The clone.</returns>
	public virtual TrackVehicle TrackClone(double  motioncovmultiplier,
	                                       double  measurecovmultiplier,
	                                       double  pdetection,
	                                       double  clutter,
	                                       bool    copytrajectory = false)
	{
		return new TrackVehicle(this, motioncovmultiplier, measurecovmultiplier,
		                        pdetection, clutter, copytrajectory);
	}

	/// <summary>
	/// Apply the motion model to the vehicle. It corresponds to a
	/// 3D odometry model following the equation:
	/// 
	/// x = x + q dx q* + N(0, Q)
	/// o = dq o dq* + N(0, Q')
	/// 
	/// where q is the midrotation quaternion (halfway between the old and new orientations) and N(a, b) is a normal function
	/// with mean 'a' and covariance matrix 'b'.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="reading">Odometry reading (dx, dy, dz, dpitch, dyaw, droll).</param>
	public virtual void Update(GameTime time, double[] reading)
	{
		Pose         = Pose        .Add(reading);
		OdometryPose = OdometryPose.Add(reading);

		double[] noise = time.ElapsedGameTime.TotalSeconds.Multiply(
		                     U.RandomGaussianVector(new double[6] {0, 0, 0, 0, 0, 0},
		                                            MotionCovarianceL));
		OdometryPose = OdometryPose.Add(noise);

		WayPoints.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, Util.SClone(Pose.State)));
	}

	/// <summary>
	/// Obtain the cumulative odometry reading since the last call to this function.
	/// </summary>
	/// <returns>State diff.</returns>
	public virtual double[] ReadOdometry(GameTime time)
	{
		double[] reading = OdometryPose.Subtract(refodometry);

		OdometryPose = new Pose3D(Pose);
		refodometry  = new Pose3D(Pose);

		WayOdometry.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, Util.SClone(reading)));

		return reading;
	}

	/// <summary>
	/// Obtain several measurements from the hidden state.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <returns>Pixel-range measurements.</returns>
	public abstract List<double[]> Measure(GameTime time);

	/// <summary>
	/// Remove all the localization history and start it again from the current position.
	/// </summary>
	public void ResetHistory()
	{
		WayOdometry.Clear();
		WayPoints.Clear();
		WayPoints.Add(Tuple.Create(0.0, Util.SClone(Pose.State)));
	}

	/// <summary>
	/// Transform a measurement vector in measurement space (pixel-range)
	/// into a map-space vector  (x-y plane).
	/// </summary>
	/// <param name="measurement">Measurement expressed as pixel-range.</param>
	/// <returns>Measurement expressed in x-y plane.</returns>
	public double[] MeasureToMap(double[] measurement)
	{
		double   px    = measurement[0];
		double   py    = measurement[1];
		double   range = measurement[2];

		double   alpha = range / Math.Sqrt(VisionFocal * VisionFocal + px * px + py * py);
		double[] diff  = new double[3] {alpha * px, alpha * py, alpha * VisionFocal};

		Quaternion rotated = Pose.Orientation *
		                     new Quaternion(0, diff[0], diff[1], diff[2]) * Pose.Orientation.Conjugate();

		return new double[3] {Pose.X + rotated.X, Pose.Y + rotated.Y, Pose.Z + rotated.Z};
	}

	/// <summary>
	/// Render the vehicle on the graphics device.
	/// The graphics device must be ready, otherwise
	/// the method will throw an exception.
	/// </summary>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public virtual void Render(double[][] camera)
	{
		RenderFOV(camera);
		RenderTrajectory(camera);
		RenderBody(camera);
		RenderMeasurements(camera);
		RenderLandmarks(camera);
	}

	/// <summary>
	/// Render all the measurements.
	/// </summary>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public void RenderMeasurements(double[][] camera)
	{
		foreach (double[] measure in MappedMeasurements) {
			RenderMeasure(measure, camera);
		}
	}

	/// <summary>
	/// Render all the landmarks.
	/// </summary>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public void RenderLandmarks(double[][] camera)
	{
		foreach (double[] landmark in Landmarks) {
			RenderLandmark(landmark, camera);
		}
	}

	/// <summary>
	/// Render the vehicle physical body on the graphics device.
	/// <param name="camera">Camera 4d transform matrix.</param>
	/// </summary>
	public void RenderBody(double[][] camera)
	{
		const float halflen = 0.06f;
		
		Color innercolor =  Color.LightBlue;
		Color outercolor =  Color.Blue;

		double[] pos = camera.TransformH(Pose.Location);
		
		VertexPositionColor[] invertices  = new VertexPositionColor[4];
		double[][]            outvertices = new double[4][];

		outvertices[0] = new double[] {pos[0] - halflen, pos[1] - halflen, pos[2]};
		outvertices[1] = new double[] {pos[0] - halflen, pos[1] + halflen, pos[2]};
		outvertices[2] = new double[] {pos[0] + halflen, pos[1] + halflen, pos[2]};
		outvertices[3] = new double[] {pos[0] + halflen, pos[1] - halflen, pos[2]};

		invertices[0] = new VertexPositionColor(outvertices[0].ToVector3(), innercolor);
		invertices[1] = new VertexPositionColor(outvertices[1].ToVector3(), innercolor);
		invertices[2] = new VertexPositionColor(outvertices[3].ToVector3(), innercolor);
		invertices[3] = new VertexPositionColor(outvertices[2].ToVector3(), innercolor);

		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, invertices, 0, invertices.Length - 2);
		Graphics.DrawUser2DPolygon(outvertices, 0.02f, outercolor, true);
		
		Quaternion x = Pose.Orientation * new Quaternion(0, 0.2f, 0, 0) * Pose.Orientation.Conjugate();
		pos = camera.TransformH(new double[3] {Pose.X + x.X, Pose.Y + x.Y, Pose.Z + x.Z});

		outvertices[0] = new double[] {pos[0] - 0.4*halflen, pos[1] - 0.4*halflen, pos[2]};
		outvertices[1] = new double[] {pos[0] - 0.4*halflen, pos[1] + 0.4*halflen, pos[2]};
		outvertices[2] = new double[] {pos[0] + 0.4*halflen, pos[1] + 0.4*halflen, pos[2]};
		outvertices[3] = new double[] {pos[0] + 0.4*halflen, pos[1] - 0.4*halflen, pos[2]};

		invertices[0] = new VertexPositionColor(outvertices[0].ToVector3(), innercolor);
		invertices[1] = new VertexPositionColor(outvertices[1].ToVector3(), innercolor);
		invertices[2] = new VertexPositionColor(outvertices[3].ToVector3(), innercolor);
		invertices[3] = new VertexPositionColor(outvertices[2].ToVector3(), innercolor);

		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, invertices, 0, invertices.Length - 2);
		Graphics.DrawUser2DPolygon(outvertices, 0.02f, Color.Blue, true);
		
		x   = Pose.Orientation * new Quaternion(0, 0, 0.2f, 0) * Pose.Orientation.Conjugate();
		pos = camera.TransformH(new double[3] {Pose.X + x.X, Pose.Y + x.Y, Pose.Z + x.Z});

		outvertices[0] = new double[] {pos[0] - 0.2*halflen, pos[1] - 0.2*halflen, pos[2]};
		outvertices[1] = new double[] {pos[0] - 0.2*halflen, pos[1] + 0.2*halflen, pos[2]};
		outvertices[2] = new double[] {pos[0] + 0.2*halflen, pos[1] + 0.2*halflen, pos[2]};
		outvertices[3] = new double[] {pos[0] + 0.2*halflen, pos[1] - 0.2*halflen, pos[2]};

		invertices[0] = new VertexPositionColor(outvertices[0].ToVector3(), innercolor);
		invertices[1] = new VertexPositionColor(outvertices[1].ToVector3(), innercolor);
		invertices[2] = new VertexPositionColor(outvertices[3].ToVector3(), innercolor);
		invertices[3] = new VertexPositionColor(outvertices[2].ToVector3(), innercolor);

		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, invertices, 0, invertices.Length - 2);
		Graphics.DrawUser2DPolygon(outvertices, 0.02f, Color.Yellow, true);
		
		x   = Pose.Orientation * new Quaternion(0, 0, 0, 0.2f) * Pose.Orientation.Conjugate();
		pos = camera.TransformH(new double[3] {Pose.X + x.X, Pose.Y + x.Y, Pose.Z + x.Z});

		outvertices[0] = new double[] {pos[0] - 0.8*halflen, pos[1] - 0.8*halflen, pos[2]};
		outvertices[1] = new double[] {pos[0] - 0.8*halflen, pos[1] + 0.8*halflen, pos[2]};
		outvertices[2] = new double[] {pos[0] + 0.8*halflen, pos[1] + 0.8*halflen, pos[2]};
		outvertices[3] = new double[] {pos[0] + 0.8*halflen, pos[1] - 0.8*halflen, pos[2]};

		invertices[0] = new VertexPositionColor(outvertices[0].ToVector3(), innercolor);
		invertices[1] = new VertexPositionColor(outvertices[1].ToVector3(), innercolor);
		invertices[2] = new VertexPositionColor(outvertices[3].ToVector3(), innercolor);
		invertices[3] = new VertexPositionColor(outvertices[2].ToVector3(), innercolor);

		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, invertices, 0, invertices.Length - 2);
		Graphics.DrawUser2DPolygon(outvertices, 0.02f, Color.Red, true);
	}

	/// <summary>
	/// Render the Field-of-View cone on the graphics device.
	/// </summary>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public void RenderFOV(double[][] camera)
	{
		Color incolorA = Color.LightGreen; incolorA.A = 30;
		Color incolorB = Color.LightGreen; incolorB.A = 15;
		Color outcolor = Color.DarkGreen;  outcolor.A = 30;

		double[][] frustum = new double[8][];
		
		frustum[0] = new double[3] {RangeClip.Min * FilmArea.Left  / VisionFocal, RangeClip.Min * FilmArea.Top    / VisionFocal, RangeClip.Min};
		frustum[1] = new double[3] {RangeClip.Min * FilmArea.Right / VisionFocal, RangeClip.Min * FilmArea.Top    / VisionFocal, RangeClip.Min};
		frustum[2] = new double[3] {RangeClip.Min * FilmArea.Right / VisionFocal, RangeClip.Min * FilmArea.Bottom / VisionFocal, RangeClip.Min};
		frustum[3] = new double[3] {RangeClip.Min * FilmArea.Left  / VisionFocal, RangeClip.Min * FilmArea.Bottom / VisionFocal, RangeClip.Min};
		frustum[4] = new double[3] {RangeClip.Max * FilmArea.Left  / VisionFocal, RangeClip.Max * FilmArea.Top    / VisionFocal, RangeClip.Max};
		frustum[5] = new double[3] {RangeClip.Max * FilmArea.Right / VisionFocal, RangeClip.Max * FilmArea.Top    / VisionFocal, RangeClip.Max};
		frustum[6] = new double[3] {RangeClip.Max * FilmArea.Right / VisionFocal, RangeClip.Max * FilmArea.Bottom / VisionFocal, RangeClip.Max};
		frustum[7] = new double[3] {RangeClip.Max * FilmArea.Left  / VisionFocal, RangeClip.Max * FilmArea.Bottom / VisionFocal, RangeClip.Max};

		for (int i = 0; i < frustum.Length; i++) {
			Quaternion local = Pose.Orientation *
			                       new Quaternion(0, frustum[i][0], frustum[i][1], frustum[i][2]) * Pose.Orientation.Conjugate();
			frustum[i] = camera.TransformH(new double[3] {local.X, local.Y, local.Z}.Add(Pose.Location));
		}

		double[][] wireA = new double[4][];
		double[][] wireB = new double[4][];
		double[][] wireC = new double[4][];
		double[][] wireD = new double[4][];

		wireA[0] = frustum[0];
		wireA[1] = frustum[1];
		wireA[2] = frustum[5];
		wireA[3] = frustum[6];

		wireB[0] = frustum[1];
		wireB[1] = frustum[2];
		wireB[2] = frustum[6];
		wireB[3] = frustum[7];
		
		wireC[0] = frustum[2];
		wireC[1] = frustum[3];
		wireC[2] = frustum[7];
		wireC[3] = frustum[4];

		wireD[0] = frustum[3];
		wireD[1] = frustum[0];
		wireD[2] = frustum[4];
		wireD[3] = frustum[5];
		
		Graphics.DrawUser2DPolygon(wireA, 0.02f, outcolor, false);
		Graphics.DrawUser2DPolygon(wireB, 0.02f, outcolor, false);
		Graphics.DrawUser2DPolygon(wireC, 0.02f, outcolor, false);
		Graphics.DrawUser2DPolygon(wireD, 0.02f, outcolor, false);

		// colored sides shouldn't get in the way of the visualization,
		// so put it behind everything else
		for (int i = 0; i < frustum.Length; i++) {
			frustum[i][2] = -100;
		}

		VertexPositionColor[] verticesA = new VertexPositionColor[8];
		VertexPositionColor[] verticesB = new VertexPositionColor[8];

		verticesA[0] = new VertexPositionColor(frustum[0].ToVector3(), incolorA);
		verticesA[1] = new VertexPositionColor(frustum[4].ToVector3(), incolorA);
		verticesA[2] = new VertexPositionColor(frustum[1].ToVector3(), incolorA);
		verticesA[3] = new VertexPositionColor(frustum[5].ToVector3(), incolorA);
		verticesA[4] = new VertexPositionColor(frustum[2].ToVector3(), incolorA);
		verticesA[5] = new VertexPositionColor(frustum[6].ToVector3(), incolorA);
		verticesA[6] = new VertexPositionColor(frustum[3].ToVector3(), incolorA);
		verticesA[7] = new VertexPositionColor(frustum[7].ToVector3(), incolorA);

		verticesB[0] = new VertexPositionColor(frustum[1].ToVector3(), incolorB);
		verticesB[1] = new VertexPositionColor(frustum[5].ToVector3(), incolorB);
		verticesB[2] = new VertexPositionColor(frustum[2].ToVector3(), incolorB);
		verticesB[3] = new VertexPositionColor(frustum[6].ToVector3(), incolorB);
		verticesB[4] = new VertexPositionColor(frustum[3].ToVector3(), incolorB);
		verticesB[5] = new VertexPositionColor(frustum[7].ToVector3(), incolorB);
		verticesB[6] = new VertexPositionColor(frustum[0].ToVector3(), incolorB);
		verticesB[7] = new VertexPositionColor(frustum[4].ToVector3(), incolorB);
		
		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, verticesA, 0, 2);
		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, verticesA, 4, 2);
		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, verticesB, 0, 2);
		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, verticesB, 4, 2);
	}

	/// <summary>
	/// Render the path that the vehicle has traveled so far.
	/// </summary>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public void RenderTrajectory(double[][] camera)
	{
		RenderTrajectory(camera, Color.Yellow);
	}

	/// <summary>
	/// Render the path that the vehicle has traveled so far.
	/// </summary>
	/// <param name="camera">Camera 4d transform matrix.</param>
	/// <param name="color">Trajectory color.</param>
	public void RenderTrajectory(double[][] camera, Color color)
	{
		DrawUtils.DrawTrajectory(Graphics, WayPoints, color, camera);
	}

	/// <summary>
	/// Simple point measurement rendering.
	/// </summary>
	/// <param name="measurement">Point measurement position.</param>
	/// <param name="camera">Camera 4d transform matrix.</param>
	private void RenderMeasure(double[] measurement, double[][] camera)
	{
		const float halflen = 0.06f;
		
		Color color =  Color.Crimson;

		measurement = camera.TransformH(measurement);
		
		double[][] vertices = new double[2][];

		vertices[0] = new double[] {measurement[0] - halflen, measurement[1] - halflen, measurement[2]};
		vertices[1] = new double[] {measurement[0] + halflen, measurement[1] + halflen, measurement[2]};
		
		Graphics.DrawUser2DPolygon(vertices, 0.02f, color, true);

		vertices[0] = new double[] {measurement[0] - halflen, measurement[1] + halflen, measurement[2]};
		vertices[1] = new double[] {measurement[0] + halflen, measurement[1] - halflen, measurement[2]};

		Graphics.DrawUser2DPolygon(vertices, 0.02f, color, true);
	}

	/// <summary>
	/// Simple point landmark rendering.
	/// </summary>
	/// <param name="landmark">Point landmark position.</param>
	/// <param name="camera">Camera 4d transform matrix.</param>
	private void RenderLandmark(double[] landmark, double[][] camera)
	{
		const float halflen = 0.024f;

		Color innercolor =  Color.LightGray;
		Color outercolor =  Color.Black;

		landmark = camera.TransformH(landmark);

		VertexPositionColor[] invertices  = new VertexPositionColor[4];
		double[][]            outvertices = new double[4][];

		outvertices[0] = new double[] {landmark[0] - halflen, landmark[1] - halflen, landmark[2]};
		outvertices[1] = new double[] {landmark[0] - halflen, landmark[1] + halflen, landmark[2]};
		outvertices[2] = new double[] {landmark[0] + halflen, landmark[1] + halflen, landmark[2]};
		outvertices[3] = new double[] {landmark[0] + halflen, landmark[1] - halflen, landmark[2]};

		invertices[0] = new VertexPositionColor(outvertices[0].ToVector3(), innercolor);
		invertices[1] = new VertexPositionColor(outvertices[1].ToVector3(), innercolor);
		invertices[2] = new VertexPositionColor(outvertices[3].ToVector3(), innercolor);
		invertices[3] = new VertexPositionColor(outvertices[2].ToVector3(), innercolor);

		Graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, invertices, 0, invertices.Length - 2);
		Graphics.DrawUser2DPolygon(outvertices, 0.02f, outercolor, true);
	}

	/// <summary>
	/// Render the sidebar info screen (extra interesting data).
	/// </summary>
	public virtual void RenderSide() {}

	/// <summary>
	/// Dispose of any resources.
	/// </summary>
	public virtual void Dispose() {}
}
}
