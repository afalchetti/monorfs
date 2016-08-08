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

using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework;

using TimedState    = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedArray    = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedMapModel = System.Collections.Generic.List<System.Tuple<double, monorfs.Map>>;

namespace monorfs
{
/// <summary>
/// Vehicle model.
/// </summary>
public abstract class Vehicle<MeasurerT, PoseT, MeasurementT> : IDisposable
	where PoseT        : IPose<PoseT>, new()
	where MeasurementT : IMeasurement<MeasurementT>, new()
	where MeasurerT    : IMeasurer<MeasurerT, PoseT, MeasurementT>, new()
{
	/// <summary>
	/// Saved odometry vector length.
	/// </summary>
	protected static int OdoSize;

	/// <summary>
	/// Saved state vector length.
	/// </summary>
	protected static int StateSize;

	/// <summary>
	/// Saved measurement vector length.
	/// </summary>
	protected static int MeasureSize;

	/// <summary>
	/// Internal motion model covariance matrix. Lie algebra representation.
	/// </summary>
	protected double[][] motionCovariance = Config.MotionCovariance;

	/// <summary>
	/// Motion model covariance matrix. Lie algebra representation.
	/// </summary>
	public double[][] MotionCovariance { get { return motionCovariance; } }

	/// <summary>
	/// Measurement model covariance matrix.
	/// </summary>
	public double[][] MeasurementCovariance = Config.MeasurementCovariance;

	/// <summary>
	/// Internal vehicle pose state.
	/// </summary>
	public PoseT Pose;

	/// <summary>
	/// State using the (noisy) odometry data.
	/// </summary>
	protected PoseT OdometryPose;

	/// <summary>
	/// Odometry reference state; state at which the odometry started accumulating.
	/// Every time the system reads the odometry it resets to the correct state.
	/// </summary>
	private PoseT refodometry;

	/// <summary>
	/// Methods and configuration for the measurement step.
	/// </summary>
	public MeasurerT Measurer;
	
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
	/// Landmark locations against which the measurements are performed, if known.
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
	/// Calculate global constants.
	/// </summary>
	static Vehicle()
	{
		OdoSize     = new PoseT().OdometrySize;
		StateSize   = new PoseT().StateSize;
		MeasureSize = new MeasurementT().Size;
	}

	/// <summary>
	/// Construct a vehicle with default constants.
	/// </summary>
	protected Vehicle() : this(new PoseT().IdentityP()) {}

	/// <summary>
	/// Construct a new Vehicle object from its initial state.
	/// </summary>
	/// <param name="initial">Initial pose.</param>
	protected Vehicle(PoseT initial)
		: this(initial, new MeasurerT()) {}

	/// <summary>
	/// Construct a new Vehicle object from its initial state and appropiate constants.
	/// </summary>
	/// <param name="initial">Initial pose.</param>
	/// <param name="measurer">Measuring config and methods.</param>
	protected Vehicle(PoseT initial, MeasurerT measurer)
	{
		Pose     = initial.DClone();
		Measurer = measurer;
		
		OdometryPose = Pose.DClone();
		refodometry  = Pose.DClone();

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
		WayVisibleMaps.Add(Tuple.Create(0.0, new Map(3)));

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
	protected Vehicle(Vehicle<MeasurerT, PoseT, MeasurementT> that, bool copytrajectory = false)
		: this(that.Pose)
	{
		this.Pose               = that.Pose.DClone();
		this.Graphics           = that.Graphics;
		this.MappedMeasurements = that.MappedMeasurements;
		this.Landmarks          = that.Landmarks;
		this.Measurer           = that.Measurer;

		this.motionCovariance      = that.motionCovariance.MemberwiseClone();
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
			this.WayVisibleMaps.Add(Tuple.Create(0.0, new Map(3)));
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
	public virtual TrackVehicle<MeasurerT, PoseT, MeasurementT>
	                   TrackClone(TrackVehicle<MeasurerT, PoseT, MeasurementT> vehicle,
	                              bool         copytrajectory = false)
	{
		return new TrackVehicle<MeasurerT, PoseT, MeasurementT>(vehicle, copytrajectory);
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
	public virtual TrackVehicle<MeasurerT, PoseT, MeasurementT>
	                   TrackClone(double  motioncovmultiplier,
	                              double  measurecovmultiplier,
	                              double  pdetection,
	                              double  clutter,
	                              bool    copytrajectory = false)
	{
		return new TrackVehicle<MeasurerT, PoseT, MeasurementT>(this,
		               motioncovmultiplier, measurecovmultiplier,
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
		Pose         = Pose        .AddOdometry(reading);
		OdometryPose = OdometryPose.AddOdometry(reading);

		double[] noise = time.ElapsedGameTime.TotalSeconds.Multiply(
		                     Util.RandomGaussianVector(new double[OdoSize],
		                                               MotionCovariance));
		OdometryPose = OdometryPose.AddOdometry(noise);

		WayPoints.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, Util.SClone(Pose.State)));
	}

	/// <summary>
	/// Obtain the cumulative odometry reading since the last call to this function.
	/// </summary>
	/// <returns>State diff.</returns>
	public virtual double[] ReadOdometry(GameTime time)
	{
		double[] reading = OdometryPose.DiffOdometry(refodometry);

		OdometryPose = Pose.DClone();
		refodometry  = Pose.DClone();

		WayOdometry.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, Util.SClone(reading)));

		return reading;
	}

	/// <summary>
	/// Obtain several measurements from the hidden state.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <returns>Pixel-range measurements.</returns>
	public abstract List<MeasurementT> Measure(GameTime time);

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
	/// Render the vehicle on the graphics device.
	/// The graphics device must be ready, otherwise
	/// the method will throw an exception.
	/// </summary>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public virtual void Render(double[][] camera)
	{
		Measurer.RenderFOV(Graphics, Pose, camera);
		RenderTrajectory(camera);
		Measurer.RenderBody(Graphics, Pose, camera);
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
		DrawUtils.DrawTrajectory<PoseT>(Graphics, WayPoints, color, camera);
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

		vertices[0] = new double[3] {measurement[0] - halflen, measurement[1] - halflen, measurement[2]};
		vertices[1] = new double[3] {measurement[0] + halflen, measurement[1] + halflen, measurement[2]};
		
		Graphics.DrawUser2DPolygon(vertices, 0.02f, color, true);

		vertices[0] = new double[3] {measurement[0] - halflen, measurement[1] + halflen, measurement[2]};
		vertices[1] = new double[3] {measurement[0] + halflen, measurement[1] - halflen, measurement[2]};

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

		outvertices[0] = new double[3] {landmark[0] - halflen, landmark[1] - halflen, landmark[2]};
		outvertices[1] = new double[3] {landmark[0] - halflen, landmark[1] + halflen, landmark[2]};
		outvertices[2] = new double[3] {landmark[0] + halflen, landmark[1] + halflen, landmark[2]};
		outvertices[3] = new double[3] {landmark[0] + halflen, landmark[1] - halflen, landmark[2]};

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
	/// Render any Heads Up Display information, on top of any other graphics in the sidebar.
	/// </summary>
	public virtual void RenderSideHUD() {}

	/// <summary>
	/// Dispose of any resources.
	/// </summary>
	public virtual void Dispose() {}

	/// <summary>
	/// Create a vehicle descriptor string.
	/// </summary>
	/// <returns>Simulated vehicle descriptor string.</returns>
	public override string ToString()
	{
		return ToString("g6");
	}

	/// <summary>
	/// Create a vehicle descriptor string.
	/// </summary>
	/// <param name="format">String formt for double values.</param>
	/// <returns>Simulated vehicle descriptor string.</returns>
	public string ToString(string format)
	{
		string descriptor = "";

		descriptor += "pose\n\t"   + Pose.State.ToString(format) + "\n";
		descriptor += "params\n\t" + Measurer.ToString(format)   + "\n";
		descriptor += "landmarks\n\t" + string.Join("\n\t",
		               Landmarks.ConvertAll(l => string.Join(" ",
		                                        l.Convert(x => x.ToString(format))))) + "\n";

		return descriptor;
	}
}
}
