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

using TimedState = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using TimedArray = System.Collections.Generic.List<System.Tuple<double, double[]>>;
using U          = monorfs.Util;

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
	private double[] state;

	/// <summary>
	/// Internal state as a vector.
	/// </summary>
	public double[] State {
		get { return state; }
		set { state = new double[value.Length]; Array.Copy(value, state,  value.Length); }
	}

	/// <summary>
	/// Location x-axis coordinate.
	/// </summary>
	public double X
	{
		get { return  State[0]; }
		set { State[0] = value; }
	}

	/// <summary>
	/// Location y-axis coordinate.
	/// </summary>
	public double Y
	{
		get { return  State[1]; }
		set { State[1] = value; }
	}

	/// <summary>
	/// Location z-axis coordinate.
	/// </summary>
	public double Z
	{
		get { return  State[2]; }
		set { State[2] = value; }
	}

	/// <summary>
	/// Space location of the vehicle, i.e. its coordinates.
	/// </summary>
	public double[] Location
	{
		get { return new double[3] {State[0], State[1], State[2]}; }
		set { State[0] = value[0]; State[1] = value[1]; State[2] = value[2]; }
	}

	/// <summary>
	/// Rotation quaternion scalar component.
	/// </summary>
	public double W
	{
		get { return  State[3]; }
		set { State[3] = value; }
	}

	/// <summary>
	/// Rotation quaternion vector component.
	/// </summary>
	public double[] K
	{
		get { return new double[3] {State[4], State[5], State[6]}; }
		set { State[4] = value[0]; State[5] = value[1]; State[6] = value[2]; }
	}
	

	/// <summary>
	/// Orientation of the vehicle.
	/// </summary>
	public Quaternion Orientation
	{
		get { return new Quaternion((float) State[4], (float) State[5], (float) State[6], (float) State[3]); }
		set { State[3] = value.W; State[4] = value.X; State[5] = value.Y; State[6] = value.Z; }
	}

	/// <summary>
	/// Angle in angle-axis representation.
	/// </summary>
	public double Angle
	{
		get { return 2 * Math.Acos(W); }
	}

	/// <summary>
	/// Axis in angle-axis representation.
	/// </summary>
	public double[] Axis
	{
		get
		{
			double normalize = Math.Sqrt(1 - W * W);
			return (normalize > 1e-8) ? K.Divide(normalize) : new double[3] {1, 0, 0};
		}
	}

	/// <summary>
	/// State using the (noisy) odometry data.
	/// </summary>
	private double[] odometrystate;

	/// <summary>
	/// Odometry reference state; state at which the odometry started accumulating.
	/// Every time the system reads the odometry it resets to the correct state.
	/// </summary>
	private double[] refodometrystate;

	/// <summary>
	/// Vision camera focal length.
	/// </summary>
	public double VisionFocal { get; private set; }

	/// <summary>
	/// 2D film clipping area (i.e. sensor size and offset) in pixel units.
	/// </summary>
	public Rectangle FilmArea { get; private set; }

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
	/// Reference sidebar drawing width.
	/// </summary>
	public int SidebarWidth { get; protected set; }

	/// <summary>
	/// Reference sidebar drawing height.
	/// </summary>
	public int SidebarHeight { get; protected set; }

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
	/// Whether the vehicle uses the sidebar or not.
	/// </summary>
	public bool HasSidebar { get; protected set; }

	/// <summary>
	/// Whether the vehicle is done with its movement and would like
	/// to be stopped.
	/// </summary>
	public bool WantsToStop { get; protected set; }

	/// <summary>
	/// Construct a vehicle with default constants.
	/// </summary>
	protected Vehicle() : this(new double[3] {0, 0, 0}, 0, new double[3] {1, 0, 0}) {}

	/// <summary>
	/// Construct a new Vehicle object from its initial state.
	/// </summary>
	/// <param name="location">Spatial coordinates.</param>
	/// <param name="theta">Orientation angle.</param>
	/// <param name="axis">Orientation rotation axis.</param>
	protected Vehicle(double[] location, double theta, double[] axis)
		: this(location, theta, axis, 575.8156,
	           new Rectangle(-640 / 2, -480 / 2, 640, 480),
			   new Range(0.1f, 10f)) {}

	/// <summary>
	/// Construct a new Vehicle object from its initial state and appropiate constants.
	/// </summary>
	/// <param name="location">Spatial coordinates.</param>
	/// <param name="theta">Orientation angle.</param>
	/// <param name="axis">Orientation rotation axis.</param>
	/// <param name="focal">Focal lenghth.</param>
	/// <param name="film">Film area.</param>
	/// <param name="clip">Range clipping area.</param>
	protected Vehicle(double[] location, double theta, double[] axis, double focal, Rectangle film, Range clip)
	{
		double w = Math.Cos(theta / 2);
		double d = Math.Sin(theta / 2);

		axis.Divide(axis.Euclidean(), true);
		
		State       = new double[7] {location[0], location[1], location[2], w, d * axis[0], d * axis[1], d * axis[2]};
		VisionFocal = focal;
		FilmArea    = film;
		RangeClip   = clip;
		
		odometrystate    = (double[]) State.Clone();
		refodometrystate = (double[]) State.Clone();

		HasDataAssociation = false;
		DataAssociation    = new List<int>();
		Landmarks          = new List<double[]>();
		MappedMeasurements = new List<double[]>();

		WayOdometry = new TimedArray();
		WayPoints   = new TimedState();
		WayPoints.Add(Tuple.Create(0.0, Util.SClone(state)));

		HasSidebar  = false;
		WantsToStop = false;
	}

	/// <summary>
	/// Perform a deep copy of another general vehicle.
	/// </summary>
	/// <param name="that">Copied general vehicle.</param>
	/// <param name="copytrajectory">If true, the vehicle historic trajectory is copied. Relatively heavy operation.</param>
	protected Vehicle(Vehicle that, bool copytrajectory = false)
		: this(that.Location, 0, new double[3] {1, 0, 0})
	{
		this.Orientation        = that.Orientation;
		this.Graphics           = that.Graphics;
		this.SidebarWidth       = that.SidebarWidth;
		this.SidebarHeight      = that.SidebarHeight;
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
			this.WayPoints   = new TimedState(that.WayPoints);
			this.WayOdometry = new TimedArray(that.WayOdometry);
		}
		else {
			this.WayOdometry = new TimedArray();
			this.WayPoints = new TimedState();
			this.WayPoints.Add(Tuple.Create(0.0, Util.SClone(that.state)));
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
	/// <param name="dx">Moved distance from odometry in the local vertical movement-perpendicular direction since last timestep.</param>
	/// <param name="dy">Moved distance from odometry in the local horizontal movement-perpendicular direction since last timestep.</param>
	/// <param name="dz">Moved distance from odometry in the local depth movement-parallel direction since last timestep.</param>
	/// <param name="dyaw">Angle variation from odometry in the yaw coordinate since last timestep.</param>
	/// <param name="dpitch">Angle variation from odometry in the pitch coordinate since last timestep.</param>
	/// <param name="droll">Angle variation from odometry in the roll coordinate since last timestep.</param>
	public virtual void Update(GameTime time, double dx, double dy, double dz, double dyaw, double dpitch, double droll)
	{
		// note that the framework uses Yaw = Y, Pitch = X, Roll = Z => YXZ Tait-Bryan parametrization
		// this is equivalent to a plane pointing upwards with its wings on the X direction
		Quaternion dorientation   = Quaternion.CreateFromYawPitchRoll((float) dyaw, (float) dpitch, (float) droll);
		Quaternion neworientation = Orientation * dorientation;
		Quaternion midrotation    = Quaternion.Slerp(Orientation, neworientation, 0.5f);
		Quaternion dlocation      = midrotation * new Quaternion((float) dx, (float) dy, (float) dz, 0) * Quaternion.Conjugate(midrotation);

		Location    = new double[3] {X + dlocation.X, Y + dlocation.Y, Z + dlocation.Z};
		Orientation = Quaternion.Normalize(neworientation);

		// add noise to the odometry readings
		odometrystate = State.Add(time.ElapsedGameTime.TotalSeconds.Multiply(
		                              U.RandomGaussianVector(new double[7] {0, 0, 0, 0, 0, 0, 0}, MotionCovarianceQ)));
		// normalize orientation
		double[] orientation = new double[4] {odometrystate[3], odometrystate[4], odometrystate[5], odometrystate[6]};
		orientation.Normalize(true);
		odometrystate[3] = orientation[0];
		odometrystate[4] = orientation[1];
		odometrystate[5] = orientation[2];
		odometrystate[6] = orientation[3];

		WayPoints.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, Util.SClone(State)));
	}

	/// <summary>
	/// Obtain the cumulative odometry reading since the last call to this function.
	/// </summary>
	/// <returns>State diff.</returns>
	public virtual double[] ReadOdometry(GameTime time)
	{
		double[] odometry = Vehicle.StateDiff(odometrystate, refodometrystate);

		odometrystate     = Util.SClone(State);
		refodometrystate  = Util.SClone(State);

		WayOdometry.Add(Tuple.Create(time.TotalGameTime.TotalSeconds, Util.SClone(odometry)));

		return odometry;
	}

	/// <summary>
	/// Obtain several measurements from the hidden state.
	/// </summary>
	/// <returns>Pixel-range measurements.</returns>
	public abstract List<double[]> Measure();

	/// <summary>
	/// Remove all the localization history and start it again from the current position.
	/// </summary>
	public void ResetHistory()
	{
		WayOdometry.Clear();
		WayPoints.Clear();
		WayPoints.Add(Tuple.Create(0.0, Util.SClone(state)));
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

		Quaternion rotated = Orientation *
		                     new Quaternion((float) diff[0], (float) diff[1], (float) diff[2], 0) * Quaternion.Conjugate(Orientation);

		return new double[3] {X + rotated.X, Y + rotated.Y, Z + rotated.Z};
	}

	/// <summary>
	/// Compute the difference between the state of two vehicles.
	/// </summary>
	/// <param name="a">Final vehicle state.</param>
	/// <param name="b">Start vehicle state.</param>
	/// <returns>State difference in local coordinates: (dx, dy, dz, dyaw, dpitch, droll),
	/// such that b + ds = a.</returns>
	private static double[] StateDiff(double[] a, double[] b)
	{
		double[] aloc = new double[3] {a[0], a[1], a[2]};
		double[] bloc = new double[3] {b[0], b[1], b[2]};
		Quaternion arot = new Quaternion((float) a[4], (float) a[5], (float) a[6], (float) a[3]);
		Quaternion brot = new Quaternion((float) b[4], (float) b[5], (float) b[6], (float) b[3]);

		Quaternion dq          = Quaternion.Conjugate(brot) * arot;
		Quaternion midrotation = Quaternion.Slerp(arot, brot, 0.5f);
		double[]   dxglobal    = aloc.Subtract(bloc);
		Quaternion dx          = Quaternion.Conjugate(midrotation) *
		                             new Quaternion((float) dxglobal[0], (float) dxglobal[1], (float) dxglobal[2], 0) *
		                             midrotation;

		double x = dq.X;
		double y = dq.Y;
		double z = dq.Z;
		double w = dq.W;

		double dyaw   = Math.Atan2(2 * (w * y + x * z), 1 - 2 * (y * y + x * x));
		double dpitch = Math.Asin (2 * (w * x - z * y));
		double droll  = Math.Atan2(2 * (w * z + y * x), 1 - 2 * (x * x + z * z));

		return new double[6] {dx.X, dx.Y, dx.Z, dyaw, dpitch, droll};
	}

	/// <summary>
	/// Compute the difference between the state of two vehicles.
	/// </summary>
	/// <returns>State difference in local coordinates: (dx, dy, dz, dyaw, dpitch, droll),
	/// such that b + ds = a.</returns>
	/// <param name="a">Final vehicle.</param>
	/// <param name="b">Start vehicle.</param>
	public static double[] StateDiff(Vehicle a, Vehicle b)
	{
		return StateDiff(a.State, b.State);
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

		foreach (double[] measure in MappedMeasurements) {
			RenderMeasure(measure, camera);
		}

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

		double[] pos = camera.TransformH(Location);
		
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
		
		Quaternion x = Orientation * new Quaternion(0.2f, 0, 0, 0) * Quaternion.Conjugate(Orientation);
		pos = camera.TransformH(new double[3] {X + x.X, Y + x.Y, Z + x.Z});

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
		
		x   = Orientation * new Quaternion(0, 0.2f, 0, 0) * Quaternion.Conjugate(Orientation);
		pos = camera.TransformH(new double[3] {X + x.X, Y + x.Y, Z + x.Z});

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
		
		x   = Orientation * new Quaternion(0, 0, 0.2f, 0) * Quaternion.Conjugate(Orientation);
		pos = camera.TransformH(new double[3] {X + x.X, Y + x.Y, Z + x.Z});

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
			Quaternion local = Orientation *
			                       new Quaternion((float) frustum[i][0], (float) frustum[i][1], (float) frustum[i][2], 0) * Quaternion.Conjugate(Orientation);
			frustum[i] = camera.TransformH(new double[3] {local.X, local.Y, local.Z}.Add(Location));
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
		double[][] vertices = new double[WayPoints.Count][];

		for (int i = 0; i < WayPoints.Count; i++) {
			double[] w  = WayPoints[i].Item2;
			vertices[i] = camera.TransformH(new double[3] {w[0], w[1], w[2]});
		}

		Graphics.DrawUser2DPolygon(vertices, 0.02f, color, false);
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
