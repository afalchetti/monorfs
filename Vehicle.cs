// Vehicle.cs
// Abstract vehicle motion and measurement model.
// Part of MonoRFS
//
// Copyright (C) 2015 Angelo Falchetti
// All rights reserved.
// Any use, reproduction, distribution or copy of this
// work via any medium is strictly forbidden without
// express written consent from the author.

using System;
using System.Collections.Generic;

using Accord;
using Accord.Math;
using Accord.Statistics.Distributions.Univariate;
using AForge.Math.Random;
using AForge;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework;

namespace monorfs
{
/// <summary>
/// Vehicle model.
/// </summary>
public abstract class Vehicle : IDisposable
{
	/// <summary>
	/// Get the internal state as a vector.
	/// </summary>
	public double[] State { get; set; }

	/// <summary>
	/// Get the location x-axis coordinate.
	/// </summary>
	public double X
	{
		get { return  State[0]; }
		set { State[0] = value; }
	}

	/// <summary>
	/// Get the location y-axis coordinate.
	/// </summary>
	public double Y
	{
		get { return  State[1]; }
		set { State[1] = value; }
	}

	/// <summary>
	/// Get the location z-axis coordinate.
	/// </summary>
	public double Z
	{
		get { return  State[2]; }
		set { State[2] = value; }
	}

	/// <summary>
	/// Get the space location of the vehicle, i.e. its coordinates.
	/// </summary>
	public double[] Location
	{
		get { return new double[3] {State[0], State[1], State[2]}; }
		set { State[0] = value[0]; State[1] = value[1]; State[2] = value[2]; }
	}

	/// <summary>
	/// Get the rotation quaternion scalar component.
	/// </summary>
	public double W
	{
		get { return  State[3]; }
		set { State[3] = value; }
	}

	/// <summary>
	/// Get the rotation quaternion vector component.
	/// </summary>
	public double[] K
	{
		get { return new double[3] {State[4], State[5], State[6]}; }
		set { State[4] = value[0]; State[5] = value[1]; State[6] = value[2]; }
	}
	

	/// <summary>
	/// Get the orientation of the vehicle.
	/// </summary>
	public Quaternion Orientation
	{
		get { return new Quaternion((float) State[4], (float) State[5], (float) State[6], (float) State[3]); }
		set { State[3] = value.W; State[4] = value.X; State[5] = value.Y; State[6] = value.Z; }
	}

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
	public List<double[]> Waypoints { get; protected set; }
	
	/// <summary>
	/// Cached measurements from the update process for rendering purposes.
	/// </summary>
	protected List<double[]> MappedMeasurements { get; set; }

	/// <summary>
	/// Construct a vehicle with default constants.
	/// </summary>
	public Vehicle() : this(new double[3] {0, 0, 0}, 0, new double[3] {1, 0, 0}) {}

	/// <summary>
	/// Construct a new Vehicle object from its initial state.
	/// </summary>
	/// <param name="location">Spatial coordinates.</param>
	/// <param name="theta">Orientation angle.</param>
	/// <param name="axis">Orientation rotation axis.</param>
	public Vehicle(double[] location, double theta, double[] axis)
		: this(location, theta, axis, 240, new Rectangle(-160, -120, 320, 240), new Range(0.8f, 2*4f)) {}

	/// <summary>
	/// Construct a new Vehicle object from its initial state and appropiate constants.
	/// </summary>
	/// <param name="location">Spatial coordinates.</param>
	/// <param name="theta">Orientation angle.</param>
	/// <param name="axis">Orientation rotation axis.</param>
	/// <param name="focal">Focal lenghth.</param>
	/// <param name="film">Film area.</param>
	/// <param name="clip">Range clipping area.</param>
	public Vehicle(double[] location, double theta, double[] axis, double focal, Rectangle film, Range clip)
	{
		double w = Math.Cos(theta / 2);
		double d = Math.Sin(theta / 2);

		axis.Divide(axis.Euclidean(), true);
		
		State       = new double[7] {location[0], location[1], location[2], w, d * axis[0], d * axis[1], d * axis[2]};
		VisionFocal = focal;
		FilmArea    = film;
		RangeClip   = clip;

		MappedMeasurements = new List<double[]>();

		Waypoints = new List<double[]>();
		Waypoints.Add(new double[4] {0, X, Y, Z});
	}

	/// <summary>
	/// Perform a deep copy of another general vehicle.
	/// </summary>
	/// <param name="that">Copied general vehicle.</param>
	/// <param name="copytrajectory">If true, the vehicle historic trajectory is copied. Relatively heavy operation.</param>
	public Vehicle(Vehicle that, bool copytrajectory = false)
		: this(that.Location, 0, new double[3] {0, 0, 0})
	{
		this.Orientation        = that.Orientation;
		this.Graphics           = that.Graphics;
		this.SidebarWidth       = that.SidebarWidth;
		this.SidebarHeight      = that.SidebarHeight;
		this.MappedMeasurements = that.MappedMeasurements;

		if (copytrajectory) {
			this.Waypoints = new List<double[]>(that.Waypoints);
		}
		else {
			this.Waypoints = new List<double[]>();
			this.Waypoints.Add(new double[4] {0, that.X, that.Y, that.Z});
		}
	}
	
	/// <summary>
	/// Apply the motion model to the vehicle.
	/// </summary>
	/// <param name="time">Provides a snapshot of timing values.</param>
	/// <param name="dx">Moved distance from odometry in the local vertical movement-perpendicular direction since last timestep.</param>
	/// <param name="dy">Moved distance from odometry in the local horizontal movement-perpendicular direction since last timestep.</param>
	/// <param name="dz">Moved distance from odometry in the local depth movement-parallel direction since last timestep.</param>
	/// <param name="dyaw">Angle variation from odometry in the yaw coordinate since last timestep.</param>
	/// <param name="dpitch">Angle variation from odometry in the pitch coordinate since last timestep.</param>
	/// <param name="droll">Angle variation from odometry in the roll coordinate since last timestep.</param>
	public abstract void Update(GameTime time, double dx, double dy, double dz, double dyaw, double dpitch, double droll);

	/// <summary>
	/// Obtain several measurements from the hidden state.
	/// </summary>
	/// <returns>Pixel-range measurements.</returns>
	public abstract List<double[]> Measure();

	/// <summary>
	/// Render the vehicle on the graphics device.
	/// The graphics device must be ready, otherwise
	/// the method will throw an exception.
	/// </summary>
	/// <param name="camera">Camera rotation matrix.</param>
	public virtual void Render(double[,] camera)
	{
		RenderFOV(camera);
		RenderTrajectory(camera);
		RenderBody(camera);

		foreach (double[] measure in MappedMeasurements) {
			RenderMeasure(measure, camera);
		}
	}

	/// <summary>
	/// Render the vehicle physical body on the graphics device.
	/// <param name="camera">Camera rotation matrix.</param>
	/// </summary>
	public void RenderBody(double[,] camera)
	{
		const float halflen = 0.06f;
		
		Color innercolor =  Color.LightBlue;
		Color outercolor =  Color.Blue;

		double[] pos = camera.Multiply(Location);
		
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
		pos = camera.Multiply(new double[3] {X + x.X, Y + x.Y, Z + x.Z});

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
		pos = camera.Multiply(new double[3] {X + x.X, Y + x.Y, Z + x.Z});

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
		pos = camera.Multiply(new double[3] {X + x.X, Y + x.Y, Z + x.Z});

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
	/// <param name="camera">Camera rotation matrix.</param>
	public void RenderFOV(double[,] camera)
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
			frustum[i] = camera.Multiply(new double[3] {local.X, local.Y, local.Z}.Add(Location));
			frustum[i][2] = -100;
		}
		
		VertexPositionColor[] verticesA = new VertexPositionColor[8];
		VertexPositionColor[] verticesB = new VertexPositionColor[8];
		double[][]            wireA     = new double[4][];
		double[][]            wireB     = new double[4][];
		double[][]            wireC     = new double[4][];
		double[][]            wireD     = new double[4][];
		
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
		
		Graphics.DrawUser2DPolygon(wireA, 0.02f, outcolor, false);
		Graphics.DrawUser2DPolygon(wireB, 0.02f, outcolor, false);
		Graphics.DrawUser2DPolygon(wireC, 0.02f, outcolor, false);
		Graphics.DrawUser2DPolygon(wireD, 0.02f, outcolor, false);
	}

	/// <summary>
	/// Render the path that the vehicle has traveled so far.
	/// </summary>
	/// <param name="camera">Camera rotation matrix.</param>
	public void RenderTrajectory(double[,] camera)
	{
		double[][] vertices = new double[Waypoints.Count][];
		Color      color    = Color.Yellow;

		for (int i = 0; i < Waypoints.Count; i++) {
			vertices[i] = camera.Multiply(new double[3] {Waypoints[i][1], Waypoints[i][2], Waypoints[i][3]});
		}

		Graphics.DrawUser2DPolygon(vertices, 0.02f, color, false);
	}

	/// <summary>
	/// Simple point measurement rendering.
	/// </summary>
	/// <param name="measurement">Point measurement position.</param>
	/// <param name="camera">Camera rotation matrix.</param>
	private void RenderMeasure(double[] measurement, double[,] camera)
	{
		const float halflen = 0.03f;
		
		Color color =  Color.Crimson;

		measurement = camera.Multiply(measurement);
		
		double[][] vertices = new double[2][];

		vertices[0] = new double[] {measurement[0] - halflen, measurement[1] - halflen, measurement[2]};
		vertices[1] = new double[] {measurement[0] + halflen, measurement[1] + halflen, measurement[2]};
		
		Graphics.DrawUser2DPolygon(vertices, 0.01f, color, true);

		vertices[0] = new double[] {measurement[0] - halflen, measurement[1] + halflen, measurement[2]};
		vertices[1] = new double[] {measurement[0] + halflen, measurement[1] - halflen, measurement[2]};

		Graphics.DrawUser2DPolygon(vertices, 0.01f, color, true);
	}

	/// <summary>
	/// Render the sidebar info screen (extra interesting data).
	/// </summary>
	public abstract void RenderSide();

	/// <summary>
	/// Dispose of any resources.
	/// </summary>
	public virtual void Dispose() {}
}
}
