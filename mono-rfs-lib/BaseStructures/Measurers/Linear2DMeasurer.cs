// Linear2DMeasurer.cs
// Very simple 2D measurer
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

using AForge;
using Accord.Math;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace monorfs
{
/// <summary>
/// Measuring and related methods and configurations for
/// a very simple LinearPose2D + LinearMeasurement2D combination.
/// </summary>
public class Linear2DMeasurer : IMeasurer<Linear2DMeasurer, LinearPose2D, LinearMeasurement2D>
{
	/// <summary>
	/// Sizes for the visibility ramp, one per measurement-space coordinate;
	/// the visibility grows linearly from zero at the original border
	/// to detectionProbability at the configured size.
	/// </summary>
	public static double[] VisibilityRamp { get { return Config.VisibilityRamp; } }

	/// <summary>
	/// Maximum visible distance (in uniform norm, i.e. a square of 2 * Range around the pose).
	/// </summary>
	public double Range { get; private set; }

	/// <summary>
	/// Construct a new Linear2DMeasurer object from default camera parameters.
	/// </summary>
	public Linear2DMeasurer()
		: this(2.0) {}

	/// <summary>
	/// Construct a new Linear2DMeasurer object from its camera parameters.
	/// </summary>
	/// <param name="range">Visible range.</param>
	public Linear2DMeasurer(double range)
	{
		Range = range;
	}

	/// <summary>
	/// Obtain a linear representation for the measurer.
	/// </summary>
	/// <returns>Linear representation.</returns>
	public double[] ToLinear()
	{
		return new double[1] {Range};
	}

	/// <summary>
	/// Retrieve the measurer from a linear representation.
	/// </summary>
	/// <param name="linear">Linear representation.</param>
	/// <returns>Measurer.</returns>
	public Linear2DMeasurer FromLinear(double[] linear)
	{
		if (linear.Length != 1) {
			throw new ArgumentException("Linear representation must have exactly one parameter.");
		}

		return new Linear2DMeasurer(linear[0]);
	}

	/// <summary>
	/// Compute the volume of the visible area (in measurement space).
	/// </summary>
	public double Volume()
	{
		return 4 * Range * Range;
	}

	/// <summary>
	/// Obtain a measurement from the given pose.
	/// It does not use any randomness or misdetection.
	/// It follows a pixel-range model where the range and
	/// pixel index from a map landmark m are:
	/// 
	/// [r, px, py] = [|m-x|, (f x/z)L, (f y/z)_L] + N(0, R)
	/// 
	/// where |a-b| is the euclidean distance between a and b and
	/// (.)_L is performed on the local axis.
	/// </summary>
	/// <param name="pose">Pose from which the measurement was made.</param>
	/// <param name="landmark">Landmark 3d location against which the measurement is performed.</param>
	/// <returns>Measurement.</returns>
	public LinearMeasurement2D MeasurePerfect(LinearPose2D pose, double[] landmark)
	{
		return new LinearMeasurement2D(landmark[0] - pose.X, landmark[1] - pose.Y);
	}

	/// <summary>
	/// Obtain the jacobian of the measurement model wrt. the landmark.
	/// </summary>
	/// <param name="pose">Pose from which the measurement was made.</param>
	/// <param name="landmark">Landmark 3d location against which the measurement is performed.</param>
	/// <returns>Measurement model linearization jacobian.</returns>
	public double[][] MeasurementJacobianL(LinearPose2D pose, double[] landmark)
	{
		return new double[2][] { new double[3] {1, 0, 0},
		                         new double[3] {0, 1, 0}};
	}

	/// <summary>
	/// Obtain the jacobian of the measurement model wrt. the pose.
	/// </summary>
	/// <param name="pose">Pose from which the measurement was made.</param>
	/// <param name="landmark">Landmark 3d location against which the measurement is performed.</param>
	/// <returns>Measurement model linearization jacobian.</returns>
	public double[][] MeasurementJacobianP(LinearPose2D pose, double[] landmark)
	{
		return new double[2][] { new double[2] {-1, 0},
		                         new double[2] {0, -1} };
	}

	/// <summary>
	/// Given a measurement and a landmark, find the pose that best relates the two.
	/// </summary>
	/// <param name="pose0">Initial estimate.</param>
	/// <param name="measurement">Measurement.</param>
	/// <param name="landmark">Landmark.</param>
	/// <returns>Best fit for the landmark-measurement pair.</returns>
	public LinearPose2D FitToMeasurement(LinearPose2D pose0, LinearMeasurement2D measurement, double[] landmark)
	{
		return new LinearPose2D(new double[2] {landmark[0] - measurement.X, landmark[1] - measurement.Y});
	}

	/// <summary>
	/// Obtain a noise measurement, uniformly random in the visible area.
	/// </summary>
	/// <returns>Random measurement.</returns>
	public LinearMeasurement2D RandomMeasure()
	{
		double x = Util.Uniform.Next() * 2 * Range - Range;
		double y = Util.Uniform.Next() * 2 * Range - Range;

		return new LinearMeasurement2D(x, y);
	}

	/// <summary>
	/// Find if a given ladmark is visible from the current pose of the vehicle
	/// using pixel-range coordinates to express the landmark.
	/// </summary>
	/// <param name="landmark">Queried landmark in pixel-range coordinates.</param>
	/// <returns>True if the landmark is visible; false otherwise.</returns>
	public virtual bool VisibleM(LinearMeasurement2D landmark)
	{
		return -Range < landmark.X && landmark.X < Range &&
		       -Range < landmark.Y && landmark.Y < Range;
	}

	/// <summary>
	/// Find if a landmark is visible in measurement space and
	/// return a fuzzy value for points near the border of the visible region.
	/// </summary>
	/// <param name="landmark">Queried landmark in pixel-range coordinates.</param>
	/// <returns>True if the landmark is visible; false otherwise.</returns>
	public virtual double FuzzyVisibleM(LinearMeasurement2D landmark)
	{
		double minwdistance = double.PositiveInfinity;

		minwdistance = Math.Min(minwdistance, (landmark.X - -Range) / VisibilityRamp[0]);
		minwdistance = Math.Min(minwdistance, (Range - landmark.X)  / VisibilityRamp[0]);

		minwdistance = Math.Min(minwdistance, (landmark.Y  - -Range) / VisibilityRamp[1]);
		minwdistance = Math.Min(minwdistance, (Range - landmark.Y)   / VisibilityRamp[1]);

		return Math.Max(0, Math.Min(1, minwdistance));
	}

	/// <summary>
	/// Transform a measurement vector in measurement space into 3D space vector.
	/// </summary>
	/// <param name="pose">Pose from which the measurement was made.</param>
	/// <param name="measurement">Measurement expressed as pixel-range.</param>
	/// <returns>Measurement expressed in 3D space.</returns>
	public double[] MeasureToMap(LinearPose2D pose, LinearMeasurement2D measurement)
	{
		return new double[3] {pose.X + measurement.X, pose.Y + measurement.Y, 0};
	}

	/// <summary>
	/// Render the vehicle physical body on the graphics device.
	/// </summary>
	/// <param name="graphics">Graphic context.</param>
	/// <param name="pose">Pose.</param>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public void RenderBody(GraphicsDevice graphics, LinearPose2D pose, double[][] camera)
	{
		const float halflen = 0.06f;

		Color innercolor =  Color.LightBlue;
		Color outercolor =  Color.Blue;

		double[] pos = camera.TransformH(pose.Location);

		VertexPositionColor[] invertices  = new VertexPositionColor[4];
		double[][]            outvertices = new double[4][];

		outvertices[0] = new double[3] {pos[0] - halflen, pos[1] - halflen, pos[2]};
		outvertices[1] = new double[3] {pos[0] - halflen, pos[1] + halflen, pos[2]};
		outvertices[2] = new double[3] {pos[0] + halflen, pos[1] + halflen, pos[2]};
		outvertices[3] = new double[3] {pos[0] + halflen, pos[1] - halflen, pos[2]};

		invertices[0] = new VertexPositionColor(outvertices[0].ToVector3(), innercolor);
		invertices[1] = new VertexPositionColor(outvertices[1].ToVector3(), innercolor);
		invertices[2] = new VertexPositionColor(outvertices[3].ToVector3(), innercolor);
		invertices[3] = new VertexPositionColor(outvertices[2].ToVector3(), innercolor);

		graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, invertices, 0, invertices.Length - 2);
		graphics.DrawUser2DPolygon(outvertices, 0.02f, outercolor, true);
	}

	/// <summary>
	/// Render the Field-of-View cone on the graphics device.
	/// </summary>
	/// <param name="graphics">Graphic context.</param>
	/// <param name="pose">Pose.</param>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public void RenderFOV(GraphicsDevice graphics, LinearPose2D pose, double[][] camera)
	{
		Color incolor  = Color.LightGreen; incolor .A = 30;
		Color outcolor = Color.DarkGreen;  outcolor.A = 30;

		double[][]            frustum     = new double[4][];
		VertexPositionColor[] invertices  = new VertexPositionColor[4];

		frustum[0] = camera.TransformH(new double[3] {pose.X - Range, pose.Y - Range, 0});
		frustum[1] = camera.TransformH(new double[3] {pose.X - Range, pose.Y + Range, 0});
		frustum[2] = camera.TransformH(new double[3] {pose.X + Range, pose.Y + Range, 0});
		frustum[3] = camera.TransformH(new double[3] {pose.X + Range, pose.Y - Range, 0});

		invertices[0] = new VertexPositionColor(frustum[0].ToVector3(), incolor);
		invertices[1] = new VertexPositionColor(frustum[1].ToVector3(), incolor);
		invertices[2] = new VertexPositionColor(frustum[3].ToVector3(), incolor);
		invertices[3] = new VertexPositionColor(frustum[2].ToVector3(), incolor);

		graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, invertices, 0, invertices.Length - 2);
		graphics.DrawUser2DPolygon(frustum, 0.02f, outcolor, true);
	}

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
	/// <param name="format">Stirng formt for double values.</param>
	/// <returns>Simulated vehicle descriptor string.</returns>
	public string ToString(string format)
	{
		return "(" + Range.ToString(format) + ")";
	}
}
}
