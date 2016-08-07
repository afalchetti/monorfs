// PRM3DMeasurer.cs
// Pose3D + PixelRangeMeasurement measurer
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
/// the Pose3D + PixelRangeMeasurement combination.
/// </summary>
public class PRM3DMeasurer : IMeasurer<PRM3DMeasurer, Pose3D, PixelRangeMeasurement>
{
	/// <summary>
	/// Sizes for the visibility ramp, one per measurement-space coordinate;
	/// the visibility grows linearly from zero at the original border
	/// to detectionProbability at the configured size.
	/// </summary>
	public static double[] VisibilityRamp { get { return Config.VisibilityRamp; } }

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
	/// Construct a new PRM3DMeasurer object from default camera parameters.
	/// </summary>
	public PRM3DMeasurer()
		: this(575.8156,
		       new Rectangle(-640 / 2, -480 / 2, 640, 480),
		       new Range(0.1f, 2f)) {}

	/// <summary>
	/// Construct a new PRM3DMeasurer object from its camera parameters.
	/// </summary>
	/// <param name="focal">Focal length.</param>
	/// <param name="film">Film area.</param>
	/// <param name="clip">Range clipping area.</param>
	public PRM3DMeasurer(double focal, Rectangle film, Range clip)
	{
		VisionFocal = focal;
		FilmArea    = film;
		RangeClip   = clip;
	}

	/// <summary>
	/// Obtain a linear representation for the measurer.
	/// </summary>
	/// <returns>Linear representation.</returns>
	public double[] ToLinear()
	{
		return new double[7] {VisionFocal, RangeClip.Min, RangeClip.Max,
		                     FilmArea.X, FilmArea.Y, FilmArea.Width, FilmArea.Height};
	}

	/// <summary>
	/// Retrieve the measurer from a linear representation.
	/// </summary>
	/// <param name="linear">Linear representation.</param>
	/// <returns>Measurer.</returns>
	public PRM3DMeasurer FromLinear(double[] linear)
	{
		if (linear.Length != 7) {
			throw new ArgumentException("Linear representation must have exactly seven parameters.");
		}

		double    focal = linear[0];
		Range     clip  = new Range((float) linear[1], (float) linear[2]);
		Rectangle film  = new Rectangle((int) linear[3], (int) linear[4], (int) linear[5], (int) linear[6]);

		return new PRM3DMeasurer(focal, film, clip);
	}

	/// <summary>
	/// Compute the volume of the visible area (in measurement space).
	/// </summary>
	public double Volume()
	{
		return FilmArea.Height * FilmArea.Width * RangeClip.Length;
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
	public PixelRangeMeasurement MeasurePerfect(Pose3D pose, double[] landmark)
	{
		double[]   diff  = landmark.Subtract(pose.Location);
		Quaternion local = pose.Orientation.Conjugate() *
		                       new Quaternion(0, diff[0], diff[1], diff[2]) * pose.Orientation;

		double range  = Math.Sign(local.Z) * diff.Euclidean();
		double px     = VisionFocal * local.X / local.Z;
		double py     = VisionFocal * local.Y / local.Z;

		return new PixelRangeMeasurement(px, py, range);
	}

	/// <summary>
	/// Obtain the jacobian of the measurement model wrt. the landmark.
	/// </summary>
	/// <param name="pose">Pose from which the measurement was made.</param>
	/// <param name="landmark">Landmark 3d location against which the measurement is performed.</param>
	/// <returns>Measurement model linearization jacobian.</returns>
	public double[][] MeasurementJacobianL(Pose3D pose, double[] landmark)
	{
		double[]   diff  = landmark.Subtract(pose.Location);
		Quaternion local = pose.Orientation.Conjugate() *
		                       new Quaternion(0, diff[0], diff[1], diff[2]) * pose.Orientation;

		// the jacobian of the homography projection part is given by
		// f/z I 0
		// sgn(loc.z) X/|X|
		// (arbitrarily choosing -1 as the sign for zero)
		double mag = ((local.Z > 0) ? 1 : -1) * Math.Sqrt(local.X * local.X + local.Y * local.Y + local.Z * local.Z);
		double[][] jprojection = {new double[3] {VisionFocal / local.Z, 0,                     -VisionFocal * local.X / (local.Z * local.Z)},
		                          new double[3] {0,                     VisionFocal / local.Z, -VisionFocal * local.Y / (local.Z * local.Z)},
		                          new double[3] {local.X / mag,         local.Y / mag,         local.Z /mag}};

		// the jacobian of the change of coordinates part of
		// the measurement process is the conjugate of the rotation matrix
		double[][] jrotation = pose.Orientation.Conjugate().ToMatrix();

		return jprojection.Multiply(jrotation);
	}

	/// <summary>
	/// Obtain the jacobian of the measurement model wrt. the pose.
	/// </summary>
	/// <param name="pose">Pose from which the measurement was made.</param>
	/// <param name="landmark">Landmark 3d location against which the measurement is performed.</param>
	/// <returns>Measurement model linearization jacobian.</returns>
	public double[][] MeasurementJacobianP(Pose3D pose, double[] landmark)
	{
		double[]   diff  = landmark.Subtract(pose.Location);
		Quaternion local = pose.Orientation.Conjugate() *
		                       new Quaternion(0, diff[0], diff[1], diff[2]) * pose.Orientation;

		// the jacobian of the homography projection part is given by
		// f/z I 0
		// sgn(loc.z) X/|X|
		// (arbitrarily choosing -1 as the sign for zero)
		double mag = ((local.Z > 0) ? 1 : -1) * Math.Sqrt(local.X * local.X + local.Y * local.Y + local.Z * local.Z);
		double[][] jprojection = {new double[3] {VisionFocal / local.Z, 0,                     -VisionFocal * local.X / (local.Z * local.Z)},
		                          new double[3] {0,                     VisionFocal / local.Z, -VisionFocal * local.Y / (local.Z * local.Z)},
		                          new double[3] {local.X / mag,         local.Y / mag,         local.Z /mag}};

		// the jacobian of the change of coordinates part of
		// the measurement process is given by
		// -Prot'    -Prot' [l - Ploc]_x
		double[][] jlocation = (-1.0).Multiply(pose.Orientation.Conjugate().ToMatrix());
		double[][] jrotation = jlocation.Multiply(Util.CrossProductMatrix(diff));

		double[][] jlocal = jlocation.Concatenate(jrotation);

		return jprojection.Multiply(jlocal);
	}

	/// <summary>
	/// Given a measurement and a landmark, find the pose that best relates the two.
	/// The problem is underconstrained, i.e. there is a solution for every rotation.
	/// The output will be a compromise between rotation and translation distance to
	/// the initial estimate.
	/// </summary>
	/// <param name="pose0">Initial estimate.</param>
	/// <param name="measurement">Measurement.</param>
	/// <param name="landmark">Landmark.</param>
	/// <returns>Best fit for the landmark-measurement pair.</returns>
	public Pose3D FitToMeasurement(Pose3D pose0, PixelRangeMeasurement measurement, double[] landmark)
	{
		double[]   diff          = landmark.Subtract(pose0.Location);
		double[][] rotmatrix     = pose0.Orientation.Conjugate().ToMatrix();
		double[]   landmarklocal = rotmatrix.Multiply(diff);
		double[]   measurelocal  = new double[3];
		double     invfocal      = 1.0 / VisionFocal;

		measurelocal[2] = measurement.Range /
		                  Math.Sqrt(1 + (measurement.X * measurement.X + measurement.Y * measurement.Y) *
		                                invfocal * invfocal);
		measurelocal[0] = measurement.X * measurelocal[2] * invfocal;
		measurelocal[1] = measurement.Y * measurelocal[2] * invfocal;

		double[] normallandmark = landmarklocal.Normalize();
		double[] normalmeasure  = measurelocal.Normalize();

		Quaternion alignrot = Quaternion.VectorRotator(normallandmark, normalmeasure);
		Quaternion rotation = alignrot.Conjugate() * pose0.Orientation;
		double[]   location = landmark.Subtract(rotation.ToMatrix().Multiply(measurelocal));

		return new Pose3D(location, rotation);
	}

	/// <summary>
	/// Obtain a noise measurement, uniformly random in the visible area.
	/// </summary>
	/// <returns>Random measurement.</returns>
	public PixelRangeMeasurement RandomMeasure()
	{
		double px    = Util.Uniform.Next() * FilmArea.Width + FilmArea.Left;
		double py    = Util.Uniform.Next() * FilmArea.Height + FilmArea.Top;
		double range = Util.Uniform.Next() * RangeClip.Length + RangeClip.Min;

		return new PixelRangeMeasurement(px, py, range);
	}

	/// <summary>
	/// Find if a given ladmark is visible from the current pose of the vehicle
	/// using pixel-range coordinates to express the landmark.
	/// </summary>
	/// <param name="landmark">Queried landmark in pixel-range coordinates.</param>
	/// <returns>True if the landmark is visible; false otherwise.</returns>
	public virtual bool VisibleM(PixelRangeMeasurement landmark)
	{
		return FilmArea.Left < landmark.X     && landmark.X     < FilmArea.Right &&
		       FilmArea.Top  < landmark.Y     && landmark.Y     < FilmArea.Bottom &&
		       RangeClip.Min < landmark.Range && landmark.Range < RangeClip.Max;
	}

	/// <summary>
	/// Find if a landmark is visible in measurement space and
	/// return a fuzzy value for points near the border of the visible region.
	/// </summary>
	/// <param name="landmark">Queried landmark in pixel-range coordinates.</param>
	/// <returns>True if the landmark is visible; false otherwise.</returns>
	public virtual double FuzzyVisibleM(PixelRangeMeasurement landmark)
	{
		double minwdistance = double.PositiveInfinity;

		minwdistance = Math.Min(minwdistance, (landmark.X - FilmArea.Left)  / VisibilityRamp[0]);
		minwdistance = Math.Min(minwdistance, (FilmArea.Right - landmark.X) / VisibilityRamp[0]);

		minwdistance = Math.Min(minwdistance, (landmark.Y  - FilmArea.Top)   / VisibilityRamp[1]);
		minwdistance = Math.Min(minwdistance, (FilmArea.Bottom - landmark.Y) / VisibilityRamp[1]);

		minwdistance = Math.Min(minwdistance, (landmark.Range - RangeClip.Min)  / VisibilityRamp[2]);
		minwdistance = Math.Min(minwdistance, (RangeClip.Max  - landmark.Range) / VisibilityRamp[2]);

		return Math.Max(0, Math.Min(1, minwdistance));
	}

	/// <summary>
	/// Transform a measurement vector in measurement space into 3D space vector.
	/// </summary>
	/// <param name="pose">Pose from which the measurement was made.</param>
	/// <param name="measurement">Measurement expressed as pixel-range.</param>
	/// <returns>Measurement expressed in 3D space.</returns>
	public double[] MeasureToMap(Pose3D pose, PixelRangeMeasurement measurement)
	{
		double   px    = measurement.X;
		double   py    = measurement.Y;
		double   range = measurement.Range;

		double   alpha = range / Math.Sqrt(VisionFocal * VisionFocal + px * px + py * py);
		double[] diff  = new double[3] {alpha * px, alpha * py, alpha * VisionFocal};

		Quaternion rotated = pose.Orientation *
		                         new Quaternion(0, diff[0], diff[1], diff[2]) * pose.Orientation.Conjugate();

		return new double[3] {pose.X + rotated.X, pose.Y + rotated.Y, pose.Z + rotated.Z};
	}

	/// <summary>
	/// Render the vehicle physical body on the graphics device.
	/// </summary>
	/// <param name="graphics">Graphic context.</param>
	/// <param name="pose">Pose.</param>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public void RenderBody(GraphicsDevice graphics, Pose3D pose, double[][] camera)
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

		Quaternion x = pose.Orientation * new Quaternion(0, 0.2f, 0, 0) * pose.Orientation.Conjugate();
		pos          = camera.TransformH(new double[3] {pose.X + x.X, pose.Y + x.Y, pose.Z + x.Z});

		outvertices[0] = new double[3] {pos[0] - 0.4*halflen, pos[1] - 0.4*halflen, pos[2]};
		outvertices[1] = new double[3] {pos[0] - 0.4*halflen, pos[1] + 0.4*halflen, pos[2]};
		outvertices[2] = new double[3] {pos[0] + 0.4*halflen, pos[1] + 0.4*halflen, pos[2]};
		outvertices[3] = new double[3] {pos[0] + 0.4*halflen, pos[1] - 0.4*halflen, pos[2]};

		invertices[0] = new VertexPositionColor(outvertices[0].ToVector3(), innercolor);
		invertices[1] = new VertexPositionColor(outvertices[1].ToVector3(), innercolor);
		invertices[2] = new VertexPositionColor(outvertices[3].ToVector3(), innercolor);
		invertices[3] = new VertexPositionColor(outvertices[2].ToVector3(), innercolor);

		graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, invertices, 0, invertices.Length - 2);
		graphics.DrawUser2DPolygon(outvertices, 0.02f, Color.Blue, true);

		x   = pose.Orientation * new Quaternion(0, 0, 0.2f, 0) * pose.Orientation.Conjugate();
		pos = camera.TransformH(new double[3] {pose.X + x.X, pose.Y + x.Y, pose.Z + x.Z});

		outvertices[0] = new double[3] {pos[0] - 0.2*halflen, pos[1] - 0.2*halflen, pos[2]};
		outvertices[1] = new double[3] {pos[0] - 0.2*halflen, pos[1] + 0.2*halflen, pos[2]};
		outvertices[2] = new double[3] {pos[0] + 0.2*halflen, pos[1] + 0.2*halflen, pos[2]};
		outvertices[3] = new double[3] {pos[0] + 0.2*halflen, pos[1] - 0.2*halflen, pos[2]};

		invertices[0] = new VertexPositionColor(outvertices[0].ToVector3(), innercolor);
		invertices[1] = new VertexPositionColor(outvertices[1].ToVector3(), innercolor);
		invertices[2] = new VertexPositionColor(outvertices[3].ToVector3(), innercolor);
		invertices[3] = new VertexPositionColor(outvertices[2].ToVector3(), innercolor);

		graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, invertices, 0, invertices.Length - 2);
		graphics.DrawUser2DPolygon(outvertices, 0.02f, Color.Yellow, true);

		x   = pose.Orientation * new Quaternion(0, 0, 0, 0.2f) * pose.Orientation.Conjugate();
		pos = camera.TransformH(new double[3] {pose.X + x.X, pose.Y + x.Y, pose.Z + x.Z});

		outvertices[0] = new double[3] {pos[0] - 0.8*halflen, pos[1] - 0.8*halflen, pos[2]};
		outvertices[1] = new double[3] {pos[0] - 0.8*halflen, pos[1] + 0.8*halflen, pos[2]};
		outvertices[2] = new double[3] {pos[0] + 0.8*halflen, pos[1] + 0.8*halflen, pos[2]};
		outvertices[3] = new double[3] {pos[0] + 0.8*halflen, pos[1] - 0.8*halflen, pos[2]};

		invertices[0] = new VertexPositionColor(outvertices[0].ToVector3(), innercolor);
		invertices[1] = new VertexPositionColor(outvertices[1].ToVector3(), innercolor);
		invertices[2] = new VertexPositionColor(outvertices[3].ToVector3(), innercolor);
		invertices[3] = new VertexPositionColor(outvertices[2].ToVector3(), innercolor);

		graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, invertices, 0, invertices.Length - 2);
		graphics.DrawUser2DPolygon(outvertices, 0.02f, Color.Red, true);
	}

	/// <summary>
	/// Render the Field-of-View cone on the graphics device.
	/// </summary>
	/// <param name="graphics">Graphic context.</param>
	/// <param name="pose">Pose.</param>
	/// <param name="camera">Camera 4d transform matrix.</param>
	public void RenderFOV(GraphicsDevice graphics, Pose3D pose, double[][] camera)
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
			Quaternion local = pose.Orientation *
			                       new Quaternion(0, frustum[i][0], frustum[i][1], frustum[i][2]) *
			                       pose.Orientation.Conjugate();
			frustum[i] = camera.TransformH(new double[3] {local.X, local.Y, local.Z}.Add(pose.Location));
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

		graphics.DrawUser2DPolygon(wireA, 0.02f, outcolor, false);
		graphics.DrawUser2DPolygon(wireB, 0.02f, outcolor, false);
		graphics.DrawUser2DPolygon(wireC, 0.02f, outcolor, false);
		graphics.DrawUser2DPolygon(wireD, 0.02f, outcolor, false);

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

		graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, verticesA, 0, 2);
		graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, verticesA, 4, 2);
		graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, verticesB, 0, 2);
		graphics.DrawUserPrimitives(PrimitiveType.TriangleStrip, verticesB, 4, 2);
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
		return VisionFocal    .ToString(format) + " "
		     + RangeClip.Min  .ToString(format) + " "
		     + RangeClip.Max  .ToString(format) + " "
		     + FilmArea.Left  .ToString(format) + " "
		     + FilmArea.Top   .ToString(format) + " "
		     + FilmArea.Width .ToString(format) + " "
		     + FilmArea.Height.ToString(format);
	}
}
}
